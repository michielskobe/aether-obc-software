/**
 * @file    iridium.c
 * @brief   Iridium 9603N SBD driver implementation.
 *
 * Architecture summary
 * ────────────────────
 *  • UART4 RX  : DMA circular + IDLE-line interrupt.
 *                HAL_UARTEx_ReceiveToIdle_DMA() fills dma_rx_buf[].
 *                The RxEventCallback copies bytes into a software ring buffer.
 *  • UART4 TX  : DMA normal (one transfer per AT command or payload).
 *                TxCpltCallback sets ctx->tx_done so the SM can advance.
 *  • Session SM: Non-blocking; driven by Iridium_SessionTick() every task tick.
 *                Steps: AT → SBDWB cmd → payload write → result → SBDIX → done.
 *  • On error  : Session is dropped silently; SM returns to IDLE.
 */

#include "iridium.h"

#include <string.h>
#include <stdio.h>
#include "cmsis_os2.h"

/* ─────────────────────────────────────────────────────────────
 * Private helpers — ring buffer
 * ───────────────────────────────────────────────────────────── */

/** Number of bytes currently available to read from the ring buffer. */
static inline uint16_t ring_available(const IridiumCtx_t *ctx)
{
    /* head is volatile; read once with a barrier to avoid torn reads */
    uint16_t h = ctx->ring_head;
    __DMB();
    return (uint16_t)((h - ctx->ring_tail) & (IRIDIUM_RING_LEN - 1u));
}

/**
 * @brief  Copy up to `len` bytes from the ring buffer into `dst`.
 * @return Number of bytes actually copied.
 */
static uint16_t ring_read(IridiumCtx_t *ctx, uint8_t *dst, uint16_t len)
{
    uint16_t avail = ring_available(ctx);
    if (avail < len) len = avail;

    for (uint16_t i = 0; i < len; i++) {
        dst[i] = ctx->ring_buf[ctx->ring_tail & (IRIDIUM_RING_LEN - 1u)];
        ctx->ring_tail++;
    }
    return len;
}

/**
 * @brief  Search ring buffer for a token string without consuming bytes.
 * @return true if `token` is present in the unread portion of the ring buffer.
 */
static bool ring_contains(const IridiumCtx_t *ctx, const char *token)
{
    uint16_t avail = ring_available(ctx);
    uint16_t tlen  = (uint16_t)strlen(token);
    if (avail < tlen) return false;

    for (uint16_t i = 0; i <= avail - tlen; i++) {
        bool match = true;
        for (uint16_t j = 0; j < tlen; j++) {
            uint8_t b = ctx->ring_buf[(ctx->ring_tail + i + j) & (IRIDIUM_RING_LEN - 1u)];
            if (b != (uint8_t)token[j]) { match = false; break; }
        }
        if (match) return true;
    }
    return false;
}

/** Flush all unread bytes from the ring buffer. */
static void ring_flush(IridiumCtx_t *ctx)
{
    ctx->ring_tail = ctx->ring_head;
}

/* ─────────────────────────────────────────────────────────────
 * Private helpers — circular history buffer
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief  Flatten history buffer into ctx->tx_payload[], oldest packet first.
 */
static void circ_snapshot(IridiumCtx_t *ctx)
{
    uint8_t start = ctx->hist.head; /* oldest slot = next write position after wrap */
    for (uint8_t i = 0; i < IRIDIUM_HIST_LEN; i++) {
        uint8_t idx = (start + i) % IRIDIUM_HIST_LEN;
        memcpy(&ctx->tx_payload[i * IRIDIUM_PKT_BYTES],
               &ctx->hist.slots[idx],
               IRIDIUM_PKT_BYTES);
    }
}

/* ─────────────────────────────────────────────────────────────
 * Private helpers — UART TX (non-blocking, DMA)
 * ───────────────────────────────────────────────────────────── */

extern UART_HandleTypeDef huart4;   /* Provided by CubeMX-generated usart.c */

/**
 * @brief  Send `len` bytes via DMA TX.  Polls ctx->tx_done with a timeout
 *         to ensure the previous transfer completed before starting a new one.
 *         Called only from the task context (never from ISR).
 * @return true on success, false on timeout.
 */
static bool uart_send(IridiumCtx_t *ctx, const uint8_t *data, uint16_t len)
{
    /* Wait for any previous DMA TX to complete (max 5 s) */
    uint32_t deadline = osKernelGetTickCount() + 5000u;
    while (!ctx->tx_done) {
        if (osKernelGetTickCount() > deadline) return false;
        osDelay(1);
    }
    ctx->tx_done = false;
    HAL_UART_Transmit_DMA(&huart4, (uint8_t *)data, len);
    return true;
}

/**
 * @brief  Send a null-terminated AT command string via DMA TX.
 */
static bool at_send(IridiumCtx_t *ctx, const char *cmd)
{
    return uart_send(ctx, (const uint8_t *)cmd, (uint16_t)strlen(cmd));
}

/* ─────────────────────────────────────────────────────────────
 * Private helpers — session SM utilities
 * ───────────────────────────────────────────────────────────── */

static inline bool session_timed_out(const IridiumCtx_t *ctx, uint32_t timeout_ms)
{
    return (osKernelGetTickCount() - ctx->session_tick) >= timeout_ms;
}

static void session_advance(IridiumCtx_t *ctx, IridiumSessionState_t next)
{
    ctx->session_state = next;
    ctx->session_tick  = osKernelGetTickCount();
    ring_flush(ctx); /* discard any stale modem output before next command */
}

static void session_fail(IridiumCtx_t *ctx)
{
    ctx->session_state = IRIDIUM_SESSION_IDLE;
    ring_flush(ctx);
    /* Drop silently — next 15 s cycle will try again */
}

/* ─────────────────────────────────────────────────────────────
 * Public API — init
 * ───────────────────────────────────────────────────────────── */

void Iridium_Init(IridiumCtx_t *ctx, UART_HandleTypeDef *huart)
{
    (void)huart; /* huart4 is referenced via the extern above */
    memset(ctx, 0, sizeof(*ctx));
    ctx->tx_done = true; /* TX is free at startup */

    /* Arm DMA circular RX with IDLE-line detection */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, ctx->dma_rx_buf, IRIDIUM_DMA_RX_LEN);

    /* Suppress half-transfer interrupts — we only want IDLE + full-transfer events */
    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
}

/* ─────────────────────────────────────────────────────────────
 * Public API — circular history buffer
 * ───────────────────────────────────────────────────────────── */

void Iridium_PushPacket(IridiumCtx_t *ctx, const data_packet_t *pkt)
{
    ctx->hist.slots[ctx->hist.head] = *pkt;
    ctx->hist.head = (ctx->hist.head + 1u) % IRIDIUM_HIST_LEN;
}

/* ─────────────────────────────────────────────────────────────
 * Public API — session control
 * ───────────────────────────────────────────────────────────── */

bool Iridium_SessionStart(IridiumCtx_t *ctx)
{
    if (ctx->session_state != IRIDIUM_SESSION_IDLE)      return false; /* session already runs */

    circ_snapshot(ctx);
    session_advance(ctx, IRIDIUM_SESSION_AT_CHECK);
    at_send(ctx, "AT\r");
    return true;
}

/* ─────────────────────────────────────────────────────────────
 * Public API — session tick (call every IridiumTask tick)
 * ───────────────────────────────────────────────────────────── */

void Iridium_SessionTick(IridiumCtx_t *ctx)
{
    switch (ctx->session_state)
    {
    /* ── Nothing to do ─────────────────────────────────────── */
    case IRIDIUM_SESSION_IDLE:
        break;

    /* ── Step 1: AT — modem alive check ────────────────────── */
    case IRIDIUM_SESSION_AT_CHECK:
        if (ring_contains(ctx, "OK")) {
            session_advance(ctx, IRIDIUM_SESSION_SBDWB_CMD);
            static char cmd[20];
            snprintf(cmd, sizeof(cmd), "AT+SBDWB=%u\r", IRIDIUM_PAYLOAD_BYTES);
            at_send(ctx, cmd);
        } else if (session_timed_out(ctx, IRIDIUM_RESP_TIMEOUT_MS)) {
            session_fail(ctx);
        }
        break;

    /* ── Step 2: SBDWB command — wait for READY ────────────── */
    case IRIDIUM_SESSION_SBDWB_CMD:
        if (ring_contains(ctx, "READY")) {
            /*
             * Build the SBDWB binary block:
             *   [ 336 bytes payload ][ 2 bytes checksum, big-endian ]
             *
             * Checksum = sum of all payload bytes, truncated to 16 bits.
             * The 9603N verifies this and replies "0\r\n" on success.
             *
             * We reuse the tail of tx_payload[] as scratch for the 2 checksum
             * bytes to avoid a separate stack buffer.  A local 338-byte array
             * on the task stack is also fine if you prefer that.
             */
            uint32_t cksum = 0;
            for (uint16_t i = 0; i < IRIDIUM_PAYLOAD_BYTES; i++) {
                cksum += ctx->tx_payload[i];
            }
            /* Append checksum to a local frame — keep stack usage bounded */
            static uint8_t sbdwb_frame[IRIDIUM_PAYLOAD_BYTES + 2u];
            memcpy(sbdwb_frame, ctx->tx_payload, IRIDIUM_PAYLOAD_BYTES);
            sbdwb_frame[IRIDIUM_PAYLOAD_BYTES]      = (uint8_t)((cksum >> 8) & 0xFFu);
            sbdwb_frame[IRIDIUM_PAYLOAD_BYTES + 1u] = (uint8_t)(cksum & 0xFFu);

            session_advance(ctx, IRIDIUM_SESSION_SBDWB_PAYLOAD);
            uart_send(ctx, sbdwb_frame, sizeof(sbdwb_frame));
        } else if (session_timed_out(ctx, IRIDIUM_RESP_TIMEOUT_MS)) {
            session_fail(ctx);
        }
        break;

    /* ── Step 3: Wait for SBDWB write result ───────────────── */
    case IRIDIUM_SESSION_SBDWB_PAYLOAD:
        /*
         * The modem responds with a single digit on its own line:
         *   0 = success, 1 = timeout, 2 = bad checksum, 3 = MO buffer full
         * We only accept "0".
         */
        if (ring_contains(ctx, "0\r\n")) {
            //session_advance(ctx, IRIDIUM_SESSION_CSQ);
            //at_send(ctx, "AT+CSQ\r");
            session_advance(ctx, IRIDIUM_SESSION_SBDIX);
            at_send(ctx, "AT+SBDIX\r");
        } else if (ring_contains(ctx, "1\r\n") ||
                   ring_contains(ctx, "2\r\n") ||
                   ring_contains(ctx, "3\r\n")) {
            session_fail(ctx);
        } else if (session_timed_out(ctx, IRIDIUM_RESP_TIMEOUT_MS)) {
            session_fail(ctx);
        }
        break;

    /* ── Step 4: Wait for CSQ result ───────────────── */
    case IRIDIUM_SESSION_CSQ:
        /*
         * The modem replies:
         *   +CSQ:<rssi>
         *   OK
         *
         * rssi status values:
         *   0 = Equivalent to 0 bars displayed on the ISU signal strength indicator.
         *   1 = Equivalent to 1 bar displayed on the ISU signal strength indicator.
         *   2 = Equivalent to 2 bars displayed on the ISU signal strength indicator.
         *   3 = Equivalent to 3 bars displayed on the ISU signal strength indicator.
         *   4 = Equivalent to 4 bars displayed on the ISU signal strength indicator.
         *   5 = Equivalent to 5 bars displayed on the ISU signal strength indicator.
         *
         */
        if (ring_contains(ctx, "1\r\n") ||
            ring_contains(ctx, "2\r\n") ||
            ring_contains(ctx, "3\r\n") ||
            ring_contains(ctx, "4\r\n") ||
            ring_contains(ctx, "5\r\n")) {
            session_advance(ctx, IRIDIUM_SESSION_SBDIX);
            at_send(ctx, "AT+SBDIX\r");
        } else if (ring_contains(ctx, "0\r\n")) {
            session_fail(ctx);
        } else if (session_timed_out(ctx, IRIDIUM_RESP_TIMEOUT_MS)) {
            session_fail(ctx);
        }
        break;

    /* ── Step 4: SBDIX session — wait for +SBDIX response ──── */
    case IRIDIUM_SESSION_SBDIX:
        /*
         * The modem replies:
         *   +SBDIX:<MO status>,<MOMSN>,<MT status>,<MTMSN>,<MT len>,<MT queued>
         *   OK
         *
         * MO status values:
         *   0 = MO message transferred successfully
         *   1 = MO message transferred, but MT message too large to receive
         *   2 = MO message transferred, but requested Location Update was not accepted
         *   10–14 = various temporary failures (retry)
         *   32 = no network access
         *
         * Fire-and-forget: we drain the response to keep the modem happy
         * but do not act on MO status — any outcome is silently dropped.
         */
        if (ring_contains(ctx, "+SBDIX:")) {
            /* Drain remaining response bytes (read until OK or timeout) */
            uint8_t discard[IRIDIUM_RING_LEN];
            ring_read(ctx, discard, ring_available(ctx));
            session_advance(ctx, IRIDIUM_SESSION_COMPLETE);
        } else if (session_timed_out(ctx, IRIDIUM_SBDIX_TIMEOUT_MS)) {
            session_fail(ctx);
        }
        break;

    /* ── Done ───────────────────────────────────────────────── */
    case IRIDIUM_SESSION_COMPLETE:
        ctx->session_state = IRIDIUM_SESSION_IDLE;
        break;

    case IRIDIUM_SESSION_ERROR:
        ctx->session_state = IRIDIUM_SESSION_IDLE;
        break;

    default:
        ctx->session_state = IRIDIUM_SESSION_IDLE;
        break;
    }
}

/* ─────────────────────────────────────────────────────────────
 * Public API — HAL callbacks (called from ISR context)
 * ───────────────────────────────────────────────────────────── */

void Iridium_TxCpltCallback(IridiumCtx_t *ctx)
{
    ctx->tx_done = true;
}

void Iridium_RxEventCallback(IridiumCtx_t *ctx, uint16_t size)
{
    // 'size' in HAL Circular Mode tells us the absolute index offset (0 to IRIDIUM_DMA_RX_LEN)
    // where the DMA pointer currently sits inside dma_rx_buf.
    static uint16_t last_dma_pos = 0;
    uint16_t current_dma_pos = size;
    uint16_t new_bytes = 0;

    // Calculate exactly how many new characters dropped into the hardware buffer
    if (current_dma_pos > last_dma_pos) {
        new_bytes = current_dma_pos - last_dma_pos;
    } else if (current_dma_pos < last_dma_pos) {
        new_bytes = (IRIDIUM_DMA_RX_LEN - last_dma_pos) + current_dma_pos;
    }

    // Move only the newly arrived characters into your software ring buffer
    uint16_t read_idx = last_dma_pos;
    uint16_t head = ctx->ring_head;
    for (uint16_t i = 0; i < new_bytes; i++) {
        ctx->ring_buf[head & (IRIDIUM_RING_LEN - 1u)] = ctx->dma_rx_buf[read_idx];
        head++;
        read_idx = (read_idx + 1) % IRIDIUM_DMA_RX_LEN;
    }
    
    ctx->ring_head = head;
    last_dma_pos = current_dma_pos;

    // DO NOT manually call HAL_UARTEx_ReceiveToIdle_DMA here anymore.
    // Circular mode handles the re-arming automatically in hardware
}
