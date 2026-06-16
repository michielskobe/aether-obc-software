/**
 * @file    iridium.h
 * @brief   Iridium 9603N SBD driver — AT layer, session SM, circular history buffer.
 *
 * Intended for STM32L4 + HAL + CMSIS-RTOS2 / FreeRTOS.
 * UART4 is used with DMA-circular RX (IDLE-line) and DMA-normal TX.
 *
 * External dependencies the caller must provide:
 *   - huart4          (UART_HandleTypeDef, configured externally)
 *   - hdma_uart4_rx   (DMA_HandleTypeDef, linked to UART4 RX)
 *   - hdma_uart4_tx   (DMA_HandleTypeDef, linked to UART4 TX)
 *
 * All functions are called from the IridiumTask context only,
 * except the two HAL callbacks which are called from ISR context.
 */

#ifndef IRIDIUM_H
#define IRIDIUM_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "data_packet.h"

/* ─────────────────────────────────────────────────────────────
 * User configuration
 * ───────────────────────────────────────────────────────────── */

/** Number of 8-byte packets in one MO message (42 × 8 = 336 bytes). */
#define IRIDIUM_HIST_LEN        42U

/** Size in bytes of one application data packet. */
#define IRIDIUM_PKT_BYTES       8U

/** Total MO payload size in bytes (must be ≤ 340). */
#define IRIDIUM_PAYLOAD_BYTES   (IRIDIUM_HIST_LEN * IRIDIUM_PKT_BYTES)   /* 336 */

/** DMA receive scratch buffer size — must be ≥ longest single modem response.
 *  512 bytes covers all standard 9603N responses with margin. */
#define IRIDIUM_DMA_RX_LEN      512U

/** Software ring buffer for RX bytes (power-of-two for efficient masking). */
#define IRIDIUM_RING_LEN        512U

/** Maximum ms to wait for a modem response before declaring a step failed. */
#define IRIDIUM_RESP_TIMEOUT_MS 5000U

/** Maximum ms to wait for the SBDIX session to complete (sessions can be long). */
#define IRIDIUM_SBDIX_TIMEOUT_MS 60000U

/* ─────────────────────────────────────────────────────────────
 * Public types
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Circular history buffer — 42 slots, single-producer single-consumer.
 *        Only IridiumTask reads and writes this; no locking required.
 */
typedef struct {
    data_packet_t slots[IRIDIUM_HIST_LEN];
    uint8_t         head;   /**< Index of the next write slot (wraps mod HIST_LEN). */
} IridiumCircBuf_t;

/**
 * @brief Session state machine states.
 */
typedef enum {
    IRIDIUM_SESSION_IDLE = 0,
    IRIDIUM_SESSION_AT_CHECK,        /* Wait AT OK, send AT+SBDWB=336         */
    IRIDIUM_SESSION_SBDWB_CMD,       /* Wait READY, send 336 B + 2 B checksum */
    IRIDIUM_SESSION_SBDWB_PAYLOAD,   /* Wait for 0\r\n (success), send AT+CSQ */
    IRIDIUM_SESSION_CSQ,             /* Check signal quality           */
    IRIDIUM_SESSION_SBDIX,           /* Send AT+SBDIX, wait +SBDIX:... */
    IRIDIUM_SESSION_COMPLETE,        /* Done — success                  */
    IRIDIUM_SESSION_ERROR,           /* Done — failure, will be dropped */
} IridiumSessionState_t;

/**
 * @brief Top-level driver context.  Declare one instance (e.g. in iridium.c as
 *        a file-static, or in main.c and pass by pointer).
 */
typedef struct {
    /* ── Circular history buffer ── */
    IridiumCircBuf_t    hist;

    /* ── Session state machine ── */
    IridiumSessionState_t  session_state;
    uint32_t            session_tick;          /**< osKernelGetTickCount() snapshot for timeouts. */

    /* ── TX DMA busy flag (set in TxCpltCallback) ── */
    volatile bool       tx_done;

    /* ── RX ring buffer (written by ISR, read by task) ── */
    uint8_t             ring_buf[IRIDIUM_RING_LEN];
    volatile uint16_t   ring_head;          /**< Written by ISR only.  */
    uint16_t            ring_tail;          /**< Read by task only.     */

    /* ── DMA scratch buffer for HAL_UARTEx_ReceiveToIdle_DMA ── */
    uint8_t             dma_rx_buf[IRIDIUM_DMA_RX_LEN];

    /* ── Snapshot buffer built by circ_snapshot before transmission ── */
    uint8_t             tx_payload[IRIDIUM_PAYLOAD_BYTES];

} IridiumCtx_t;

/* ─────────────────────────────────────────────────────────────
 * Public API
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the driver context and start DMA circular RX.
 *         Call once before starting the IridiumTask.
 * @param  ctx   Pointer to a zero-initialised IridiumCtx_t.
 * @param  huart Pointer to the HAL UART handle (UART4).
 */
void Iridium_Init(IridiumCtx_t *ctx, UART_HandleTypeDef *huart);

/**
 * @brief  Push one packet into the circular history buffer.
 *         Call from IridiumTask after dequeuing from the CMSIS message queue.
 */
void Iridium_PushPacket(IridiumCtx_t *ctx, const data_packet_t *pkt);

/**
 * @brief  Kick off a new MO transmission session using the current history buffer.
 *         Safe to call if a session is already running — returns immediately.
 * @return true if a session was started, false if skipped.
 */
bool Iridium_SessionStart(IridiumCtx_t *ctx);

/**
 * @brief  Drive the session state machine forward.
 *         Call on every IridiumTask tick (after draining the queue).
 *         Returns immediately when session_state == IRIDIUM_SESSION_IDLE.
 */
void Iridium_SessionTick(IridiumCtx_t *ctx);

/**
 * @brief  HAL TX-complete callback — call from HAL_UART_TxCpltCallback().
 *         Sets ctx->tx_done = true so the task knows DMA is free.
 */
void Iridium_TxCpltCallback(IridiumCtx_t *ctx);

/**
 * @brief  HAL RX-event callback — call from HAL_UARTEx_RxEventCallback().
 *         Copies received bytes into the ring buffer and re-arms DMA RX.
 * @param  size  Number of bytes placed in dma_rx_buf by the DMA transfer.
 */
void Iridium_RxEventCallback(IridiumCtx_t *ctx, uint16_t size);

#endif /* IRIDIUM_H */
