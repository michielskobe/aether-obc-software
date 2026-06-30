import serial
import struct
import time

# -----------------------------
# RXSM protocol constants
# -----------------------------
SYNC1 = 0xAA
SYNC2 = 0x55

CRC_POLY = 0x1021
CRC_INIT = 0x0000


# -----------------------------
# CRC-16 (matches STM32 HAL_CRC config)
# -----------------------------
def crc16_ccitt_false(data: bytes) -> int:
    crc = CRC_INIT

    for b in data:
        crc ^= (b << 8)

        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) & 0xFFFF) ^ CRC_POLY
            else:
                crc = (crc << 1) & 0xFFFF

    return crc & 0xFFFF


# -----------------------------
# Build RXSM frame
# -----------------------------
def build_frame(msg_id: int, payload: bytes = b"") -> bytes:
    if len(payload) > 58:
        raise ValueError("Payload too large for RXSM frame")

    length = len(payload)

    # Header (everything EXCEPT CRC)
    frame_wo_crc = bytearray()
    frame_wo_crc.append(SYNC1)
    frame_wo_crc.append(SYNC2)

    # MSGID big-endian
    frame_wo_crc += struct.pack(">H", msg_id)

    # length
    frame_wo_crc.append(length)

    # payload
    frame_wo_crc += payload

    # CRC computed over entire frame except CRC itself
    crc = crc16_ccitt_false(frame_wo_crc)

    # append CRC big-endian
    frame_wo_crc += struct.pack(">H", crc)

    return bytes(frame_wo_crc)


# -----------------------------
# Send command
# -----------------------------
def send_command(port: str, baud: int, msg_id: int, payload: bytes = b""):
    frame = build_frame(msg_id, payload)

    with serial.Serial(port, baud, timeout=1) as ser:
        ser.write(frame)
        ser.flush()

    print(f"Sent: {frame.hex(' ').upper()}")


# -----------------------------
# Example usage
# -----------------------------
if __name__ == "__main__":
    # Example IDs from your header (replace with real ones if needed)
    EPS_PING            = 0x100  # example placeholder
    EPS_RAIL_ENABLE     = 0x105
    EPS_RAIL_DISABLE    = 0x106
    OBC_TEST_MODE       = 0x800
    OBC_FLIGHT_MODE     = 0x801
    SIMULATE_LO         = 0x802
    SIMULATE_SODS       = 0x803
    SIMULATE_SOE        = 0x804
    SIMULATE_EJECTION   = 0x805

    PORT = "/dev/ttyUSB0"      # or "/dev/ttyUSB0"
    BAUD = 38400
    #for i in range(0,10):
        # 1) Ping (no payload)
        #send_command(PORT, BAUD, EPS_PING)

        # 2) Command with payload
        #send_command(PORT, BAUD, EPS_RAIL_ENABLE, payload=b"\x10")
    #send_command(PORT, BAUD, EPS_RAIL_DISABLE, payload=b"\x10")

    send_command(PORT, BAUD, SIMULATE_EJECTION)