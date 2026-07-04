#!/usr/bin/env python3
"""
SD Card Mission Data Parser
Reads raw SD card data written by the embedded SD Card Manager task.
- Sectors 0-2: metadata (metadata_t struct)
- Sector 3+:   data packets (data_packet_t structs, 8 bytes each)
"""

import csv
import os
import signal
import struct

# ── Configuration ─────────────────────────────────────────────────────────────

SD_DEVICE = "/dev/sda"      # your SD card device
BLOCK_SIZE = 512            # SD block size
ENTRY_SIZE = 8              # size of data_packet_t
DATA_START = 4              # first 3 sectors reserved for metadata
OUTPUT_CSV = "../Data/mission_data.csv"
METADATA_CSV = "../Data/mission_metadata.csv"
TEMP_CSV = "../Data/mission_data_temp.csv"  # temporary CSV during reading
PROGRESS_FILE = "../Data/last_block.txt"

# metadata_t field layout: (byte_offset, byte_size, format)
# Matches the embedded struct (big-endian, STM32)
# typedef struct {
#     uint32_t sequence;
#     uint32_t last_written_sector;
#     uint8_t  rxsm_lo;
#     uint8_t  rxsm_sods;
#     uint8_t  rxsm_soe;
#     uint8_t  ffu_ejection;
#     uint8_t  cgg1_fire;
#     uint8_t  cgg2_fired;
#     uint8_t  bw1_fired;
#     uint8_t  bw2_fired;
#     uint8_t  gnss[8];
#     uint16_t crc;
# } metadata_t;   

METADATA_STRUCT = "<II8B8sH"
METADATA_FIELDS = [
    "sequence",
    "last_written_sector",
    "rxsm_lo",
    "rxsm_sods",
    "rxsm_soe",
    "ffu_ejection",
    "cgg1_fire",
    "cgg2_fired",
    "bw1_fired",
    "bw2_fired",
    "gnss",
    "crc",
]

# ── Signal handling ────────────────────────────────────────────────────────────
stop_requested = False
 
def signal_handler(sig, frame):
    global stop_requested
    print("\nInterrupt received — stopping gracefully...")
    stop_requested = True
 
signal.signal(signal.SIGINT, signal_handler)

# ── Progress tracking ─────────────────────────────────────────────────────────
 
def save_progress(block_num):
    with open(PROGRESS_FILE, "w") as f:
        f.write(str(block_num))
 
def load_progress():
    if os.path.exists(PROGRESS_FILE):
        with open(PROGRESS_FILE, "r") as f:
            val = int(f.read().strip())
            return max(val, DATA_START)  # never go below sector 3
    return DATA_START

# ── Metadata ──────────────────────────────────────────────────────────────────
 
def compute_crc16(data: bytes) -> int:
    """CRC-16/CCITT, polynomial 0x1021, init 0x0000 — matches STM32 HAL config."""
    crc = 0x0000  # match hcrc.Init.InitValue = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc
 
def parse_metadata_block(block: bytes) -> dict | None:
    """
    Parse a single 512-byte sector as a metadata_t struct.
    Returns a dict on success, None if the block is blank or CRC fails.
    """
    size = struct.calcsize(METADATA_STRUCT)
    if len(block) < size:
        return None
 
    raw = block[:size]
 
    # Reject blank sectors
    if all(b == 0x00 for b in raw):
        return None
 
    values = struct.unpack(METADATA_STRUCT, raw)
    m = dict(zip(METADATA_FIELDS, values))
 
    # CRC is computed over all fields except the crc field itself
    crc_payload = raw[:-2]  # everything before the last 2 bytes
    expected_crc = compute_crc16(crc_payload)
    m["crc_ok"] = (expected_crc == m["crc"])
 
    if not m["crc_ok"]:
        print(f"  [WARN] Metadata CRC mismatch: computed 0x{expected_crc:04X}, stored 0x{m['crc']:04X}")
 
    # Format gnss as hex string for readability
    m["gnss"] = "0x" + m["gnss"].hex().upper()
 
    return m
 
def read_metadata(device_path: str) -> dict | None:
    """
    Read all 3 metadata sectors, validate CRC, apply majority voting.
    Returns the best metadata dict, or None if all copies are corrupt.
    """
    copies = []
    print("Reading metadata sectors 0–2...")
    with open(device_path, "rb") as f:
        for sector in range(3):
            f.seek(sector * BLOCK_SIZE)
            block = f.read(BLOCK_SIZE)
            m = parse_metadata_block(block)
            if m is None:
                print(f"  Sector {sector}: blank or unreadable")
            else:
                print(f"  Sector {sector}: OK (sequence={m['sequence']}, last_sector={m['last_written_sector']})")
                copies.append(m)
 
    if not copies:
        print("[ERROR] All metadata copies are corrupt or blank.")
        return None
 
    if len(copies) == 1:
        print("[WARN] Only one valid metadata copy found — no majority voting possible.")
        return copies[0]
 
    # Majority voting: find two copies that agree on sequence number
    for i, a in enumerate(copies):
        for b in copies[i + 1:]:
            if a["sequence"] == b["sequence"]:
                print(f"Metadata majority agreed on sequence={a['sequence']}")
                return a
 
    # No majority — fall back to highest sequence number
    best = max(copies, key=lambda c: c["sequence"])
    print(f"[WARN] No metadata majority — using highest sequence={best['sequence']}")
    return best
 
def save_metadata_csv(metadata: dict, filename: str):
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, "w", newline="") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=metadata.keys())
        writer.writeheader()
        writer.writerow(metadata)
    print(f"Metadata saved to {filename}")
 



# ── Data packet decoding ──────────────────────────────────────────────────────
 
def decode_packet(entry_bytes: bytes) -> dict | None:
    """
    Decode a single 8-byte data_packet_t entry.
    Returns None for zero-padded (empty) entries.
    """
    if entry_bytes == b'\x00' * ENTRY_SIZE:
        return None
 
    timestamp = int.from_bytes(entry_bytes[:3], "big")
    packet_id = entry_bytes[3]
    data = entry_bytes[4:8]
 
    if packet_id == 0:  # latitude
        raw_lat = (data[0] << 16) | (data[1] << 8) | data[2]
        return {
            "timestamp":     timestamp,
            "type":          "GNSS_LATITUDE",
            "latitude_deg":  (raw_lat * (180.0 / 16777216.0)) - 90.0,
            "fix_quality":   data[3],
        }
 
    elif packet_id == 1:  # longitude
        raw_lon = (data[0] << 16) | (data[1] << 8) | data[2]
        return {
            "timestamp":     timestamp,
            "type":          "GNSS_LONGITUDE",
            "longitude_deg": (raw_lon * (360.0 / 16777216.0)) - 180.0,
            "satellites":    data[3],
        }
 
    elif packet_id == 2:  # altitude
        raw_alt = (data[0] << 8) | data[1]
        return {
            "timestamp":     timestamp,
            "type":          "GNSS_ALTITUDE",
            "altitude_m":    raw_alt * 1.5,
            "hdop":          data[2] / 10.0,
        }
    elif packet_id == 4:  # EPS_BATTERY
        return {
            "timestamp": timestamp,
            "type": "EPS_BATTERY",
            "current_mA": int.from_bytes(data[0:2], "big"),
            "voltage_mV": int.from_bytes(data[2:4], "big"),
        }

    elif packet_id == 5:  # CS_STATUS Part 1
        return {
            "timestamp": timestamp,
            "type": "CS_STATUS",
            "CPU_Usage": data[0],
            "CPU_Temp": data[1],
            "RAM_Usage": data[2],
            "eMMC_Usage": data[3],
        }
    
    elif packet_id == 6: ## CS_STATUS Part 2
        return {
            "timestamp": timestamp,
            "type": "CS_STATUS",
            "SD_Usage": data[0],
            "Cam1_RTP": data[1],
            "Cam2_RTP": data[2],
            "CAN_Status": (data[3] >> 4) & 0x01,
            "SPI_MUX_Status": (data[3] >> 3) & 0x01,
            "SD_Status": (data[3] >> 2) & 0x01,
            "Cam2_Status": (data[3] >> 1) & 0x01,
            "Cam1_Status": data[3] & 0x01,
        }

    elif packet_id == 7:  # IFS_ALTIMETER Part 1
        return {
            "timestamp": timestamp,
            "type": "IFS_ALTIMETER",
            "pressure_mbar": int.from_bytes(data, "big", signed=True) * 100,
        }
    
    elif packet_id == 8:  # IFS_ALTIMETER Part 2
        return {
            "timestamp": timestamp,
            "type": "IFS_ALTIMETER",
            "temperature_C": int.from_bytes(data, "big", signed=True) * 100,
        }

    elif packet_id == 9:  # IFS_TCOUPLE Part 1
        return {
            "timestamp": timestamp,
            "type": "IFS_TCOUPLE",
            "T1": int.from_bytes(data[0:2], "big", signed=True) * 0.25,
            "T2": int.from_bytes(data[2:4], "big", signed=True) * 0.25,
        }
    
    elif packet_id == 10:  # IFS_TCOUPLE Part 2
        return {
            "timestamp": timestamp,
            "type": "IFS_TCOUPLE",
            "T3": int.from_bytes(data[0:2], "big", signed=True) * 0.25,
            "T4": int.from_bytes(data[2:4], "big", signed=True) * 0.25,
        }

    elif packet_id == 17:  # IFS_TCOUPLE_INTERN Part 1
        return {
            "timestamp": timestamp,
            "type": "IFS_TCOUPLE_INTERN",
            "T1": int.from_bytes(data[0:2], "big", signed=True) * 0.0625,
            "T2": int.from_bytes(data[2:4], "big", signed=True) * 0.0625,
        }
    
    elif packet_id == 18:  # IFS_TCOUPLE_INTERN Part 2
        return {
            "timestamp": timestamp,
            "type": "IFS_TCOUPLE_INTERN",
            "T3": int.from_bytes(data[0:2], "big", signed=True) * 0.0625,
            "T4": int.from_bytes(data[2:4], "big", signed=True) * 0.0625,
        }
    
    elif packet_id == 19:  # IFS_TCOUPLE_ERROR
        b0 = data[0]
        b1 = data[1]

        return {
            "timestamp": timestamp,
            "type": "IFS_TCOUPLE_ERROR",

            "TC1_OC_fault":   b0 & 0x01,
            "TC1_SCG_fault":  (b0 >> 1) & 0x01,
            "TC1_SCV_fault":  (b0 >> 2) & 0x01,
            "TC2_OC_fault":   (b0 >> 3) & 0x01,
            "TC2_SCG_fault":  (b0 >> 4) & 0x01,
            "TC2_SCV_fault":  (b0 >> 5) & 0x01,

            "TC3_OC_fault":   b1 & 0x01,
            "TC3_SCG_fault":  (b1 >> 1) & 0x01,
            "TC3_SCV_fault":  (b1 >> 2) & 0x01,
            "TC4_OC_fault":   (b1 >> 3) & 0x01,
            "TC4_SCG_fault":  (b1 >> 4) & 0x01,
            "TC4_SCV_fault":  (b1 >> 5) & 0x01,
        }
    
    elif packet_id == 20:  # IFS_STAGNATION
        temp = int.from_bytes(data[0:2], "big", signed=True)
        press = int.from_bytes(data[2:4], "big", signed=True)

        return {
            "timestamp": timestamp,
            "type": "IFS_STAGNATION",
            "temperature_raw": temp,
            "pressure_raw": press,
        }

    elif packet_id == 21:  # IFS_BW_CURRENTS
        return {
            "timestamp": timestamp,
            "type": "IFS_BW_CURRENTS",
            "INA1": int.from_bytes(data[0:2], "big"),
            "INA2": int.from_bytes(data[2:4], "big"),
        }

    elif packet_id == 22:  # IFS_CGG_CURRENTS
        return {
            "timestamp": timestamp,
            "type": "IFS_CGG_CURRENTS",
            "INA1": int.from_bytes(data[0:2], "big"),
            "INA2": int.from_bytes(data[2:4], "big"),
        }

    elif packet_id == 23:  # IFS_MANIFOLD
        return {
            "timestamp": timestamp,
            "type": "IFS_MANIFOLD",
            "pressure": int.from_bytes(data[0:2], "big"),
        }

    elif packet_id == 24:  # IFS_ACCELERATION Part 1
        return {
            "timestamp": timestamp,
            "type": "IFS_ACCELERATION",
            "Z": int.from_bytes(data[0:2], "big", signed=True),
            "Y": int.from_bytes(data[2:4], "big", signed=True),
        }
    
    elif packet_id == 25:  # IFS_ACCELERATION Part 2
        return {
            "timestamp": timestamp,
            "type": "IFS_ACCELERATION",
            "X": int.from_bytes(data[0:2], "big", signed=True),
            "acceleration_temp": int.from_bytes(data[2:4], "big", signed=True),
        }

    elif packet_id == 32:  # IFS_ROTATION Part 1
        return {
            "timestamp": timestamp,
            "type": "IFS_ROTATION",
            "yaw": int.from_bytes(data[0:2], "big", signed=True),
            "roll": int.from_bytes(data[2:4], "big", signed=True),
        } 
    
    elif packet_id == 33:  # IFS_ROTATION Part 2
        return {
            "timestamp": timestamp,
            "type": "IFS_ROTATION",
            "pitch": int.from_bytes(data[0:2], "big", signed=True),
        } 
    
    else:  # unknown packet type
        return {
            "timestamp":     timestamp,
            "type":          "unknown",
            "id":            packet_id,
            "raw":           data.hex(),
        }
 
# ── Main data read ────────────────────────────────────────────────────────────
 
def read_sd_raw(device_path: str, start_block: int = DATA_START, end_block: int = None) -> list:
    global stop_requested
 
    all_entries = []
    empty_streak = 0
    block_num = start_block
 
    print(f"Reading data sectors {start_block}–{end_block if end_block else 'end'}...")
 
    with open(device_path, "rb") as f:
        f.seek(start_block * BLOCK_SIZE)
 
        while True:
            if stop_requested:
                print("Stop requested — saving progress...")
                save_progress(block_num)
                break
 
            if end_block is not None and block_num >= end_block:
                print(f"Reached last written sector ({end_block}).")
                break
 
            if empty_streak > 10:
                print("10 consecutive empty sectors — assuming end of data.")
                break
 
            block = f.read(BLOCK_SIZE)
            if not block or len(block) < BLOCK_SIZE:
                print("Physical end of device reached.")
                break
 
            if all(b == 0x00 for b in block):
                empty_streak += 1
                block_num += 1
                continue
 
            empty_streak = 0
 
            num_entries = BLOCK_SIZE // ENTRY_SIZE
            for i in range(num_entries):
                entry_bytes = block[i * ENTRY_SIZE:(i + 1) * ENTRY_SIZE]
                decoded = decode_packet(entry_bytes)
                if decoded is not None:
                    all_entries.append(decoded)
 
            block_num += 1
 
            # Periodic temp save and progress update
            if block_num % 1000 == 0:
                save_progress(block_num)
                save_csv(all_entries, TEMP_CSV)
                print(f"  ...{block_num} blocks read, {len(all_entries)} entries so far")
 
    save_progress(block_num)
    return all_entries
 
# ── CSV output ────────────────────────────────────────────────────────────────
 
CSV_FIELDS = [
    "timestamp",
    "type",

    # GNSS
    "latitude_deg",
    "longitude_deg",
    "altitude_m",
    "fix_quality",
    "satellites",
    "hdop",

    # EPS battery (0x504)
    "current_mA",
    "voltage_mV",

    # CS status (0x505)
    "CPU_Usage",
    "CPU_Temp",
    "RAM_Usage",
    "eMMC_Usage",
    "SD_Usage",
    "Cam1_RTP",
    "Cam2_RTP",
    "CAN_Status",
    "SPI_MUX_Status",
    "SD_Status",
    "Cam2_Status",
    "Cam1_Status",

    # Altimeter (0x507)
    "pressure_mbar",
    "temperature_C",

    # Thermocouples (0x509 / 0x511)
    "T1",
    "T2",
    "T3",
    "T4",

    # Thermocouple error (0x513)
    "TC1_OC_fault",
    "TC1_SCG_fault",
    "TC1_SCV_fault",
    "TC2_OC_fault",
    "TC2_SCG_fault",
    "TC2_SCV_fault",
    "TC3_OC_fault",
    "TC3_SCG_fault",
    "TC3_SCV_fault",
    "TC4_OC_fault",
    "TC4_SCG_fault",
    "TC4_SCV_fault",

    # Stagnation (0x514)
    "temperature_raw",
    "pressure_raw",

    # Currents (0x515 / 0x516)
    "INA1",
    "INA2",

    # Manifold (0x517)
    "pressure",

    # Acceleration (0x518)
    "Z",
    "Y",
    "X",
    "acceleration_temp",

    # Rotation (0x520)
    "yaw",
    "roll",
    "pitch",

    # Generic / fallback
    "id",
    "raw",
]
 
def save_csv(entries: list, filename: str):
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, "w", newline="") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(entries)
 
# ── Entry point ───────────────────────────────────────────────────────────────
 
if __name__ == "__main__":
    # 1. Read and parse metadata
    metadata = read_metadata(SD_DEVICE)
 
    if metadata:
        save_metadata_csv(metadata, METADATA_CSV)
        end_block = metadata["last_written_sector"]
        # Metadata may lag behind actual data writes
        if end_block < DATA_START:
            print(
                f"[WARN] Metadata last_written_sector={end_block} "
                f"looks uninitialized/stale — scanning until empty sectors."
            )
            end_block = None

        print(f"\nMission status:")
        print(f"  Sequence number:     {metadata['sequence']}")
        print(f"  Last written sector: {end_block}")
        print(f"  RXSM LO:             {'YES' if metadata['rxsm_lo'] else 'NO'}")
        print(f"  RXSM SODS:           {'YES' if metadata['rxsm_sods'] else 'NO'}")
        print(f"  RXSM SOE:            {'YES' if metadata['rxsm_soe'] else 'NO'}")
        print(f"  FFU ejection:        {'YES' if metadata['ffu_ejection'] else 'NO'}")
        print(f"  CGG1 fired:          {'YES' if metadata['cgg1_fire'] else 'NO'}")
        print(f"  CGG2 fired:          {'YES' if metadata['cgg2_fired'] else 'NO'}")
        print(f"  BW1 fired:           {'YES' if metadata['bw1_fired'] else 'NO'}")
        print(f"  BW2 fired:           {'YES' if metadata['bw2_fired'] else 'NO'}")
        print(f"  Last GNSS:           {metadata['gnss']}")
    else:
        print("[WARN] No valid metadata found — reading until 10 empty sectors.")
        end_block = None
 
    # 2. Resume from last progress or start fresh
    start_block = load_progress()
    if start_block > DATA_START:
        print(f"\nResuming from sector {start_block}...")
    else:
        # Clean up old temp files on fresh start
        for f in [TEMP_CSV, PROGRESS_FILE]:
            if os.path.exists(f):
                os.remove(f)
 
    # 3. Read data sectors
    print()
    entries = read_sd_raw(SD_DEVICE, start_block=start_block, end_block=None)
 
    # 4. Save final output
    save_csv(entries, OUTPUT_CSV)
    print(f"\nDone. Saved {len(entries)} entries to {OUTPUT_CSV}")
 
    # 5. Clean up temp files on successful completion
    for f in [TEMP_CSV, PROGRESS_FILE]:
        if os.path.exists(f):
            os.remove(f)
