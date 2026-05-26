#!/usr/bin/env python3
import csv
import os
import signal

SD_DEVICE = "/dev/sda"      # your SD card device
BLOCK_SIZE = 512             # SD block size
ENTRY_SIZE = 8               # size of data_packet_t
OUTPUT_CSV = "../Data/mission_data.csv"
TEMP_CSV = "../Data/mission_data_temp.csv"  # temporary CSV during reading

# Track the last processed block
last_block_processed = 0
stop_requested = False

def signal_handler(sig, frame):
    global stop_requested
    print("\nInterrupt received! Stopping gracefully...")
    stop_requested = True

signal.signal(signal.SIGINT, signal_handler)

def decode_packet(entry_bytes):
    timestamp = int.from_bytes(entry_bytes[:3], "big")
    packet_id = entry_bytes[3]
    data = entry_bytes[4:8]

    if packet_id == 0:  # latitude
        raw_lat = (data[0] << 16) | (data[1] << 8) | data[2]

        latitude_deg = (
            raw_lat * (180.0 / 16777216.0)
        ) - 90.0

        return {
            "timestamp": timestamp,
            "type": "latitude",
            "latitude_deg": latitude_deg,
            "fix_quality": data[3],
        }

    elif packet_id == 1:  # longitude
        raw_lon = (data[0] << 16) | (data[1] << 8) | data[2]

        longitude_deg = (
            raw_lon * (360.0 / 16777216.0)
        ) - 180.0

        return {
            "timestamp": timestamp,
            "type": "longitude",
            "longitude_deg": longitude_deg,
            "satellites": data[3],
        }

    elif packet_id == 2:  # altitude
        raw_alt = (data[0] << 8) | data[1]

        altitude_m = raw_alt * 1.5

        return {
            "timestamp": timestamp,
            "type": "altitude",
            "altitude_m": altitude_m,
            "hdop": data[2] / 10.0,
        }

    return {
        "timestamp": timestamp,
        "type": "unknown",
        "id": packet_id,
        "raw": data.hex(),
    }

def read_sd_raw(device_path, start_block=4):
    global stop_requested, last_block_processed

    all_entries = []

    with open(device_path, "rb") as f:
        # Move to start_block
        f.seek(start_block * BLOCK_SIZE)

        block_num = start_block
        while True:
            if stop_requested:
                break

            block = f.read(BLOCK_SIZE)
            print(f"Reading block {block_num}")
            print(block[:32].hex())
            if not block or all(b == 0x00 for b in block):
                break  # end of used data

            num_entries = BLOCK_SIZE // ENTRY_SIZE
            for i in range(num_entries):
                entry_bytes = block[i*ENTRY_SIZE:(i+1)*ENTRY_SIZE]
                if entry_bytes == b'\x00'*ENTRY_SIZE:
                    continue  # empty entry

                decoded = decode_packet(entry_bytes)
                all_entries.append(decoded)

            block_num += 1
            last_block_processed = block_num

            # Save periodically to temp CSV
            if block_num % 1000 == 0:  # every 1000 blocks
                save_csv(all_entries, TEMP_CSV)

    return all_entries

def save_csv(entries, filename):
    with open(filename, "w", newline="") as csvfile:
        fieldnames = [
            "timestamp",
            "type",
            "latitude_deg",
            "longitude_deg",
            "altitude_m",
            "fix_quality",
            "satellites",
            "hdop",
            "id",
            "raw",
        ]

        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for entry in entries:
            writer.writerow(entry)

if __name__ == "__main__":
    # If a temp CSV exists, resume from last block
    start_block = 4
    entries = []
    if os.path.exists(TEMP_CSV):
        print("Found temp CSV. Resuming from last processed block...")
        with open(TEMP_CSV, newline='') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)  # skip header
            for row in reader:
                timestamp, sensor_id, value = int(row[0]), int(row[1]), int(row[2])
                entries.append((timestamp, sensor_id, value))
        start_block = last_block_processed

    new_entries = read_sd_raw(SD_DEVICE, start_block=start_block)
    entries.extend(new_entries)
    save_csv(entries, OUTPUT_CSV)
    print(f"Saved {len(entries)} entries to {OUTPUT_CSV}")