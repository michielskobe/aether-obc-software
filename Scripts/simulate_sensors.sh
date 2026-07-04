#!/bin/bash
DEV=/dev/ttyUSB3
SPEED=500000

send() {
    local ID=$1
    local DATA=$2
    ./canusb -d $DEV -s $SPEED -t -i $ID -j $DATA -n 1 #>/dev/null 2>&1
}

# ----------------------------
# timing anchors (monotonic)
# ----------------------------
t16=$(date +%s.%N)
t10=$t16
t2=$t16
t1=$t16

# ----------------------------
# rate-tracking counters
# ----------------------------
count16=0
count10=0
count2=0
count1=0
stats_anchor=$t16
STATS_INTERVAL=1.0   # print achieved rates every 1s

echo "Starting transmission ..."

while true; do
now=$(date +%s.%N)

# ==================================================
# 16 Hz - IFS_ROTATION (0x520)
# ==================================================
if awk "BEGIN {exit !($now >= $t16)}"; then
    yaw=100
    roll=-50
    pitch=25
    y=$(printf "%04X" $((yaw & 0xFFFF)))
    r=$(printf "%04X" $((roll & 0xFFFF)))
    p=$(printf "%04X" $((pitch & 0xFFFF)))
    send 520 "${y}${r}${p}"
    count16=$((count16+1))
    t16=$(awk "BEGIN {print $t16 + 0.0625}")
fi

# ==================================================
# 10 Hz - IFS_ACCELERATION (0x518)
# ==================================================
if awk "BEGIN {exit !($now >= $t10)}"; then
    ax=100
    ay=-200
    az=300
    temp=2500
    axh=$(printf "%04X" $((ax & 0xFFFF)))
    ayh=$(printf "%04X" $((ay & 0xFFFF)))
    azh=$(printf "%04X" $((az & 0xFFFF)))
    th=$(printf "%04X" $temp)
    send 518 "${azh}${ayh}${axh}${th}"
    count10=$((count10+1))
    t10=$(awk "BEGIN {print $t10 + 0.1}")
fi

# ==================================================
# 2 Hz - IFS_STAGNATION (0x514)
# ==================================================
if awk "BEGIN {exit !($now >= $t2)}"; then
    send 514 "04D208CA"
    count2=$((count2+1))
    t2=$(awk "BEGIN {print $t2 + 0.5}")
fi

# ==================================================
# 1 Hz - all slow telemetry
# ==================================================
if awk "BEGIN {exit !($now >= $t1)}"; then
    # EPS (0.5A, 12V)
    send 504 "01F42EE0"
    # CS_STATUS
    send 505 "1E2D3C4B5A69781F"
    # Altimeter
    send 507 "09C4186A"
    # Thermocouples
    send 509 "006400C8012C0190"
    # Internal thermocouples
    send 511 "000A0014001E0028"
    # Currents
    send 515 "006E00DC"
    send 516 "012C01F4"
    # Manifold
    send 517 "0800"
    count1=$((count1+1))
    t1=$(awk "BEGIN {print $t1 + 1.0}")
fi

# ==================================================
# Periodic achieved-rate report (independent of firmware)
# ==================================================
if awk "BEGIN {exit !($now - $stats_anchor >= $STATS_INTERVAL)}"; then
    elapsed=$(awk "BEGIN {print $now - $stats_anchor}")
    rate16=$(awk "BEGIN {printf \"%.2f\", $count16 / $elapsed}")
    rate10=$(awk "BEGIN {printf \"%.2f\", $count10 / $elapsed}")
    rate2=$(awk "BEGIN {printf \"%.2f\", $count2 / $elapsed}")
    rate1=$(awk "BEGIN {printf \"%.2f\", $count1 / $elapsed}")
    echo "[rates] 0x520: ${rate16} Hz (target 16)  |  0x518: ${rate10} Hz (target 10)  |  0x514: ${rate2} Hz (target 2)  |  1Hz block: ${rate1} Hz (target 1)"
    count16=0
    count10=0
    count2=0
    count1=0
    stats_anchor=$now
fi

sleep 0.0005
done