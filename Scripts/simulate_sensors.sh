#!/bin/bash

DEV=/dev/ttyUSB4
SPEED=500000

send() {
    local ID=$1
    local DATA=$2

    sudo ./canusb -d $DEV -s $SPEED -t -i $ID -j $DATA -n 1 \n
}

while true; do
    NOW=$(date +%s.%N)

    ########################################
    # 16 Hz (every 62.5 ms)
    ########################################
    send 520 "000000000000" &
    sleep 0.0625

    ########################################
    # 10 Hz (every 100 ms)
    ########################################
    send 518 "0000000000000000" &

    ########################################
    # 2 Hz (every 500 ms)
    ########################################
    HALFSEC=$(date +%s)
    if (( HALFSEC % 2 == 0 )); then
        send 514 "000003E8" &
    fi

    ########################################
    # 1 Hz messages
    ########################################
    SEC=$(date +%s)

    if (( SEC != LAST_SEC )); then
        LAST_SEC=$SEC

        # 0x502 Altitude (raw=1000 -> 1500 m)
        send 502 "03E8"

        # 0x504 Battery
        # Current = 500 mA, Voltage = 12000 mV
        send 504 "01F42EE0"

        # 0x505 Camera status
        # CPU=20%, Temp=45C, RAM=30%, eMMC=40%, SD=50%
        # Cam1 RTP=100, Cam2 RTP=100, all status bits OK
        send 505 "142D1E283264641F"

        # 0x507 Altimeter
        # Pressure raw = 10000
        send 507 "0000271000000000"

        # 0x509 Thermocouples
        send 509 "03E803E803E803E8"

        # 0x511 Internal thermocouples
        send 511 "00C800C800C800C8"

        # 0x515 Burn-wire currents
        send 515 "00000000"

        # 0x516 CGG currents
        send 516 "00000000"

        # 0x517 Manifold pressure
        # Raw ADC = 2048
        send 517 "0800"
    fi
done