#!/bin/bash
CAR_ID=`cat src/launch/param/vehicle_id.txt`

# check if script called with sudo
if [[ $(id -u) -ne 0 ]] ; then echo "Please run with sudo" ; exit 1 ; fi


# check if script called with folder arg
if [ -z "$1" ]
then
    echo "usage: " $0 " log_folder"
    exit
fi

# Check if a directory does not exist
if [ -d $1 ] ; then
    echo "record directory already exist: " $1
    exit
fi

echo "record on folder: " $1
echo "CAR ID: $CAR_ID  <- IS IT CORRECT??? if not set it in the script"
mkdir $1
tcpdump src 10.42.$CAR_ID.20 or 10.42.$CAR_ID.21 or 10.42.$CAR_ID.22 -ni bond1 -w $1/luminar.pcap &
tcpdump src 10.42.$CAR_ID.60 or 10.42.$CAR_ID.61 -ni bond1 -w $1/gps.pcap &
candump -L can0 > $1/can0.log &
candump -L can1 > $1/can1.log &
candump -L can2 > $1/can2.log &
candump -L can3 > $1/can3.log &
candump -L can4 > $1/can4.log &
candump -L can5 > $1/can5.log &

while true; do
    clear
    echo "CAR ID: $CAR_ID  <- IS IT CORRECT??? if not set it in the script"
    du -hs $1/*
    echo ""
    df  -k $1 -h
    sleep 1
done

kill $(jobs -p)