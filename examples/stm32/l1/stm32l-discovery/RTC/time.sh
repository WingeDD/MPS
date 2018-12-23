#!/bin/bash
device="$1"
time=$(date "+%H:%M:%S")
date=$(date "+%d.%m.%y")

stty 38400 -F "${device}" raw -echo 
echo "${time}" >"${device}"
echo "${date}" >"${device}"
cat "${device}"
