#!/bin/sh
modprobe ftdi_sio
echo "1cf1 001d" > /sys/module/ftdi_sio/drivers/usb-serial:ftdi_sio/new_id
