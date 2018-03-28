#!/bin/bash

#rewrote to use roslaunch instead of bash to control node start and stop
#Point to ROS master on the network
if [ -z "$2" ]
then
    echo "Usage: ./rover_onboard_node_launch.sh master_hostname calibration_location"
    exit 1
else
    export ROS_MASTER_URI=http://$1:11311
fi

#Function to lookup correct path for a given device
findDevicePath() {
    for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
    (
        syspath="${sysdevpath%/dev}"
        devname="$(udevadm info -q name -p $syspath)"
        [[ "$devname" == "bus/"* ]] && continue
        eval "$(udevadm info -q property --export -p $syspath)"
        if [[ "$syspath" == *"tty"* ]] && [[ "$ID_SERIAL" == *"$1"* ]]
        then
            echo $devname
            break
        fi
    )
    done
}

microcontrollerDevicePath=$(findDevicePath Arduino)
if [ -z "$microcontrollerDevicePath" ]
then
    echo "Error: Microcontroller device not found"
fi

gpsDevicePath=$(findDevicePath u-blox)
if [ -z "$gpsDevicePath" ]
then
    echo "Error: u-blox GPS device not found"
fi

roslaunch sipi_controller swarmie_real.launch name:=$HOSTNAME arduino_dev:=/dev/$microcontrollerDevicePath gps_dev:=/dev/$gpsDevicePath use_gps:="false"
