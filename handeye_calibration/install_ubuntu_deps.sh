#!/bin/bash

ROSVERSION=`rosversion -d`
PACKAGES="ros-$ROSVERSION-visp-hand2eye-calibration"
echo "Installing $PACKAGES"
read -p "Are you sure? " -n 1 -r
echo    # (optional) move to a new line
if [[ $REPLY =~ ^[Yy]$ ]]
then
    sudo apt-get install $PACKAGES
fi



