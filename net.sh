#!/bin/bash

sudo ifconfig wlan0 up
sudo nmcli radio wifi
sudo nmcli dev status
sudo nmcli --ask dev wifi connect "Rice Visitor"
