#!/bin/sh
echo "going to project file"
cd /home/pi/projects/netbens/Gold_Rush;
make clean;
make;
cp dist/Debug/GNU-Linux/gold_rush ~/GoldRush;
cd ~/GoldRush/;
echo "installing";
sudo ./installer.sh;

