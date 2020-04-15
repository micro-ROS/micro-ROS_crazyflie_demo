#!/bin/bash
set -e

python3 micro-ROS-brigde.py &
while [ ! -f "/.env/variables.env" ]; do
    sleep 2
done
exec socat TCP-LISTEN:1189,reuseaddr,fork FILE:/.env/variables.env