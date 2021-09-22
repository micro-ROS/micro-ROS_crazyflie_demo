#!/bin/bash
set -e

python3 uros_cf_bridge.py &
while [ ! -f "/.env/variables.env" ]; do
    sleep 2
done

socat TCP-LISTEN:1189,reuseaddr,fork FILE:/.env/variables.env