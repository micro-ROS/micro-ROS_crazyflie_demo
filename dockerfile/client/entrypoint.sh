#!/bin/bash
set -e

cfclient &
while [ ! -f "/.env/variables.env" ]; do
    sleep 2
done
exec socat TCP-LISTEN:1189,reuseaddr,fork FILE:/.env/variables.env