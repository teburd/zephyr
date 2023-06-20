#!/bin/sh
sigrok-cli -d dreamsourcelab-dslogic --config samplerate=1M -O csv -C 0 --time 10s -t 0=r
