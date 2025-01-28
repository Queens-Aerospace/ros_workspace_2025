#!/bin/bash
gnome-terminal --tab --title="Micro DDS" -- bash -c \
'MicroXRCEAgent udp4 -p 8888; exec bash'
