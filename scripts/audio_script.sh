#!/bin/sh -e
sshpass -p 'finder' ssh finder@finder-think "arecord -D plughw:1,0 -f dat" | aplay -f dat
