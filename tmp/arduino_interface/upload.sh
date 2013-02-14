#!/bin/bash
#
# upload to arduino
# assumes atmega328p
# assumes USB port is /dev/ttyACM0

chip="m328p"
port="/dev/ttyACM0"
hex=$1

if [ "$hex" == "" ] || [ ! -e "$hex" ]
then
  echo provide hex file
  exit
fi

avrdude -c arduino -p $chip -P $port -U flash:w:$hex
