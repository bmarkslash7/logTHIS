#!/bin/bash

chip=t85

if [ "$1" = '' ]; then echo provide hex file; exit; fi
echo running "avrdude -c usbtiny -p $chip -U flash:w:$1"
avrdude -c usbtiny -p $chip -U flash:w:$1

