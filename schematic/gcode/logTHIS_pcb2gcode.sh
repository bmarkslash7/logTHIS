#!/bin/bash

bdir="../schem"

offset="0.005"

f="$bdir/logTHIS-Front.gtl"
b="$bdir/logTHIS-Back.gbl"
d="$bdir/logTHIS.drl"

of="logTHIS_front.ngc"
ob="logTHIS_back.ngc"
od="logTHIS_drill.ngc"

feed="3.5"
seek="9.0"
speed="8000"
drillfeed="3.5"

zw="-0.005"
zs="0.1"
zt="1"
zd="-.125"

pcb2gcode --front $f --back $b --drill $d --front-output $of --back-output $ob --drill-output $od --offset $offset --zsafe $zs --zchange $zt --mill-feed $feed --mill-speed $speed --zdrill $zd --drill-feed $drillfeed --drill-speed $speed --zwork $zw

