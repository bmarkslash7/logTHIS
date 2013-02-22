#!/bin/bash

proj="dht22_sensor_schem"
bdir="../../$proj"

offset="0.005"


f="$bdir/$proj-Front.gtl"
b="$bdir/$proj-Back.gbl"
d="$bdir/$proj.drl"

of="${proj}_front.ngc"
ob="${proj}_back.ngc"
od="${proj}_drill.ngc"

feed="3.5"
seek="9.0"
speed="8000"
drillfeed="3.5"

zw="-0.005"
zs="0.1"
zt="1"
zd="-.125"

pcb2gcode --front $f --back $b --drill $d --front-output $of --back-output $ob --drill-output $od --offset $offset --zsafe $zs --zchange $zt --mill-feed $feed --mill-speed $speed --zdrill $zd --drill-feed $drillfeed --drill-speed $speed --zwork $zw

# filter out f, s, m and t  directives
sed 's/([^)]*)//g' $ob | sed 's/;.*//g' | sed -r 's/[fFmMsStT]-?[0-9]*(\.[0-9]*)?//g' > $ob.clean

ngc_panelize -f $ob.clean -o $ob.panelize -r 3 -c 2 -a 1 -b 1 -D

x_shift=`egrep -o 'x_shift multiplier: .*' ${ob}.panelize | cut -f3 -d' ' `;
y_shift=`egrep -o 'y_shift multiplier: .*' ${ob}.panelize | cut -f3 -d' ' `;

echo x_shift $x_shift, y_shift $y_shift


# filter out f, m, s and t directives
# and expand g81 directives to explicite gcode
sed 's/([^)]*)//g' $od | sed 's/;.*//g' | sed -r 's/[fFmMsStT]-?[0-9]*(\.[0-9]*)?//g' | ngc_g81_expand > $od.clean

ngc_panelize -f $od.clean -o $od.panelize -r 3 -c 2 -X $x_shift -Y $y_shift -D

echo f100 > $proj.ngc.visual_inspection
cat $ob.panelize $od.panelize >> $proj.ngc.visual_inspection





