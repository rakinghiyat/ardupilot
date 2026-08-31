#!/bin/sh
#
# Penampil FlightGear untuk model sakti (JSBSim + ArduPlane SITL).
#
# Jalankan SITL lebih dulu di terminal lain, WAJIB dengan --enable-fgview
# (itulah yang membuat ArduPilot mengirim paket FDM ke UDP 5503):
#
#   sim_vehicle.py -v ArduPlane -f jsbsim:sakti --console --enable-fgview \
#     -L Batujajar -w \
#     --add-param-file=Tools/autotest/aircraft/sakti/sakti.parm \
#     --add-param-file=Tools/autotest/aircraft/sakti/custom.parm
#
# Prasyarat: FlightGear/ardupilot_surface.xml sudah disalin ke direktori
# Protocol/ FlightGear. Lihat FlightGear/README.txt di akar repo.
#
# Dua soket masuk:
#   UDP 5503  native-fdm : posisi/sikap + RPM mesin + status mesin (suara)
#   TCP 5510  generic    : defleksi 4 permukaan kendali (animasi)
#

AUTOTESTDIR=$(cd "$(dirname "$0")" && pwd)

fgfs \
    --native-fdm=socket,in,60,,5503,udp \
    --generic=socket,in,60,,5510,tcp,ardupilot_surface \
    --fdm=external \
    --aircraft=sakti \
    --fg-aircraft="$AUTOTESTDIR/aircraft" \
    --airport=WICC \
    --geometry=800x500 \
    --bpp=32 \
    --disable-hud-3d \
    --disable-horizon-effect \
    --timeofday=noon \
    --enable-sound \
    --prop:/sim/sound/enabled=true \
    --prop:/sim/sound/volume=1.0 \
    --prop:/sim/sound/atc/enabled=false \
    --prop:/sim/sound/avionics/enabled=false \
    --prop:/sim/startup/intro-music=false \
    --disable-fullscreen \
    --disable-random-objects \
    --disable-ai-models \
    --fog-disable \
    --disable-specular-highlight \
    --disable-anti-alias-hud \
    --wind=0@0 \
    --verbose \
    --prop:/sim/terrasync/http-server=https://flightgear.sourceforge.net/scenery \
    $*