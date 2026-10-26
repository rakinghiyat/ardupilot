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
# Photoscenery (citra satelit area Pacitan) -- Útil paisagem:
#   --telnet=5000     dipakai Útil paisagem (mode "follow aircraft") untuk
#                      membaca posisi pesawat live dan mengunduh tile di
#                      sekitarnya. Lihat ~/Applications/util-paisagem.
#   --fg-scenery      daftar folder scenery, dipisah ":", urut prioritas
#                      (yang pertama dicek lebih dulu):
#                        1. CUSTOM_TERRAIN_DIR -- mesh terrain hasil build
#                           TerraGear sendiri (elevasi SRTM-1 30m + garis
#                           pantai OSM) untuk area kecil sekitar Pacitan,
#                           memperbaiki bentuk tanah/pantai yang salah di
#                           scenery default. Lihat
#                           ~/Applications/terragear_pacitan (di luar repo).
#                        2. PHOTOSCENERY_DIR -- citra satelit dari Útil
#                           paisagem.
#                      TerraSync (download otomatis bawaan fgfs) tetap jadi
#                      fallback untuk area di luar kedua folder ini --
#                      tidak diganggu sama sekali.
#   Aktifkan render-nya sekali lewat menu dalam-sim: View -> Rendering
#   Options -> centang "Satellite Photoscenery" (tersimpan ke preferensi,
#   tidak perlu diulang tiap sesi).
#
# Vegetasi: pohon acak datang dari /sim/rendering/random-vegetation yang
# default-nya TRUE -- --disable-random-objects TIDAK mencakupnya (toggle
# terpisah). vegetation-density=0.15 menyisakan sedikit pohon agar tetap
# wajar; ganti ke --disable-random-vegetation kalau mau benar-benar gundul.
#

AUTOTESTDIR=$(cd "$(dirname "$0")" && pwd)
PHOTOSCENERY_DIR="$HOME/Library/Application Support/FlightGear/Photoscenery"
CUSTOM_TERRAIN_DIR="$HOME/Applications/terragear_pacitan/output"
mkdir -p "$PHOTOSCENERY_DIR"

fgfs \
    --native-fdm=socket,in,60,,5503,udp \
    --generic=socket,in,60,,5510,tcp,ardupilot_surface \
    --fdm=external \
    --aircraft=sakti \
    --fg-aircraft="$AUTOTESTDIR/aircraft" \
    --fg-scenery="$CUSTOM_TERRAIN_DIR:$PHOTOSCENERY_DIR" \
    --telnet=5000 \
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
    --disable-random-buildings \
    --prop:/sim/rendering/vegetation-density=0.15 \
    --disable-ai-models \
    --fog-disable \
    --disable-specular-highlight \
    --disable-anti-alias-hud \
    --wind=0@0 \
    --verbose \
    --prop:/sim/terrasync/http-server=https://flightgear.sourceforge.net/scenery \
    $*