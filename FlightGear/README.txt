================================================================================
 ardupilot_surface.xml - protokol generic FlightGear untuk model SITL "sakti"
================================================================================

APA INI
-------
Berkas definisi protokol "generic" FlightGear yang menerjemahkan aliran teks
dari ArduPilot SITL menjadi properti FlightGear, supaya keempat permukaan
kendali (elevon) pesawat ikut bergerak di layar.

Berkas ini TIDAK dibaca dari dalam repo ini. FlightGear hanya mencarinya di
direktori Protocol/ miliknya sendiri, jadi harus disalin manual sekali saja.


PASANG DI MANA
--------------
Salin ke direktori Protocol/ pada FG_ROOT (direktori data FlightGear):

  Linux
    /usr/share/games/flightgear/Protocol/
    atau  $FG_ROOT/Protocol/

  macOS
    ~/Library/Application Support/FlightGear/fgdata_<versi>/Protocol/
    contoh: ~/Library/Application Support/FlightGear/fgdata_2024_1/Protocol/
    (atau di dalam bundel: /Applications/FlightGear.app/Contents/Resources/data/Protocol/)

  Windows
    C:\Program Files\FlightGear <versi>\data\Protocol\

Contoh perintah (macOS):

  cp FlightGear/ardupilot_surface.xml \
     ~/Library/Application\ Support/FlightGear/fgdata_2024_1/Protocol/

Nama berkas HARUS tetap "ardupilot_surface.xml", karena itulah nama yang
dirujuk di baris perintah fgfs:

  --generic=socket,in,60,,5510,tcp,ardupilot_surface

Lihat Tools/autotest/fg_sakti_view.sh.

PERINGATAN: kalau di mesin Anda ada lebih dari satu direktori Protocol/
(misalnya $FG_HOME/Protocol/ dan $FG_ROOT/Protocol/), pastikan TIDAK ada
salinan lama ardupilot_surface.xml yang tertinggal di salah satunya. Salinan
lama akan menutupi yang benar dan menimbulkan gejala di bagian PEMERIKSAAN.


KENAPA URUTANNYA TIDAK BOLEH DIUBAH
-----------------------------------
Pengirimnya adalah SITL::JSBSim::send_servos() di
libraries/SITL/SIM_JSBSim.cpp. Setiap frame ia mengirim SATU baris teks ke
TCP 5510, lima bilangan dipisah spasi, diakhiri newline:

  <LO> <RO> <LI> <RI> <throttle>\n

  LO, RO   = defleksi elevon outboard kiri & kanan, DERAJAT (rentang +-18)
  LI, RI   = defleksi elevon inboard  kiri & kanan, DERAJAT (rentang +-12)
  throttle = posisi throttle ternormalisasi, 0..1

Berkas ini memetakan kelimanya, berurutan, ke properti:

  fcs/left-outboard-pos-deg
  fcs/right-outboard-pos-deg
  fcs/left-inboard-pos-deg
  fcs/right-inboard-pos-deg
  fcs/throttle-pos-norm

Properti itulah yang dianimasikan oleh
Tools/autotest/aircraft/sakti/Models/sakti.xml.

Jadi tiga hal ini harus selalu cocok bertiga:
  1. urutan lima nilai di send_servos()
  2. urutan <chunk> di berkas ini
  3. nama properti yang dipakai animasi di Models/sakti.xml

<var_separator>space</var_separator> wajib ada, karena pengirim memisahkan
nilai dengan spasi.


KALAU SALAH, GEJALANYA
----------------------
- Urutan chunk tertukar  -> permukaan inboard dan outboard bertukar tempat;
                            pesawat terlihat bergerak "terbalik" saat manuver.
- Node ditulis ke tempat lain (misalnya /controls/flight/custom/*)
                         -> animasi tidak bergerak sama sekali, karena
                            Models/sakti.xml membaca fcs/*-pos-deg.
- Berkas tidak dipasang  -> FlightGear gagal memuat protokol; permukaan diam.
- <var_separator> hilang -> hanya nilai pertama yang terbaca.


PEMERIKSAAN CEPAT
-----------------
Jalankan SITL dan fg_sakti_view.sh, lalu di FlightGear buka
Debug > Browse Internal Properties > fcs. Nilai *-pos-deg harus berubah saat
pesawat manuver. Beri perintah roll ke kanan: outboard kiri dan kanan harus
bergerak berlawanan arah, dan kedua inboard bergerak searah.

Catatan: TCP 5510 di-hardcode di SIM_JSBSim.cpp (tidak ikut pergeseran
instance N*10 seperti port lain), jadi hanya SATU instance SITL yang bisa
menyuapi FlightGear pada satu waktu.
