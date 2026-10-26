/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulator connector for JSBSim
*/

#include "SIM_config.h"

#if AP_SIM_JSBSIM_ENABLED

#include "SIM_JSBSim.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

// the asprintf() calls are not worth checking for SITL
#pragma GCC diagnostic ignored "-Wunused-result"

#define DEBUG_JSBSIM 1

JSBSim::JSBSim(const char *frame_str) :
    Aircraft(frame_str),
    sock_control(false),
    sock_fgfdm(true),
    sock_fg_surface(false),      // ← tambah
    fg_surface_connected(false), // ← tambah
    initialised(false),
    jsbsim_script(nullptr),
    jsbsim_fgout(nullptr),
    fuel_capacity_lbs(0),
    created_templates(false),
    started_jsbsim(false),
    opened_control_socket(false),
    opened_fdm_socket(false),
    frame(FRAME_NORMAL)
{
    if (strstr(frame_str, "elevon")) {
        frame = FRAME_ELEVON;
    } else if (strstr(frame_str, "vtail")) {
        frame = FRAME_VTAIL;
    } else {
        frame = FRAME_NORMAL;
    }
    const char *model_name = strchr(frame_str, ':');
    if (model_name != nullptr) {
        jsbsim_model = model_name + 1;
    }
}

/*
  create template files
 */
bool JSBSim::create_templates(void)
{
	control_port = 5505 + instance*10;
	fdm_port = 5504 + instance*10;

	printf("JSBSim backend started: instance=%u control_port=%u fdm_port=%u\n",
		   instance, control_port, fdm_port);
		   
    if (created_templates) {
        return true;
    }

    asprintf(&jsbsim_script, "%s/jsbsim_start_%u.xml", autotest_dir, instance);
    asprintf(&jsbsim_fgout,  "%s/jsbsim_fgout_%u.xml", autotest_dir, instance);

    printf("JSBSim_script: '%s'\n", jsbsim_script);
    printf("JSBSim_fgout: '%s'\n", jsbsim_fgout);

    FILE *f = fopen(jsbsim_script, "w");
    if (f == nullptr) {
        AP_HAL::panic("Unable to create jsbsim script %s", jsbsim_script);
    }
    fprintf(f,
"<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
"<?xml-stylesheet type=\"text/xsl\" href=\"http://jsbsim.sf.net/JSBSimScript.xsl\"?>\n"
"<runscript xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n"
"    xsi:noNamespaceSchemaLocation=\"http://jsbsim.sf.net/JSBSimScript.xsd\"\n"
"    name=\"Testing %s\">\n"
"\n"
"  <description>\n"
"    test ArduPlane using %s and JSBSim\n"
"  </description>\n"
"\n"
"  <use aircraft=\"%s\" initialize=\"reset\"/>\n"
"\n"
"  <!-- we control the servos via the jsbsim console\n"
"       interface on TCP 5124 -->\n"
"  <input port=\"%u\"/>\n"
"\n"
"  <run start=\"0\" end=\"10000000\" dt=\"%.6f\">\n"
"    <property value=\"0\"> simulation/notify-time-trigger </property>\n"
// ← tambah: tombol LAUNCH manual (dikirim send_servos dari SERVO7).
// WAJIB dideklarasikan di run-script, bukan cuma di .xml pesawat:
// kalau JSBSim menerima "set" untuk properti tak dikenal, sisa batch
// termasuk "iterate 1" dibatalkan -> tidak ada paket FDM -> link 1 down.
"    <property value=\"0\"> fcs/launch-trigger </property>\n"
"\n"
"    <event name=\"start engine\">\n"
"      <condition> simulation/sim-time-sec le 0.01 </condition>\n"
"      <set name=\"propulsion/engine[0]/set-running\" value=\"1\"/>\n"
"      <notify/>\n"
"    </event>\n"
"\n"
"    <event name=\"Trim\">\n"
"      <condition>simulation/sim-time-sec ge 0.01</condition>\n"
"      <set name=\"simulation/do_simple_trim\" value=\"2\"/>\n"
"      <notify/>\n"
"    </event>\n"
"  </run>\n"
"\n"
"</runscript>\n"
"",
            jsbsim_model,
            jsbsim_model,
            jsbsim_model,
            control_port,
            1.0/rate_hz);
    fclose(f);

    f = fopen(jsbsim_fgout, "w");
    if (f == nullptr) {
        AP_HAL::panic("Unable to create jsbsim fgout script %s", jsbsim_fgout);
    }
    fprintf(f, "<?xml version=\"1.0\"?>\n"
            "<output name=\"127.0.0.1\" type=\"FLIGHTGEAR\" port=\"%u\" protocol=\"UDP\" rate=\"%f\">\n"
            "  <time type=\"simulation\" resolution=\"1e-6\"/>\n"
            "</output>",
            fdm_port, rate_hz);
    fclose(f);

    char *jsbsim_reset;
    asprintf(&jsbsim_reset, "%s/aircraft/%s/reset.xml", autotest_dir, jsbsim_model);

    printf("JSBSim_reset: '%s'\n", jsbsim_reset);

    f = fopen(jsbsim_reset, "w");
    if (f == nullptr) {
        AP_HAL::panic("Unable to create jsbsim reset script %s", jsbsim_reset);
    }
    float r, p, y;
    dcm.to_euler(&r, &p, &y);
    /*
      <elevation> memberitahu JSBSim ketinggian TANAH di lokasi home.
      Tanpa ini JSBSim menganggap tanah ada di permukaan laut, sehingga
      seluruh aerodinamika dihitung pada densitas permukaan laut walau
      lapangannya jauh lebih tinggi -> lift berlebih, stall speed &
      jarak takeoff terlalu optimis, dan airspeed yang dilaporkan
      berselisih dengan EAS yang dicatat ArduPilot (EAS2TAS).
      home.alt dalam cm, JSBSim minta meter.

      Catatan: <altitude> di file IC JSBSim adalah tinggi DI ATAS TANAH
      (AGL), bukan di atas permukaan laut - lihat FGInitialCondition
      Load_v1(): "altitude" -> SetAltitudeAGLFtIC(). Jadi pesawat tetap
      menempel di tanah, tidak jatuh dari ketinggian home.
    */
    fprintf(f,
            "<?xml version=\"1.0\"?>\n"
            "<initialize name=\"Start up location\">\n"
            "  <latitude unit=\"DEG\" type=\"geodetic\"> %f </latitude>\n"
            "  <longitude unit=\"DEG\"> %f </longitude>\n"
            "  <elevation unit=\"M\"> %f </elevation>\n"
            "  <altitude unit=\"M\"> 1.3 </altitude>\n"
            "  <vt unit=\"FT/SEC\"> 0.0 </vt>\n"
            "  <gamma unit=\"DEG\"> 0.0 </gamma>\n"
            "  <phi unit=\"DEG\"> 0.0 </phi>\n"
            "  <theta unit=\"DEG\"> 13.0 </theta>\n"
            "  <psi unit=\"DEG\"> %f </psi>\n"
            "</initialize>\n",
            home.lat*1.0e-7,
            home.lng*1.0e-7,
            home.alt*0.01,
            degrees(y));
    fclose(f);

    created_templates = true;
    return true;
}


/*
  start JSBSim child
 */
bool JSBSim::start_JSBSim(void)
{
    if (started_jsbsim) {
        return true;
    }
    if (!open_fdm_socket()) {
        return false;
    }

    int p[2];
    int devnull = open("/dev/null", O_RDWR|O_CLOEXEC);
    if (pipe(p) != 0) {
        AP_HAL::panic("Unable to create pipe");
    }
    pid_t child_pid = fork();
    if (child_pid == 0) {
        // in child
        setsid();
        dup2(devnull, 0);
        dup2(p[1], 1);
        close(p[0]);
        for (uint8_t i=3; i<100; i++) {
            close(i);
        }
        char *logdirective;
        char *script;
        char *nice;
        char *rate;

        asprintf(&logdirective, "--logdirectivefile=%s", jsbsim_fgout);
        asprintf(&script, "--script=%s", jsbsim_script);
        asprintf(&nice, "--nice=%.8f", 10*1e-9);
        asprintf(&rate, "--simulation-rate=%f", rate_hz);

        if (chdir(autotest_dir) != 0) {
            perror(autotest_dir);
            exit(1);
        }

        int ret = execlp("JSBSim",
                         "JSBSim",
                         "--suspend",
                         rate,
                         nice,
                         logdirective,
                         script,
                         nullptr);
        if (ret != 0) {
            perror("JSBSim");
        }
        exit(1);
    }
    close(p[1]);
    jsbsim_stdout = p[0];

    // read startup to be sure it is running
    char c;
    if (read(jsbsim_stdout, &c, 1) != 1) {
        AP_HAL::panic("Unable to start JSBSim");
    }

    if (!expect("JSBSim Execution beginning")) {
        AP_HAL::panic("Failed to start JSBSim");
    }
    if (!open_control_socket()) {
        AP_HAL::panic("Failed to open JSBSim control socket");
    }

    fcntl(jsbsim_stdout, F_SETFL, fcntl(jsbsim_stdout, F_GETFL, 0) | O_NONBLOCK);

    started_jsbsim = true;
    check_stdout();
    close(devnull);
    return true;
}

/*
  check for stdout from JSBSim
 */
void JSBSim::check_stdout(void) const
{
    char line[100];
    ssize_t ret = ::read(jsbsim_stdout, line, sizeof(line));
    if (ret > 0) {
#if DEBUG_JSBSIM
        write(1, line, ret);
#endif
    }
}

/*
  a simple function to wait for a string on jsbsim_stdout
 */
bool JSBSim::expect(const char *str) const
{
    const char *basestr = str;
    while (*str) {
        char c;
        if (read(jsbsim_stdout, &c, 1) != 1) {
            return false;
        }
        if (c == *str) {
            str++;
        } else {
            str = basestr;
        }
#if DEBUG_JSBSIM
        write(1, &c, 1);
#endif
    }
    return true;
}

/*
  open control socket to JSBSim
 */
bool JSBSim::open_control_socket(void)
{
    if (opened_control_socket) {
        return true;
    }
    if (!sock_control.connect("127.0.0.1", control_port)) {
        return false;
    }
    printf("Opened JSBSim control socket\n");
    sock_control.set_blocking(false);
    opened_control_socket = true;

    char startup[] =
        "info\n"
        "resume\n"
        "iterate 1\n"
        "set atmosphere/turb-type 4\n";
    sock_control.send(startup, strlen(startup));
    return true;
}

/*
  open fdm socket from JSBSim
 */
bool JSBSim::open_fdm_socket(void)
{
    if (opened_fdm_socket) {
        return true;
    }
    if (!sock_fgfdm.bind("127.0.0.1", fdm_port)) {
        check_stdout();
        return false;
    }
    sock_fgfdm.set_blocking(false);
    sock_fgfdm.reuseaddress();
    opened_fdm_socket = true;
    return true;
}

bool JSBSim::open_fg_surface_socket(void) // <- tambah
{
    if (!sock_fg_surface.connect("127.0.0.1", 5510)) {
        return false;
    }
    sock_fg_surface.set_blocking(false);
    fg_surface_connected = true;
    fprintf(stderr, "JSBSim: Connected to FG surface socket port 5510\n");
    return true;
}

/*
  decode and send servos
*/
void JSBSim::send_servos(const struct sitl_input &input)
{
    char *buf = nullptr;
    float servo1   = filtered_servo_angle(input, 0);
    float servo2   = filtered_servo_angle(input, 1);
    float servo3   = filtered_servo_angle(input, 2);
    float throttle = filtered_servo_range(input, 4);
    // ← tambah: tombol LAUNCH manual dari SERVO7 (indeks 6).
    // Dibaca Systems/launcher.xml sebagai fcs/launch-trigger.
    // fcs/launch-trigger WAJIB dideklarasikan di sakti*.xml, kalau tidak
    // JSBSim menolak perintah "set" properti tak dikenal dan SITL membeku.
    float launch_sw = filtered_servo_range(input, 6);

    float wind_speed_fps = input.wind.speed / FEET_TO_METERS;

    asprintf(&buf,
        "set fcs/aileron-cmd-norm %f\n"
        "set fcs/elevator-cmd-norm %f\n"
        "set fcs/rudder-cmd-norm %f\n"
        "set fcs/throttle-cmd-norm %f\n"
        "set fcs/launch-trigger %f\n"
        "set atmosphere/psiw-rad %f\n"
        "set atmosphere/wind-mag-fps %f\n"
        "set atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps %f\n"
        "set atmosphere/turbulence/milspec/severity %f\n"
        "iterate 1\n",
        (double)servo1,
        (double)servo3,
        (double)servo2,
        (double)throttle,
        (double)launch_sw,
        (double)radians(input.wind.direction),
        (double)wind_speed_fps,
        (double)(wind_speed_fps / 3.0f),
        (double)input.wind.turbulence);

    ssize_t buflen = strlen(buf);
    sock_control.send(buf, buflen);
    free(buf);

    // =========================================================
    // KIRIM POS-DEG KE FLIGHTGEAR
    // Konversi dari cmd-norm ke degrees untuk animasi FG
    // LO/RO: range ±18°, LI/RI: range ±12°
    // =========================================================
    if (fg_surface_connected) {
        float LO_deg = servo1 * 18.0f;
        float RO_deg = servo2 * 18.0f;
        float LI_deg = servo3 * 12.0f;
        float RI_deg = servo3 * 12.0f;
        char fg_buf[160];
        int len = snprintf(fg_buf, sizeof(fg_buf),
            "%.4f %.4f %.4f %.4f %.4f\n",
            (double)LO_deg,
            (double)RO_deg,
            (double)LI_deg,
            (double)RI_deg,
            (double)throttle);
        ssize_t fg_sent = sock_fg_surface.send(fg_buf, len);
        if (fg_sent < 0 && errno != EAGAIN) {
            fg_surface_connected = false;
        }
    }

    static uint32_t counter = 0;
    if (++counter % 100 == 0) {
        // rpm[0] diisi dari fdm.rpm[0] di recv_fdm().
        // airspeed dalam m/s (EAS) -> dikonversi ke knot agar sebanding
        // dengan velocities/vc-kts di JSBSim.
        const double vc_kts = airspeed / KNOTS_TO_METERS_PER_SECOND;

        fprintf(stderr,
            "[VERIFY] s1=%.3f s2=%.3f s3=%.3f thr=%.3f | fuel=%.1f%% %.2fL/jam rpm=%5.0f vc=%.0f kts\n",
            servo1, servo2, servo3, throttle,
            (double)(fuel_level_ratio*100), (double)fuel_flow_lph,
            (double)rpm[0], vc_kts);

        // Nilai PWM mentah dari ArduPilot. Tersedia di sini lewat
        // input.servos[] (struct sitl_input) - inilah keluaran servo
        // sebelum di-scale jadi cmd-norm, jadi JSBSim TIDAK perlu tahu
        // apa-apa soal PWM. Indeks 0-based: s1 = servos[0].
        fprintf(stderr,
            "[PWM] s1=%uPWM s2=%uPWM s3=%uPWM s4=%uPWM s5=%uPWM\n",
            input.servos[0], input.servos[1], input.servos[2],
            input.servos[3], input.servos[4]);
    }
}

/* nasty hack ....
   JSBSim sends in little-endian
 */
void FGNetFDM::ByteSwap(void)
{
    uint32_t *buf = (uint32_t *)this;
    for (uint16_t i=0; i<sizeof(*this)/4; i++) {
        buf[i] = ntohl(buf[i]);
    }
    // fixup the 3 doubles
    buf = (uint32_t *)&longitude;
    uint32_t tmp;
    for (uint8_t i=0; i<3; i++) {
        tmp = buf[0];
        buf[0] = buf[1];
        buf[1] = tmp;
        buf += 2;
    }
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void JSBSim::recv_fdm(const struct sitl_input &input)
{
    FGNetFDM fdm;
    check_stdout();

    do {
        while (sock_fgfdm.recv(&fdm, sizeof(fdm), 100) != sizeof(fdm)) {
            send_servos(input);
            check_stdout();
        }
        fdm.ByteSwap();
    } while (fdm.cur_time == time_now_us);

    accel_body = Vector3f(fdm.A_X_pilot, fdm.A_Y_pilot, fdm.A_Z_pilot) * FEET_TO_METERS;

    double p, q, r;
    SIM::convert_body_frame(degrees(fdm.phi), degrees(fdm.theta),
                             degrees(fdm.phidot), degrees(fdm.thetadot), degrees(fdm.psidot),
                             &p, &q, &r);
    gyro = Vector3f(p, q, r);

    velocity_ef = Vector3f(fdm.v_north, fdm.v_east, fdm.v_down) * FEET_TO_METERS;
    location = {
        int32_t(RAD_TO_DEG_DOUBLE * fdm.latitude * 1.0e7),
        int32_t(RAD_TO_DEG_DOUBLE * fdm.longitude * 1.0e7),
        int32_t(fdm.agl*100 + home.alt),
        Location::AltFrame::ABSOLUTE
    };
    dcm.from_euler(fdm.phi, fdm.theta, fdm.psi);
    airspeed = fdm.vcas * KNOTS_TO_METERS_PER_SECOND;
    airspeed_pitot = airspeed;

    // update magnetic field
    update_mag_field_bf();
    
    rpm[0] = fdm.rpm[0];
    rpm[1] = fdm.rpm[1];

    /*
      BAHAN BAKAR.

      FGNetFDM sudah membawa isi tangki (fuel_quantity, satuan POUND -
      FGOutputFG mengirim FGTank::GetContents() apa adanya) dan laju alir
      per-engine (fuel_flow, satuan US galon/jam). Sebelumnya kedua field
      ini dibuang di sini, jadi ArduPilot sama sekali tidak tahu bahwa
      JSBSim sedang membakar bahan bakar - padahal massa & CG pesawat di
      JSBSim sudah ikut berubah.

      Kapasitas tangki tidak ikut dikirim di paket FDM, jadi isi tangki
      pada frame pertama dipakai sebagai "penuh". Itu valid selama file
      pesawat memberi <contents> == <capacity> (setup normal, dan seperti
      itulah sakti). Rasio ini yang dibaca sender fuel-level tiruan di
      AP_HAL_SITL, sehingga tidak ada asumsi densitas bahan bakar di sini.
    */
    if (fdm.num_tanks > 0) {
        const float contents_lbs = fdm.fuel_quantity[0];
        if (!has_fuel_state && is_positive(contents_lbs)) {
            fuel_capacity_lbs = contents_lbs;
            has_fuel_state = true;
        }
        if (has_fuel_state) {
            fuel_level_ratio = constrain_float(contents_lbs / fuel_capacity_lbs, 0, 1);
        }
    }
    if (fdm.num_engines > 0) {
        // US gallons/hour -> litres/hour
        fuel_flow_lph = fdm.fuel_flow[0] * 3.785411784f;
    }

    time_now_us = fdm.cur_time;
}

void JSBSim::drain_control_socket()
{
    const uint16_t buflen = 1024;
    char buf[buflen];
    ssize_t received;
    do {
        received = sock_control.recv(buf, buflen, 0);
    } while (received > 0);
}
/*
  update the JSBSim simulation by one time step
 */
void JSBSim::update(const struct sitl_input &input)
{
    while (!initialised) {
        if (!create_templates() ||
            !start_JSBSim() ||
            !open_control_socket() ||
            !open_fdm_socket()) {
            time_now_us = 1;
            return;
        }
        initialised = true;
    }
    if (!fg_surface_connected) {
        open_fg_surface_socket();  // ← tambah
    }
    send_servos(input);
    recv_fdm(input);
    adjust_frame_time(rate_hz);
    sync_frame_time();
    drain_control_socket();
}

} // namespace SITL

#endif  // AP_SIM_JSBSIM_ENABLED