# SAKTI UAV — SITL

Software-In-The-Loop simulation for the **sakti** UAV — a delta-wing aircraft with a
gasoline pusher engine, launched from a catapult rail.

This repository is derived from [ArduPilot](https://github.com/ArduPilot/ardupilot),
with the addition of a JSBSim flight dynamics model, a Lua-based elevon mixer, a
FlightGear bridge, and fuel-burn simulation. Three processes run together, connected
by sockets:

| Process | Role |
|---|---|
| **JSBSim** | 6-DOF flight dynamics model: aerodynamics, mass, propulsion, fuel burn, ground reactions |
| **ArduPlane SITL** | the same autopilot firmware that flies the real aircraft — TECS, L1 navigation, attitude PIDs, Lua scripting engine |
| **FlightGear** | visuals only (`--fdm=external`); it computes no physics at all |

## The aircraft

| | |
|---|---|
| Configuration | delta wing, four control surfaces, **no rudder** |
| Control surfaces | 2 inboard elevons ±12°, 2 outboard elevons ±18° |
| Engine | 170 cc two-stroke gasoline, pusher propeller Ø 0.508 m |
| Mass | 50.3 kg (empty 25.3 + payload 15 + fuel 10) |
| Launch | catapult rail, 30° nose-up, 42 m/s exit speed |
| Balance | total CG 1.060 m — 9.3 % static margin |

## Features

- **Complete JSBSim FDM** — four-surface aerodynamics (LIFT/DRAG/PITCH/ROLL/YAW plus
  hinge torque), ground reactions modelling the 30° catapult cradle, and the catapult
  force itself as an `external_reactions` entry.
- **Catapult as an XML state machine** — arm → engine run-up → fire → release, latched
  with `<switch>` blocks in `Systems/launcher.xml`.
- **Lua elevon mixer** running at 50 Hz **inside the ArduPilot process** (not over a
  socket), combining aileron and elevator demand into left/right outboard output.
- **Fuel-burn simulation** — tank contents and fuel flow from JSBSim are routed to a
  simulated analog sender on pin 11 and read back as a `BATT2` fuel gauge
  (`FuelLevel_Analog`). Aircraft mass and CG shift as fuel is consumed.
- **Real engine RPM forwarded to FlightGear** instead of being guessed from PWM, so
  propeller animation and engine sound follow the actual JSBSim engine.
- **Four-surface control animation in FlightGear** over a generic protocol on TCP 5510.
- **Altitude-compensated mixture**, so the engine does not lose power at altitude the
  way JSBSim's uncompensated-carburettor model would otherwise cause.
- **Correct field air density** — the home elevation is written into JSBSim's initial
  conditions instead of defaulting to sea level.

## Installation

Tested on macOS (Apple Silicon) and Ubuntu.

### 1. Clone with submodules

```bash
git clone https://github.com/rakinghiyat/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```

Submodules are required — the repository stores only gitlinks, not their contents.

### 2. ArduPilot prerequisites

```bash
# macOS
Tools/environment_install/install-prereqs-mac.sh -y

# Ubuntu
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Reload your environment afterwards:

```bash
. ~/.profile        # or just open a new terminal
```

### 3. JSBSim — REQUIRED

**JSBSim is not part of this repository, and step 2 does not install it.** It is a
separate program that ArduPilot launches from your `PATH`. The repository does ship
the script that builds it:

```bash
Tools/scripts/build-jsbsim.sh
```

That script clones [JSBSim-Team/jsbsim](https://github.com/JSBSim-Team/jsbsim) into
`~/jsbsim`, builds it with CMake, and stops there. Add the result to your `PATH`
yourself:

```bash
export PATH=$PATH:~/jsbsim/build/src
```

Put that line in `~/.zshrc` or `~/.bashrc` to make it permanent, then confirm:

```bash
JSBSim --version
```

Without this step SITL fails with **"link 1 down"**, because the FDM bridge cannot
find the JSBSim binary.

### 4. Build ArduPlane SITL

```bash
./waf configure --board sitl
./waf plane
```

> Lua scripts are embedded into the firmware **at build time**. After editing a `.lua`
> file, re-run `./waf plane`. After adding a **new** `.lua` file, run
> `./waf configure --board sitl` first — the ROMFS file list is generated during
> configure.

### 5. FlightGear (optional, visuals only)

Install FlightGear from your system's package manager
(`brew install --cask flightgear`, or your distribution's package).

> ### ⚠️ Copy `ardupilot_surface.xml` — the step everyone misses
>
> FlightGear reads protocol definitions only from its own `Protocol/` directory,
> **not from this repository**. The file has to be copied over by hand:
>
> ```bash
> # macOS
> cp FlightGear/ardupilot_surface.xml \
>    ~/Library/Application\ Support/FlightGear/fgdata_*/Protocol/
>
> # Linux
> cp FlightGear/ardupilot_surface.xml $FG_ROOT/Protocol/
> ```
>
> Keep the filename exactly `ardupilot_surface.xml`. Without this the control surfaces
> in FlightGear will not move at all. If an older copy of the file is left behind in
> another `Protocol/` directory, inboard and outboard surfaces can end up swapped.
>
> Full details — including why the column order must match the sender exactly — are in
> [`FlightGear/README.txt`](FlightGear/README.txt).

## Running

```bash
sim_vehicle.py -v ArduPlane -f jsbsim:sakti --console --enable-fgview -L Batujajar -w \
  --add-param-file=Tools/autotest/aircraft/sakti/sakti.parm \
  --add-param-file=Tools/autotest/aircraft/sakti/custom.parm
```

Then, in another terminal, for visuals:

```bash
Tools/autotest/fg_sakti_view.sh
```

| Flag | Why it matters |
|---|---|
| `-f jsbsim:sakti` | selects the JSBSim backend and the model name |
| `--enable-fgview` | the **only** thing that opens UDP 5503. Without it FlightGear receives no position, attitude, RPM or engine sound |
| `-L Batujajar` | field coordinates and elevation; this elevation is what JSBSim uses for air density |
| `-w` | wipes `eeprom.bin`. Without it, stale parameters persist and **override** `--add-param-file` |

## Ports

For SITL instance 0. For instance N, add `N × 10` to each port **except 5510**, which
is hardcoded.

| Port | Proto | Direction | Payload |
|---|---|---|---|
| `5505` | TCP | ArduPilot → JSBSim | servo commands and wind, then `iterate 1` |
| `5504` | UDP | JSBSim → ArduPilot | FGNetFDM: position, attitude, velocity, RPM, fuel |
| `5503` | UDP | ArduPilot → FlightGear | FGNetFDM: position/attitude + RPM + engine state |
| `5510` | TCP | ArduPilot → FlightGear | four control-surface deflections (text) |
| `5760` | TCP | ArduPilot ↔ MAVProxy | MAVLink, bidirectional |
| `14550` | UDP | MAVProxy ↔ GCS | MAVLink, bidirectional |

## Layout

```
Tools/autotest/aircraft/sakti/
    sakti.xml            FDM: aerodynamics, mass, tank, catapult
    sakti-set.xml        FlightGear aircraft definition
    sakti.parm           base autopilot parameters
    custom.parm          tuning overlay
    Systems/             controls, launcher, landing_gear, auto_engine
    Engines/             engine.xml (FGPiston) + propeller.xml
    Models/              FlightGear mesh, animation and sound

ROMFS/scripts/elevon.lua                elevon mixer (embedded at build time)
Tools/autotest/fg_sakti_view.sh         FlightGear viewer
FlightGear/ardupilot_surface.xml        FG generic protocol — copy it manually!

libraries/SITL/SIM_JSBSim.{cpp,h}       FDM bridge
libraries/SITL/SIM_Aircraft.{cpp,h}     fuel state
libraries/SITL/SITL.h
libraries/AP_HAL_SITL/SITL_State.cpp    engine RPM and state to FlightGear
libraries/AP_HAL_SITL/SITL_State_common.{cpp,h}
libraries/AP_HAL_SITL/AnalogIn.cpp      simulated fuel sender (pin 11)
```

## License

ArduPilot, including every change in this repository, is licensed under the
**GNU General Public License v3** — see [`COPYING.txt`](COPYING.txt).

Original work: [ArduPilot Project](https://github.com/ArduPilot/ardupilot) ·
[ardupilot.org](https://ardupilot.org).
SITL with JSBSim documentation:
[ardupilot.org/dev/docs/sitl-with-jsbsim.html](https://ardupilot.org/dev/docs/sitl-with-jsbsim.html).
