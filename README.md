## Hector

A little hexapod.

### Parts

- 12 off [SC09 Serial Bus Servo (2.3kg)](https://thepihut.com/products/serial-bus-servo-2-3kg)
- [M5Stack ATOM S3R](https://thepihut.com/products/atoms3r-development-kit-with-0-85-display-8mb-psram)
- [MPM3610 5V Buck Converter Breakout - 21V In 5V Out at 1.2A](https://thepihut.com/products/adafruit-mpm3610-5v-buck-converter-breakout-21v-in-5v-out-at-1-2a)
- 2S LiPo
- Small Schottky diode (SS14, 1N5817), a switch (or XT30 plug) and optionally a ~5 A fuse
- Sensor TBD

### Wiring

See [wiring.md](wiring.md): the ATOM drives the bus through the adapter's UART header (jumper in A), powered from the battery through the buck and a diode, which leaves its USB-C free for flashing.

### Chassis

See `cad` folder above. I used a Bambu P2S.

### Software

`hector.c` drives the servos from a Raspberry Pi 5 through a [Waveshare Serial Bus Servo Driver Board](https://thepihut.com/products/serial-bus-servo-driver-board) (USB, jumper in B). It's a copy of the [servo](https://github.com/op12no2/servo) tester, so all of its commands (`ping`, `stat`, `move`, `torque`, `limits`, ...) work here too, plus the hexapod commands below.

```
make
./hector [/dev/ttyACM0] [baud]     # default 1000000
```

The same `hector.c` also builds for the ATOM S3R (ESP-IDF, in `atom/`), which drives the bus through the adapter's UART header (jumper in A, see [wiring.md](wiring.md)), with the REPL on the ATOM's USB:

```
cd atom
source ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/serial/by-id/usb-Espressif_* flash monitor     # ctrl-] leaves the monitor
```

At power-on the ATOM runs the checks, stands up unless something failed, runs the standing checks, then opens its eyes: they blink and glance about while it waits for a command. The whites go yellow or red if a check warned or failed, with the problem under them. While it waits it also watches the servos (a low or flat battery, heat, error bits, a load held high, a servo not replying: shown under the eyes, and for the serious ones it sits down), and while it's standing, tip it up between two legs and put it down: it bows towards that edge (those two legs' lifts go up to 700, so that edge of the body comes down onto the ground), stands again, and looks that way. Knocked over, it sits. Every command works the same way there; `quit` has nothing to quit to, and changes to `offset[]` last until the ATOM resets. Any terminal on its USB port works as well as `idf.py monitor` (e.g. `picocom /dev/ttyACM0`), but opening the port with `idf.py monitor` resets the ATOM.

Leg *n* is hip (yaw) servo *n* and lift servo *n*+6. Where each leg is, and which way each servo turns, is set in `legs[]` at the top of the hexapod section of `hector.c`.

`offset[]`, just above it, is a per-servo offset added to every goal position sent, by `move` as well as the hexapod commands, so with an offset of 20 on servo 7, `move 7 511` sends 531. The lifts' offsets come from `calibrate`. `offset` and `calibrate` change it until you exit; they print the table as a line of C to paste into `hector.c` to keep it. `pos`, `stat` and `move`'s check report the servo's actual position, with the offset included; `ww` to the goal register doesn't add it.

| Command | Description |
|---|---|
| `stand [ms]` | all feet down, hips centred, taking ms (default 1000) |
| `sit [ms]` | all six lifts up to 1000, taking ms (default 1000), the same as `move 7-12 1000 1000`: the body sits on the ground with the legs pointing up inside it, so there's no load on the servos. Do it before switching off; `stand` gets up again |
| `legtest [1-6]` | each leg in turn (or one, by hip id): lift servo up to 700, forward 80, back, down to its stance pose; the other legs stay where they are |
| `ident [id]` | twitch each id (default 1-12) in turn, a second apart, to see which servo has which id: 30 steps up from where it is and back; it prints which leg `legs[]` says the id belongs to |
| `offset [id val]` | show the per-servo offsets, or set one; the change lasts until exit |
| `calibrate` | stand, then nudge the lift offsets, a step or two at a time from where they are, until all six feet carry the same load (within 1.5%); prints the offset table to paste into `hector.c` |
| `walk [cycles] [stride] [turn]` | stand, then walk with the current gait; stride < 0 walks backwards, turn > 0 turns left (stride 0 turns on the spot); with no cycles it walks until a key is pressed; a key stops it gracefully (legs step back to centre), ctrl-c freezes it where it is; afterwards it prints each servo's peak load (a servo held over 80% for 4 s drops to 20% torque) |
| `set [name value]` | list or set walk parameters (below) |
| `loads [secs]` | stream every servo's load and position error (goal and position both read from the servo) as fast as the bus allows, about 80 sweeps of all 12 a second, for secs seconds (default 10) or until a key; reads only, so stand first. The ATOM's screen shows the seconds, to time presses by |
| `imu [secs]` | stream the ATOM's IMU (BMI270) at 50 lines a second: acceleration (g), rotation (deg/s), x forward, y left, z up, and the tilt from the first reading; default 10 s, or until a key |
| `selftest` | the checks it runs at startup, again (they only read, nothing moves). Before standing (the go/no-go for standing up): the screen and the IMU answer (ATOM only), every servo replies, battery (warns under 7.0 V, fails under 6.6), temperature, error bits, overload protection at the defaults, the gains (P 15, D 15, I 1 on the lifts and 0 on the hips), positions inside the range the legs are driven over. Standing (only if `stand` ran last, not `sit`): the load on each foot (warns if one is over 5% off the mean: calibrate), sag (a foot pushed up over 6 steps from its goal), level (IMU, warns over 5°), the highest load on any servo, the battery under load. On the ATOM each check also goes up on the screen as it runs, then the eyes come back, their whites yellow with warnings or red with failures, with the first problem under them |

| Parameter | Default | |
|---|---|---|
| `gait` | wave | `wave` (1 leg up at a time), `ripple` (2) or `tripod` (3) |
| `step` | 800 | ms each leg spends in the air (a cycle is 6 steps in wave, 3 in ripple, 2 in tripod) |
| `stride` | 50 | hip swing either side of centre, servo steps (0.29° each) |
| `lift` | 70 | foot lift during a step, steps |
| `height` | 0 | legs this far below centre when standing, -150..50: > 0 raises the body, < 0 lowers it with the legs splayed (the legs reach the chassis at about 50 below) |

First run: prop the body up so the feet are off the ground, `legtest`, and check that each leg lifts *up* and swings *forward*, and that the leg that moves is the one named. Fix `legs[]` if not. Then put it down and `walk 2`.
