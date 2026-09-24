## Hector

A little hexapod.

### Parts

- 12 off [SC09 Serial Bus Servo (2.3kg)](https://thepihut.com/products/serial-bus-servo-2-3kg)
- [M5Stack ATOM S3R](https://thepihut.com/products/atoms3r-development-kit-with-0-85-display-8mb-psram)
- [MPM3610 5V Buck Converter Breakout - 21V In 5V Out at 1.2A](https://thepihut.com/products/adafruit-mpm3610-5v-buck-converter-breakout-21v-in-5v-out-at-1-2a)
- 2S LiPo
- Sensor TBD

### Chassis

See `cad` folder above. I used a Bambu P2S.

### Software

`hector.c` drives the servos from a Raspberry Pi 5 through a [Waveshare Serial Bus Servo Driver Board](https://thepihut.com/products/serial-bus-servo-driver-board) (USB, jumper in B). It's a copy of the [servo](https://github.com/op12no2/servo) tester, so all of its commands (`ping`, `stat`, `move`, `torque`, `limits`, ...) work here too, plus the hexapod commands below.

```
make
./hector [/dev/ttyACM0] [baud]     # default 1000000
```

Leg *n* is hip (yaw) servo *n* and lift servo *n*+6. Where each leg is, and which way each servo turns, is set in `legs[]` at the top of the hexapod section of `hector.c`.

| Command | Description |
|---|---|
| `stand [ms]` | all feet down, hips centred, taking ms (default 1000) |
| `legtest [1-6]` | each leg in turn (or one, by hip id): up 30, forward 60, back, down |
| `walk [cycles] [stride] [turn]` | stand, then walk with the current gait; stride < 0 walks backwards, turn > 0 turns left (stride 0 turns on the spot); with no cycles it walks until a key is pressed; a key stops it gracefully (legs step back to centre), ctrl-c freezes it where it is; afterwards it prints each servo's peak load (a servo held over 80% for 4 s drops to 20% torque) |
| `set [name value]` | list or set walk parameters (below) |

| Parameter | Default | |
|---|---|---|
| `gait` | tripod | `wave` (1 leg up at a time), `ripple` (2) or `tripod` (3) |
| `step` | 1000 | ms each leg spends in the air (a cycle is 6 steps in wave, 3 in ripple, 2 in tripod) |
| `stride` | 100 | hip swing either side of centre, servo steps (0.29° each) |
| `lift` | 100 | foot lift during a step, steps |
| `height` | -60 | legs this far below centre when standing, -150..50: > 0 raises the body, < 0 lowers it with the legs splayed (the legs reach the chassis at about 50 below) |

First run: prop the body up so the feet are off the ground, `legtest`, and check that each leg lifts *up* and swings *forward*, and that the leg that moves is the one named. Fix `legs[]` if not. Then put it down and `walk 2`.
