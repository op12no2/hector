# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Hector, a small hexapod: 6 legs, 2 DOF each, 12 Waveshare SC09 bus servos. `hector.c` is a single-file C REPL that started as a copy of `../servo/servo.c` (the generic SC09 tester, https://github.com/op12no2/servo) and keeps all its commands; the hexapod commands (`stand`, `legtest`, `walk`, `set`) are in the `/* ---- hexapod ---- */` section. Fixes to the servo layer (protocol, `move`, the line editor) likely belong in `../servo` too. Its `CLAUDE.md` has the details of the SC09 and the adapter: register behaviour, move overshoot, the unreliable moving flag, the dead zone and so on. There are no tests and no dependencies beyond libm.

## Hardware

- **Host:** Raspberry Pi 5 (where Claude Code runs) via the Waveshare Bus Servo Adapter (A) over USB, `/dev/ttyACM0` (stable path `/dev/serial/by-id/usb-1a86_USB_Single_Serial_*`), jumper in B, 1 Mbps. The README lists an M5Stack ATOM S3R as the eventual on-board controller; the gait code has no I/O of its own besides `send_pose()`, to ease a port.
- **Servos:** SC09, 300° over 0..1023 (0.293°/step), 2.3 kg·cm. Supply is a 2S LiPo straight to the servos (rated 4.8–8.4 V).
- **Geometry:** `cad/README.md` has the parts, how they fit together, and the foot position against `height`. The parts share one coordinate frame. To measure the STEP files, `pip install cadquery-ocp` (Open CASCADE; aarch64 wheels exist) into a venv. Its OCP 8 API needs `Bnd_Box.CornerMin/CornerMax`, not `Get()`, and `TopoDS.Face`, not `Face_s`. All-up weight is about 500 g.
- **Legs:** hip (yaw) servos are ids 1–6, lift servos 7–12; leg n = hip n + lift n+6. Nominal (stand) position is 511 for all. Lifts can go up to the servo limit but only about 50 steps below 511 before the leg hits the chassis (the user judges 50 itself OK); `DOWN_MAX` (50) clamps that in `send_pose()`. `height` runs from -`SPLAY_MAX` (-150: stance legs 150 above centre, body lower, legs splayed) to `DOWN_MAX`. Hips can easily do 300 either way; `HIP_MAX` (150) clamps them, as neighbouring legs can meet.
- **Overload protection** (Waveshare's SCS memory table, https://files.waveshare.com/upload/5/5c/SCS_Series_Memory_Table_Analysis.xls; the SC09s here hold its defaults): if a servo's load (0x3C, the drive duty cycle in 0.1%) stays over the overload torque (0x27, 80%) for the protection time (0x26, 100 × 40 ms = 4 s), its output drops to the protection torque (0x25, 20%) and its status reports overload (bit 5). Unloading conditions (0x13) = 0x20, i.e. only overload protection enabled. A lift servo is loaded all the time its foot is down, so standing still and slow gaits (a wave stance at an 800 ms step is 4.0 s) are the risk, more than fast ones.
- **Servo 9** (the lift of leg 3, LR) is broken: it goes into overload protection and sags (found at 549 against a goal of 511, at 20% load). A replacement was ordered on 2026-09-24 (it will need `setid 1 9`). Until then, ignore its overload reports and the limp on that corner; don't add workarounds for it.
- **Layout and directions:** `legs[]` (layout, `hip_dir`, `lift_dir`) as first written walked the robot correctly without changes, so the guesses were right. If a leg is rebuilt, check it with `legtest` (lift of only 30, within the 50-step chassis margin) before walking: a wrong `lift_dir` would drive the swing legs down into the chassis.

## Build / run

```
make                               # cc -O2 -Wall -Wextra -o hector hector.c -lm
./hector [/dev/ttyACM0] [baud]     # defaults: /dev/ttyACM0, 1000000
echo "walk 2" | ./hector           # non-tty stdin works for scripted commands
```

Keep the build warning-free under `-Wall -Wextra`. Walking can only really be verified on the robot; if you can't test on it, say so. Never make it move unless the user is there: it can walk off the desk.

## Git

The repo is https://github.com/op12no2/hector (private). Commit and push to `origin main` whenever a change is done and tested; no need to ask first. If it could only be tested off the robot (e.g. against a fake bus), say so in the commit message. Keep README, `help()` and this file in step with the code in the same commit. The user sometimes adds files through the GitHub web UI, so pull before starting work.

## Architecture (hexapod section)

- **`legs[]`:** per leg, side (0 left, 1 right) and row (0 front, 1 middle, 2 rear), hip and lift ids, `hip_dir` (+1 if a higher position swings the foot forward), `lift_dir` (+1 if a higher position raises the foot), trims (added to 511), and the current state `x` (hip, + forward) and `z` (lift, + up from the stance pose), in servo steps.
- **`send_pose(tm)`:** turns every leg's x/z into 12 goal positions (applying dir, trim, `height` and the clamps) and sends them as one `SYNC_WRITE` (`sync_move()`, which takes a position per id) with goal time tm. It's the only thing that commands the legs.
- **`gaits[]`:** a gait is the fraction of the cycle each leg is in the air (`swing`) and each leg's offset in the cycle, indexed by [side][row]: wave (one leg at a time, back to front, right side then left), ripple (two), tripod (three).
- **`walk()`:** streams a pose every `TICK_MS` (20 ms, goal time 20). Global phase advances by dt/period (period = `step` / `swing`). On the ground a leg's hip sweeps back at 2·amp per stance time. That rate is the same for every foot on the ground, so no foot slips. In its swing window a leg lifts (z = lift·sin πw) and its hip follows a cubic (Hermite) from its lift-off x to `amp`, starting and ending at the ground rate, so the foot doesn't scuff or jolt at lift-off and touch-down. The curve overshoots `±amp` by a few percent, more in faster gaits (tripod ≈ 9% of amp at each end). amp = r·(stride ∓ turn) (left/right). The ramp r rises over the first cycle, so the legs that step last aren't dragged too far back. On stop it falls over one step; then each leg not within 3 steps of centre takes one more step, to centre. The settle check runs before a leg can start a swing, so a leg already at centre doesn't step in place. A leg that starts inside its swing window waits for its next one (state 2). A key (tty only) or `cycles` starts the graceful stop; ctrl-c (SIGINT handler, only during `walk`) freezes the pose and returns to the prompt. Each tick reads one servo's load, round robin (a 2-byte read after the sync write; ticks stay about 20 ms), and the walk ends with a per-servo peak load report. Those reads also note status errors, which print after the command.
- **`check_servos()`:** `walk`, `legtest` and `stand` first read voltage/temp from all 12 servos and refuse if any don't reply, since sync writes get no replies. It prints any status errors (e.g. overload) straight away rather than after the command.
- **`set`:** the `params[]` table (gait, step, stride, lift, height) with ranges. The defaults (tripod, step 1000, stride 100, lift 100, height -60: a slow, low, splayed walk) are the user's own tuning; they chose them for a natural-looking gait, so keep them unless asked. The gentle first-walk settings were wave, step 800, stride 50, lift 70, height 0.
