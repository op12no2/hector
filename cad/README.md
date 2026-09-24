## CAD

Each part is here twice: `.step` (the model, for measuring or editing) and `.stl` (the same part laid flat for printing). Units are mm. The STEP files come from Open CASCADE 7.8.

| Part | What it is |
|---|---|
| `04_six_leg_platform` | the body: a 164 mm disc, 3 mm thick, with six hip servo pockets and a 3×3 grid of M3 holes (20 mm pitch) in the middle |
| `02_hip_cradle` | bolts to a hip servo's horn and holds that leg's lift servo on its side (×6) |
| `03_fixed_leg` | a 4 mm plate on the lift servo's horn, running out about 20 mm and then bending down to the foot (×6) |

### How it goes together

- **Legs:** six, 60° apart. The middle legs are on the platform's x axis, so forward is along y. The front and rear legs are 60° either side of the middle ones.
- **Hip servos:** each sits in a pocket with its mounting holes (M2) 42 and 70.5 mm from the centre. The shaft is at the rim end, about 62 mm from the centre (the SC09's shaft is 5.65 mm off the midpoint of its mounting holes). The hip axis is vertical.
- **Lift servos:** the cradle holds each one with its axis horizontal, directly above the hip axis and 12 mm above the cradle's underside.
- **Leg plates:** each sits about 18 mm along the lift axis from the hip axis, so the foot is offset sideways from the hip, not straight out from it.
- **Horns:** the servo horns take a 6 mm hub with two M2 screws 19.8 mm apart (the holes in the cradle base and the leg).

### Leg geometry

With the leg as drawn (taken to be the 511 pose), the foot is about 41 mm out from the lift axis and 71 mm below it. The lift servo turns the whole leg about that axis, so `height` in `hector.c` (one step = 0.293°) moves the foot like this:

| `height` | Foot out from lift axis | Foot below lift axis |
|---|---|---|
| +50 | 22 | 79 |
| 0 | 41 | 71 |
| −60 | 60 | 55 |
| −100 | 70 | 42 |
| −150 | 79 | 23 |

The first column is the lever arm the lift servo works against: its torque is the weight on that foot times this distance. A lower, more splayed body means more torque, up to 83 mm when the foot is level with the axis.
