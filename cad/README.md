## CAD

Each part is here twice: `.step` (the model, for measuring or editing) and `.stl` (the same part laid flat for printing). Units are mm. The STEP files come from Open CASCADE 7.8.

| Part | What it is |
|---|---|
| `04_six_leg_platform` | the body: a 220 mm disc, 3 mm thick, with six spokes carrying the hip servo pockets, and a central wiring cage |
| `06_electronics_lid` | a 100 mm disc that closes the top of the cage |
| `02_hip_cradle` | bolts to a hip servo's horn and holds that leg's lift servo on its side (×6) |
| `03_fixed_leg` | a 4 mm plate on the lift servo's horn, running out about 28 mm and then bending down to a rounded foot (×6) |
| `05_one_hip_assembly` | the body and lid with one leg fitted (hip and lift servos, horns, cradle, leg), as named parts; STEP only |

The parts share one coordinate frame: body centre on the z axis, the underside of the plate at z = 0 (the assembly sits 0.8 mm lower than the loose cradle and leg files).

### Body

- **Plate:** an outer ring (r 102–110) joined to a hub by six spokes, with big lightening windows between them. Each spoke carries a hip servo pocket, and two 3 mm ribs on top run along it either side of the servo. A 10 mm hole in the middle.
- **Cage:** a tube, 100 mm across and 1.6 mm wall, standing 63 mm above the underside of the plate, with rows of slots round it for wires. The lid's 4 mm spigot drops into its top.
- **Inside the cage:** six pairs of M3 holes, 8 mm apart, on a 70 mm circle (one pair in line with each leg), for the electronics.

### How it goes together

- **Legs:** six, 60° apart. The middle legs are on the x axis, so forward is along y. The front and rear legs are 60° either side of the middle ones.
- **Hip servos:** each sits in a pocket, hanging under the plate (to 18.5 mm below it), with its mounting holes (M2) 70 and 98.5 mm from the centre. The shaft is 90 mm from the centre (the SC09's shaft is 5.65 mm off the midpoint of its mounting holes). The hip axis is vertical.
- **Lift servos:** the cradle holds each one with its axis horizontal, directly above the hip axis, 12 mm above the cradle's underside and 23 mm above the underside of the plate.
- **Leg plates:** each sits about 18 mm along the lift axis from the hip axis, so the foot is offset sideways from the hip, not straight out from it.
- **Horns:** the servo horns take a 6 mm hub with two M2 screws 19.8 mm apart (the holes in the cradle base and the leg).

### Leg geometry

With the leg as drawn (taken to be the 511 pose), the bottom of the foot is 42.5 mm out from the lift axis and 81 mm below it, 91.5 mm in all (the old, shorter leg was 41 and 71). The lift servo turns the whole leg about that axis, so `height` in `hector.c` (one step = 0.293°) moves the foot like this, with the body's height off the ground on level feet:

| `height` | Foot out from lift axis | Foot below lift axis | Plate underside off the ground | Hip servos off the ground |
|---|---|---|---|---|
| +50 | 21 | 89 | 66 | 47 |
| 0 | 42 | 81 | 58 | 39 |
| −60 | 65 | 64 | 41 | 23 |
| −100 | 77 | 50 | 27 | 8 |
| −120 | 81 | 42 | 19 | 0 |

Below about −120 the hip servos, not the feet, touch the ground. The second column is the lever arm the lift servo works against: its torque is the weight on that foot times this distance. A lower, more splayed body means more torque, up to 91.5 mm when the foot is level with the axis.

### Clearances

Measured by turning the parts in the assembly, so only as good as the "as drawn = 511" assumption:

- **Lift, foot down (`height` > 0):** the leg reaches the outer ring of the plate at about 26° (89 steps). The old body measured 92 steps the same way, yet the legs met the chassis at about 50 on the robot, so the drawn pose is probably not exactly 511; expect about the same margin as before.
- **Lift, foot up:** nothing in the way.
- **Hip, towards the side of the hip the leg plate is on:** the leg's lower half reaches the outer ring, sooner the lower the foot: about 61 steps at `height` +50, 123 at 0, 177 at −60 and 218 at −100 (the old body: 75, 137 and 198). The other way it's clear to about 90°. All six legs are fitted the same way round, so that's the same sense of rotation for every hip (anticlockwise from above in these files): forward on one side of the robot and back on the other. As `hip_dir` is −1 on the left and +1 on the right, it's also either a higher position for every hip or a lower one.
- **Neighbouring legs:** they don't meet, with each hip turned up to 118° towards the other.
