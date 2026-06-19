# H Problem Requirements Summary

Contest constraints captured from `H题_自动行驶小车.pdf`:

- MCU must be TI MSPM0 series. This project targets MSPM0G3507.
- Car outline must be within 25 cm x 15 cm x 15 cm.
- Wheeled car only; crawler track and mecanum wheels are not allowed.
- The car may only move forward during runs.
- Camera is not allowed.
- No external navigation markers or devices may be added.
- The car must give audible and visible prompts at start/end points and when passing A, B, C, or D.
- On arc segments, the car projection must remain on the black arc.

Field geometry used by firmware constants:

- Top and bottom straight chord length: 100 cm.
- Arc radius: 40 cm.
- Arc length: pi x 40 cm = about 125.66 cm.
- Diagonal chord used by requirement 3: sqrt(100^2 + 80^2) = about 128.06 cm.

Scored route requirements:

- Requirement 1: start at A, drive to B, stop within 15 s.
- Requirement 2: A -> B -> C -> D -> A, one lap within 30 s.
- Requirement 3: A -> C -> B -> D -> A, one lap within 40 s.
- Requirement 4: run requirement 3 for four laps; less time is better.
