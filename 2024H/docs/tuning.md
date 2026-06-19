# Tuning Guide

Tune in this order:

1. Encoder scale

   Drive a known 100 cm straight line at low speed. Adjust `APP_ENCODER_TICKS_PER_CM` until the reported average distance is within 2 cm.

2. Straight yaw hold

   Use requirement 1 at a low speed. Tune `APP_STRAIGHT_KP`, then `APP_STRAIGHT_KD`, until the car reaches B without visible weaving.

3. Arc line tracking

   Place the LF04 array on the black arc. Confirm the hardware layer maps black line LOW to `APP_LINE_BLACK_READING`, while white HIGH maps to 0. Tune `APP_LINE_KP` until the car recenters quickly, then add small `APP_LINE_KD` to reduce oscillation.

4. Segment lengths

   Verify the car projection covers each waypoint when the prompt fires. Adjust `APP_STRAIGHT_AB_CM`, `APP_DIAGONAL_AC_CM`, and `APP_ARC_HALF_CM` only after encoder scale is correct.

5. Speed

   Raise `APP_CRUISE_SPEED_CM_S` and `APP_ARC_SPEED_CM_S` gradually. Requirement 4 rewards lower time, but arc stability is more important than peak speed.

Safety rules in the controller:

- Motor commands are clamped to forward-only values.
- Final segment completion stops both motors.
- Waypoint prompts are non-blocking, so the route does not pause unless it reaches the final point.
