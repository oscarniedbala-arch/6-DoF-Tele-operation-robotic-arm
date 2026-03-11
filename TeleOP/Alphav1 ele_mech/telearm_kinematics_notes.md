# Telearm kinematics notes (prototype documentation)

Date: 2025-11-28

## Geometry
6‑DOF elbow manipulator + spherical wrist
- J1: base yaw (about Z0)
- J2: shoulder pitch
- J3: elbow pitch
- J4: wrist yaw (or forearm roll)
- J5: wrist pitch
- J6: wrist roll

IK splits cleanly via;
1) J1–J3 position the **wrist centre**
2) J4–J6 set end‑effector orientation

## Nominal link lengths
- L1 = 80 mm (base offset)
- L2 = 180 mm (upper arm)
- L3 = 160 mm (forearm)
- d4 = 70 mm (wrist offset to wrist centre)
- d6 = 80 mm (wrist centre to tool tip)

Approx reach (straight): L1 + L2 + L3 + d4 + d6 ≈ 570 mm  
Practical reach (with joint limits): ~350–500 mm

## Forward kinematics
- **Standard DH** (a, alpha, d, theta) per joint, or
- **Product of exponentials

## Inverse kinematics split 
Given end‑effector pose:
- position p (3×1)
- orientation R (3×3)

1) Wrist centre:
- Let the tool Z axis be ẑ = [0,0,1] in tool frame.
- Wrist centre:
  p_wc = p - d6 * (R * ẑ)

2) Solve J1:
- theta1 = atan2(p_wc_y, p_wc_x)

3) Solve J2–J3 (planar 2‑link after base rotation)
Transform p_wc into the J2 plane:
- r = sqrt(p_wc_x^2 + p_wc_y^2) - L1
- z = p_wc_z - base_height_offset (if any)

Law of cosines:
- D = (r^2 + z^2 - L2^2 - L3^2) / (2*L2*L3)
- theta3 = atan2(±sqrt(1 - D^2), D)   (elbow‑up / elbow‑down)

Then:
- theta2 = atan2(z, r) - atan2(L3*sin(theta3), L2 + L3*cos(theta3))

4) Orientation (J4–J6)
Compute R0_3 from theta1..theta3.
Then:
- R3_6 = (R0_3)^T * R

Extract theta4..theta6 from R3_6 using wrist convention

## Resolution maths
Stepper:
- 200 steps/rev (1.8°)
- microstep m (e.g., 16): steps/rev = 200*m
- gear ratio G (e.g., 10:1): output steps/rev = 200*m*G
- steps/deg = (200*m*G)/360

Example m=16, G=10:
- steps/deg ≈ (200*16*10)/360 ≈ 88.9 steps/deg

AS5600 encoder:
- 12‑bit → 4096 counts/rev
- counts/deg ≈ 4096/360 ≈ 11.38 counts/deg


## Joint limits
- J1: ±160°
- J2: -20° to +110°
- J3: -10° to +140°
- J4: ±180° (limit if no slip ring)
- J5: ±90°
- J6: ±180° (limit if no slip ring)
