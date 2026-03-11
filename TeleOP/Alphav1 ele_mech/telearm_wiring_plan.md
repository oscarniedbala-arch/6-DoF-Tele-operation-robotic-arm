# Telearm electronics wiring plan (module-level)

Date: 03-01-2026

This is a **module/wiring harness schematic** plan intended to be translated into KiCad.

## System split
- **Master ESP32-S3**: reads user manipulator, sends targets over CAN
- **Robot ESP32-S3**: real-time motion, reads encoders, drives motors/servos
- **Compute (Raspberry Pi / PC)**: video + UI + logging (not hard real-time)

## Current stock
- ESP32‑S3 dev boards (multiple)
- PCA9685 (16‑ch servo PWM driver)
- TCA9548A I2C multiplexer boards (5 pcs)
- AS5600 magnetic encoders (6 pcs total across orders)
- TB6600 stepper drivers (4 pcs total)
- Micro limit switches (10 pcs)
- DC‑DC modules (Mini360 buck, XL6009 buck/boost)
- DS3240 high torque servos: 2× DS3240‑180 + 1× DS3240‑270

## Axis allocation (6-DOF + tool)
Use the extra 7th actuator for tool? Still unsure
- J1 base yaw = stepper + TB6600
- J2 shoulder pitch = stepper + TB6600
- J3 elbow pitch = stepper + TB6600
- J4 wrist yaw (or forearm roll) = stepper + TB6600
- J5 wrist pitch = DS3240‑270 (PCA9685 channel)
- J6 wrist roll = DS3240‑180 (PCA9685 channel)
- Tool actuator (gripper/cutter) = DS3240‑180 (PCA9685 channel)

## Buses and signals
### CAN (master ↔ robot)
- ESP32 TWAI TX/RX → external CAN transceiver → CANH/CANL
- Use twisted pair; terminate ends with 120Ω.

### I2C (robot side)
- ESP32 I2C SDA/SCL → PCA9685 + TCA9548A
- TCA9548A channels 0–5 → one AS5600 each (because AS5600 address is fixed)

### Step/Dir
- ESP32 GPIO: STEP/DIR/EN per TB6600 (use opto inputs properly)

### Safety
- E‑Stop chain: hardware drop of motor & servo power
- Limit switches: NC wiring preferred, to GPIO with pullups
- Current sensing: motor rail + servo rail (overcurrent/jam detection)

## Connector standards
Power:
- XT60 for 24V motor rail
- XT30 (or similar) for 6–8.4V servo rail
Signal:
- JST‑XH / JST‑VH for limit switches and sensors
- GX12 aviation connectors for arm-to-base harness (optional but neat)

## add-ons 
- CAN transceivers (SN65HVD230 class) × at least 4
- Proper 24V PSU (10–15A class)
- Proper 24→6V high-current buck (≥10A) for servo rail
- Hall current sensor modules (ACS758 class) ×2
- E‑Stop mushroom switch + wiring
- Ferrite clamps, shielded twisted-pair cable, heatshrink, glands, drag chain

