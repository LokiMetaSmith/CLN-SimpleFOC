# CLN-SimpleFOC

This repository contains the firmware for the [CLN closed-loop stepper motor drivers](https://github.com/creapunk/CLN-ClosedLoopNemaDriver). This firmware is based on [SimpleFOC](https://github.com/simplefoc/Arduino-FOC). Currently, only the **CLN17 V2.0** hardware is supported.

## Safety warning

Please be attentive when interacting with the CLN. It is a good idea to occasionally check the temperature of the DRV8844 (even with your finger) to make sure it is not overheating. In addition, for the first tests, it is a very good idea to use a laboratory power supply that can limit the current. 1 Ampere should be quite enough to start with. If tested irresponsibly, [**MAGIC SMOKE MAY ESCAPE**](https://en.wikipedia.org/wiki/Magic_smoke) 🤣 

## How to use

1. You need the CLN17 V2.0 driver mounted on a motor. Ideally, you should connect it to your computer via the [SWD over USB Type-C](https://hackaday.io/project/192857-swd-over-usb-type-c-new-way-of-programming-boards) adapter. If you don't have it, you can probably use the standard ST-Link V2 to do the job too, but it's a little more complicated.
1. Download the project and open it in [PlatformIO](https://platformio.org/).
1. Open the [src/config.h](https://github.com/AntonEvmenenko/CLN-SimpleFOC/blob/main/src/config.h) and **make sure that `POWER_SUPPLY_VOLTAGE` corresponds to your power supply voltage**. Also, **make sure that all the motor parameters listed in the `Motor` section correspond to your motor**.
1. Use PlatformIO to build and upload the project.
1. On first power-up, the motor performs a self-calibration and stores the parameters in the MCU’s “EEPROM” (actually the last page of FLASH). On subsequent startups, calibration is skipped. To force calibration, hold down SW1 while powering on the CLN. **The motor shaft must be free to rotate during calibration**.
1. By default the motor works in position control mode. You can control the motor by sending the commands via the Serial interface, which by default works via the MCU's USB. See [here](https://docs.simplefoc.com/commander_motor) for a list of the available commands. For example, you can use the following command to change the position (shaft's angle) setpoint to PI radians:

```
M3.14
```

## Distributed Motion Control & Cascaded Dual-Loop Architecture

This firmware has been modified to support a distributed motion control architecture. A central kinematic controller streams continuous absolute linear position targets over a CAN-FD bus to independent CLN boards.

To achieve sub-micron, zero-backlash positioning, each CLN board implements a **Cascaded Dual-Loop Control** system combining two distinct sensors:

### 1. The Inner Loop (Rotary Magnetic Sensor)
The inner loop handles high-bandwidth tasks: electrical commutation (FOC) and velocity/torque control.
- It uses the built-in `MagneticEncoderTLE5012B` on the CLN board.
- Because the motor is configured for velocity control (`MotionControlType::velocity`), the SimpleFOC library uses this inner rotary sensor for two things:
  1. **Commutation:** It precisely measures the rotor angle to determine exactly which coils to energize.
  2. **Velocity Control:** It constantly measures how fast the motor is spinning and uses its internal velocity PID loop to maintain the demanded target speed.

### 2. The Outer Loop (Linear Encoder)
The outer loop is entirely responsible for tracking the exact physical output position and eliminating backlash.
- It uses an external **MT6835 Linear Magnetic Encoder**, read via SPI directly by the CLN board.
- The control loop constantly reads the absolute position from this linear encoder and compares it to the target position received over the CAN bus.
- An independent PID controller (`PID_outer_position`) takes this position error and calculates a **target velocity** to command the inner loop.

### How they work together
If the outer loop detects a 2mm error, it might command a target velocity of 5 rad/s. The inner loop takes this command, and uses its high-bandwidth internal rotary sensor to track its own speed and execute precise FOC commutation to maintain 5 rad/s smoothly. As the linear axis approaches the target, the outer loop continuously shrinks the target velocity down to 0, at which point the inner loop gracefully holds the motor still.

This cascaded approach ensures that the system can completely ignore backlash and mechanical slop in the transmission, as the outer loop relies entirely on the direct linear position of the final output.

## Additional useful information

1. Attach the magnet to the motor shaft so that it is as close to the PCB as possible. You may need to use a spacer.
1. You could easily change the motion control mode by modifying the first lines of the `initMotorParameters()` function.
1. By default the motor current is set to 1.5A. You could change that by modifying the corresponding line in the `initMotorParameters()` function.
1. The CLN's LED displays the firmware's status. Red (🟥) means that initialization is in progress. Blue (🟦) means that calibration is in progress. Green (🟩) means that the firmware is working normally.
1. The firmware prints logs via Serial during startup. If something is wrong, check the logs first.
1. I recommend doing any debugging and interaction with the servo via the [MCUViewer V1.1.0](https://github.com/klonyyy/MCUViewer/releases/tag/v1.0.1). You can find the corresponding configuration file in the project's root. A bunch of variables have already been added there. You can also easily tweak any PID/LPF/etc coefficients via the MCUViewer.

## Limitations

1. At the moment, the servo motor can only be controlled via Serial commands, as described above. If you're interested in having additional interfaces/protocols for control, feel free to open an issue; I or someone from the community may be able to implement it.

## Information for developers

1. The latest SimpleFOC code is not compatible with CLN17 V2.0. I am currently trying to make the necessary changes to SimpleFOC to fix this, the discussion is [here](https://community.simplefoc.com/t/low-side-current-sensing-for-stepper-motors/7235), the draft PR is [here](https://github.com/simplefoc/Arduino-FOC/pull/472).

## Disclaimer

**⚠️ USE AT YOUR OWN RISK ⚠️**

THIS PROJECT IS PROVIDED AS IS, WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES. USE IT AT YOUR OWN RISK. THE AUTHORS AND CONTRIBUTORS ARE NOT RESPONSIBLE FOR ANY DAMAGE TO EQUIPMENT, INJURY, OR OTHER ISSUES THAT MAY RESULT FROM THE USE, MISUSE, OR MALFUNCTION OF THE HARDWARE, FIRMWARE OR SOFTWARE. MAKE SURE TO UNDERSTAND THE FUNCTIONALITY OF THE CLN DRIVER AND FOLLOW ALL SAFETY INSTRUCTIONS WHEN TESTING OR DEPLOYING THIS FIRMWARE
