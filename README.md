# carvera-teensy-moteus

Use [Moteus](https://mjbots.com/) motor controller instead of factory standard BLD-300B motor driver to run Carvera CNC spindle motor. This is just **a demo** to showcase a possibility of this setup. The code is **not ready** for everyday use.

# Hardware
- [Carvera CNC](https://www.makera.com/products/carvera) (might work with Carvera Air too)
- [Teensy 4.1](https://www.digikey.ca/en/products/detail/sparkfun-electronics/DEV-16996/13158152)
- [Moteus C1](https://mjbots.com/products/moteus-c1) or [Moteus N1](https://mjbots.com/products/moteus-n1)
- [M600 encoder for Moteus](https://mjbots.com/products/ma600-breakout)
- [mjcanfd-usb-1x](https://mjbots.com/products/mjcanfd-usb-1x) and [JST PH3 Cable](https://mjbots.com/products/jst-ph3-cable) for initial Moteus calibration
- [Adafruit CAN Pal - CAN Bus Transceiver - TJA1051T/3](https://www.adafruit.com/product/5708)
- [Adafruit 8-channel Bi-directional Logic Level Converter - TXB0108](https://www.adafruit.com/product/395)
- various connectors and cables, depending on wiring setup.
  - connectors near Carvera spindle motor are JST SM, 3 pin for motor phases and 5 pin for hall sensor ([example kit on amazon](https://www.amazon.com/dp/B07CTKD7P4))
  - Moteus CAN-FD connector is [JST PHR-3](https://mjbots.com/products/phr-3)
  - connectors used on BLD-300B motor driver of Carvera are 2EDG: 2 pin for 48V power, 8 pin for motor phases and hall, 7 pin for carvera smoothieboard connection. Lower pin count male sockets work with higher pin count female socket ([example kit on amazon](https://www.amazon.com/dp/B09TK222YH))

# Notes and references

- Article about [using Escon motor controller with Carvera](https://www.instructables.com/Carvera-Spindle-Power-Upgrade-Stock-Motor/) has lots of info including wiring descriptions
- With 2EDG terminals on hand it's easy to reuse the 8 pin wires going from BLD-300B to the motor for 48v power + PWM signal + speed feedback + alarm signal. Makes it easy to quickly go back for factory standard setup
- Carvera outputs 5V PWM signal that at [1000Hz](https://github.com/MakeraInc/CarveraFirmware/blob/652bb526d8be32bb45078712ec01f80e42b1ae1b/src/config.default#L323), this signal determines how much power should be applied to motor, smoothieboard uses PID control for this. This needs to go through level shifter before connecting to Teensy.
- Speed feedback signal seems to work at 3.3V, at [12 pulses per revolution](https://github.com/MakeraInc/CarveraFirmware/blob/652bb526d8be32bb45078712ec01f80e42b1ae1b/src/config.default#L325), works when directly connected to Teensy
- [Carvera Discord thread about modifying Carvera spindle control](https://discord.com/channels/910194756473225269/1337945314161721355)
- Carvera spindle motor is 8 pole motor
- Moteus Arduino library does not support Teensy. Need to install "Moteus" library via Arduino IDE library manager, then replace Moteus folder in Arduino/libraries with https://github.com/kylevernyi/moteus-teensy
- Need to connect Moteus controller to a computer for initial configuration and calibration. Need to do that with belt off.
- Moteus controller can't reach max RPM when using motor's hall sensors, had to use M600 magenitic encoder. CAD file for the bracket is in the repo.

