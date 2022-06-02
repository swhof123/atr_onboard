# Onboard software for All Terrain Rover

## Background
This repository contains one of the onboard software component of a 4WD All Terrain Rover.
I develop this rover to use to mow steep slopes on my lawn. 

The rover itself carries the following components:
- Motors & Motor control: 4 hoverboard motors, controlled by 2 hoverboard boards running a slightly modified version of https://github.com/EFeru/hoverboard-firmware-hack-FOC/
- Batteries: 3 x 36V, 4,4 Ah
- High precision GPS: Ardusimple simpleRTK2B
- Onboard CPU: Two ESP32. 
  - One ESP32 relaying the motor control and the mower to the cloud main processor, running the software shared in this repro. The initial version is based on the example provided by Emanuel Feru: https://github.com/EFeru/hoverboard-firmware-hack-FOC/tree/master/Arduino/hoverserial
  - One ESP32 relaying the GPS unit to the cloud processor, running https://github.com/nebkat/esp32-xbee
- Connectivity: iPhone, opening a local hotspot. Both ESP32 and the cloud main processor need to be in the same network
- Mover: 40V battery powered lawn mover

## Improvements needed
### Must haves: 
- The ESP32 should connect to the internet & the cloud main processor via mobile network, using https://www.amazon.de/gp/product/B081JMWSVD/. The internet connect is needed for 4 things:
  - Submit driving commands from cloud main processor to motor control board.
  - Submit motor parameters from motor control board to cloud main processor.
  - Retrieve GPS correciton data from the internet
  - Submit rover GPS postion from the GPS modul to the cloud main processor.
- There should be a way for over the air updates. 
### Nice to haves:
- Instead of two ESP32, there should only be one needed. If this is not possible, one of the ESP32 need to connect to the other one to use its internet connection.
- Currently, the connection to the cloud main processor is opend and closed for each package that is.

The code can either be an Arduino file, but also any other method is possible, if proper compliation instructions are included. 
