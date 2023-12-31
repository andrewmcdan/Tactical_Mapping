# This project is under construction

# Tactical Mapping Micros
This repo contains the code that runs on the microcontrollers that are part of my Tactical Mapping project. See the ReadMe.md in each section for a detailed description of that controller's role.

### Adafruit Feather M0 + LoRa
[![Build Status of Last Commit](https://github.com/andrewmcdan/Tactical_Mapping/actions/workflows/c-cpp.yml/badge.svg)](https://github.com/andrewmcdan/Tactical_Mapping/actions/workflows/c-cpp.yml)
- Facilitates communication with the LoRa radio
- Communicates with the Teensy 4.1 via serial UART

### Teensy 4.0
[![Build Status of Last Commit](https://github.com/andrewmcdan/Tactical_Mapping/actions/workflows/teensyBuild.yml/badge.svg)](https://github.com/andrewmcdan/Tactical_Mapping/actions/workflows/teensyBuild.yml)
- Main microcontroller that facilitates communications between all the modules
- Provide UI via OLED screen with a rotary encoder for input
- Communicates with OLED via I2C
- Communicates with GPS module via serial UART
- Communicates with BLE module via serial UART

The auto-build action that runs each time I push new code to the repo results in an artifact file that is available for me to download. The artifact includes a json file that is a report of the memory usage for the sketch and also the hex file that was compiled.  

## A note on modules

GPS and / or BLE modules may not be present in a given configuration. In the case of a beacon (which would be deployed without a person/phone nearby), there's no reason to waste power on the BLE module. For units that carry the device with them, and have an accompanying phone, the phone can provide GPS location, and thus there is no reason for there to be a GPS module. 

Each configuration runs the same code, and it is the up to the code to determine if the module is present.

## Project Status
Currently all code is alpha level / initial experimentation. Although the plan is to get this project to a fully functional state, that will depend on availability of time given my work / school schedule.
