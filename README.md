# Tactical Mapping Micros
This repo contains the code that runs on the microcontrollers that are part of my Tactical Mapping project.

### Adafruit Feather M0 + LoRa
- Facilitates communication with the LoRa radio
- Communicates with the Teensy 4.1 via serial UART

### Teensy 4.1
- Main microcontroller that facilitates communications between all the modules
- Provide UI via OLED screen with a rotary encoder for input
- Communicates with OLED via I2C
- Communicates with GPS module via serial UART with CTS/RTS line

## Project Status
Currently all code is alpha level / initial experimentation.