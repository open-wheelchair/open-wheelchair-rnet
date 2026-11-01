# Open Wheelchair

This repository contains open-source firmware sources for controlling R-net powered wheelchairs.

## Features

- [X] Powering on a wheelchair
- [X] Driving controls
- [X] Wireless control by external remote over ESP-NOW
- [X] Control over UART
- [ ] Changing mode (driving vs moving seating)
- [ ] Change profile (Indoor / outdoor / ...)
- [ ] Change speed in driving mode
- [ ] Change seating actuator
- [ ] Peripherals
    - [ ] Horn
    - [ ] Direction indicators
    - [ ] Front lamp
    - [ ] Hazard indicators
    - [ ] Battery indicator
- [ ] Attendant mode
- [ ] Pairing algorithm to pair to arbitrary wheelchairs without sniffing
- [ ] Documentation
- [ ] Test bench to test against different wheelchairs

## Safety

Don't put a human in a wheelchair with this controller! Also, at this time, the remote control does not have any authentication / security in it, so don't rely on it to be secure.

That said, the UART and wireless control are reasonably 'safe':

- Control signals will only be sent to the wheelchair if a sufficiently recent update was received on the wireless link (~100 ms)
- If no update was received on the wireless link, no joystick commands will be sent
- Control signals received on UART will only be passed for a short time (~100 ms) and only if a recent update over wireless was received that allows UART.

TODO elaborate on safety vs security


## AI policy

This codebase is fully written without use of AI/LLMs. This means:

- AI is not used for reverse engineering
- AI is not used to write code
- AI is not used to write text/documentation/issues/...

Please also keep this in mind when opening issues or PRs to contribute. Be reasonable and don't force me to write out an entire AI policy document.

## Dev log

Bottom post is most recent.

### 2026-10-19

Reverse engineered a bit of the communication protocol using https://github.com/open-wheelchair/scope-can-analyzer

Started with implementing the firmware. For now, we use an MCP2515 dev board (even though it does high speed CAN instead of fault-tolerant CAN). Eventually, we'll design a custom PCB.

At this time, a lot of initialization / authentication / addresses are very specific to the one wheelchair on my research bench, so this project is currently likely not reproducible as-is. That said, we will eventually also reverse engineer this.

### 2026-10-22

Rewrote the hardware stack to use proper fault-tolerant CAN transceiver and the native TWAI (ESP32's term for CAN) controller in the ESP32.
Upgraded to ESP-IDF v6.1.0.
Also rewrote the authentication code to make the initial handshake both less code, less hardcoded, and more robust (hopefully!). Still not swapable to other wheelchairs without changing the code.

## 2026-10-24

Rewrote the software to be async and use state machines instead of the code being peppered with send() and recv() calls. This state-machine architecture now also makes it a lot nicer to work on features like changing mode / speed / profiles. Figured out a lot of the 'node address' fields in packets by looking at two different wheelchairs with the joystick and motor controller (PM) on different addresses.

## 2026-10-30

Added mode changing to seating mode (so now you can both drive & change seating, but not at the same time). Also investigated turning on more compiler warnings to get more warnings, but was unfortunately stopped by the code in the ESP-IDF not being warning-free, so then I'd have to turn off `-Werror`. Might still be useful to turn them on before each release, just to check that there are no warnings in our own code.

## 2026-11-01

Added logic to automatically go to a certain seating axis in seating mode. The UART input link is now on a separate physical port, in preparation of also sending data back from the chair to attached computer/device, the other built-in UART to USB port is still used for logs. The input UART also takes in a seating axis ID. The code makes sure that seating commands / drive commands don't get mixed up, and that no movement happens while the internal state of the wheelchair is settling. I really love state machines!
