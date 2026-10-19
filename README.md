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

Top post is most recent.

### 2026-10-19

Reverse engineered a bit of the communication protocol using https://github.com/open-wheelchair/scope-can-analyzer

Started with implementing the firmware. For now, we use an MCP2515 dev board (even though it does high speed CAN instead of fault-tolerant CAN). Eventually, we'll design a custom PCB.

At this time, a lot of initialization / authentication / addresses are very specific to the one wheelchair on my research bench, so this project is currently likely not reproducible as-is. That said, we will eventually also reverse engineer this.
