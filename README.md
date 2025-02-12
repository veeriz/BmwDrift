# CAN Bus Gateway

## Overview

The CAN Bus Gateway is a Python-based application designed to interface with a CAN bus, handling key press and release events from a keypad, controlling LEDs, and forwarding messages to an ECU. It is configured to work with a Raspberry Pi using the RPi.GPIO and python-can libraries.

Poor mans PDM, using Blinkmarine 12key can keypad flashed to j1939. A RPI3b with can hat acts as Gateway. and forwards some key presses to Maxxecu and some to control analog pins connected to Solid state relays
this is for prototyping, i will then refactor the code to run on a microcontroller.


## Features

- Listens for CAN messages indicating key press and release events.
- Controls LED colors based on key states.
- Adjusts the PWM duty cycle for specified keys.
- Forwards CAN messages to an ECU based on key events.

## Requirements

- Raspberry Pi with CAN interface
- BlinkMarine keypad with J1939 flashed, baud rate changed to 500.
- Maxxecu with Can bus activated and wired together
- RPi.GPIO library
- python-can library

## Disclaimer
Use at your own risk, this code has yet to be tested, and serves as a playground before designing a microcontroller board with appropriate safe guards.

## Configuration

### CAN Bus Configuration

- **CAN_INTERFACE**: The CAN interface name (e.g., `can0`).
- **BITRATE**: The CAN bus bitrate (e.g., `500000`).

### Key Configuration

The `CAN_OBJECTS` list defines the keys, their properties, and behavior. Each key configuration includes the following:

- **name**: A unique name for the key.
- **state**: The initial state of the key.
- **type**: The key type (`binary` or `multi`).
- **levels**: The number of levels for multi-level keys.
- **key_number**: The unique key number used in CAN messages.
- **mode**: The key mode (`momentary` or `latch`).
- **led_colors**: A dictionary mapping key states to LED colors.
- **forward_address**: The CAN address to forward messages to (11-bit standard identifier).
- **forward_messages**: A dictionary mapping key states to forward messages.
- **pwm_pins**: (Optional) A list of GPIO pins for PWM control.

Example configuration for `CAN_OBJECTS`:

```python
CAN_OBJECTS = [
    {'name': 'fanKey', 'state': 0, 'type': 'multi', 'levels': 5, 'key_number': 0x01, 'mode': 'latch', 'led_colors': {0: 0x07, 1: 0x03, 2: 0x02, 3: 0x04, 4: 0x01}, 'forward_address': 0x18FFA07A, 'pwm_pins': [18, 23]},
    {'name': '1Key', 'state': 0, 'type': 'binary', 'key_number': 0x02, 'mode': 'latch', 'led_colors': {0: 0x00, 1: 0x02}, 'pwm_pins': [24]},
    {'name': 'BoostKey', 'state': 0, 'type': 'multi', 'levels': 2, 'key_number': 0x03, 'mode': 'latch', 'led_colors': {0: 0x00, 1: 0x03, 2: 0x06}, 'forward_address': 0x07C, 'forward_messages': {0: b'\x00\x00', 1: (250 * 10).to_bytes(2, byteorder='little')}},
    {'name': 'TCKey', 'state': 0, 'type': 'multi', 'levels': 4, 'key_number': 0x04, 'mode': 'momentary', 'led_colors': {0: 0x00, 1: 0x04, 2: 0x05, 3: 0x07, 4: 0x08}},
    {'name': 'StartKey', 'state': 0, 'type': 'binary', 'key_number': 0x07, 'mode': 'latch', 'led_colors': {0: 0x00, 1: 0x05}, 'forward_address': 0x07B, 'forward_messages': {0: b'\x00', 1: b'\x01'}},
    {'name': '2Key', 'state': 0, 'type': 'binary', 'key_number': 0x08, 'mode': 'momentary', 'led_colors': {0: 0x00, 1: 0x06}, 'pwm_pins': [25]},
    {'name': 'LCKey', 'state': 0, 'type': 'binary', 'key_number': 0x09, 'mode': 'latch', 'led_colors': {0: 0x00, 1: 0x07}},
    {'name': 'ALSKey', 'state': 0, 'type': 'binary', 'key_number': 0x0A, 'mode': 'momentary', 'led_colors': {0: 0x00, 1: 0x08}}
]


