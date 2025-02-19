import RPi.GPIO as GPIO
import can
import time
import threading
import logging
import signal
import sys

# Setup logging configuration
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# CAN bus configuration
CAN_INTERFACE = 'can0'  # The CAN interface to use (e.g., 'can0')
BITRATE = 500000  # Bitrate for the CAN bus communication

# GPIO setup for PWM
PWM_FREQUENCY = 1000  # PWM frequency in Hz (1kHz)

# Duty cycle mapping for multi-level keys
DUTY_CYCLE_MAP = {
    0: 0,    # 0% duty cycle
    1: 25,   # 25% duty cycle
    2: 50,   # 50% duty cycle
    3: 75,   # 75% duty cycle
    4: 100   # 100% duty cycle
}

# LED color definitions
LED_COLORS = {
    0: 0x00,  # off
    1: 0x01,  # red
    2: 0x02,  # green
    3: 0x03,  # blue
    4: 0x04,  # yellow
    5: 0x05,  # cyan
    6: 0x06,  # magenta
    7: 0x07,  # white/light blue
    8: 0x08,  # amber/orange
    9: 0x09   # yellow/green
}

# LED state definitions
LED_STATES = {
    0: 0x00,  # off
    1: 0x01,  # on
    2: 0x02,  # blink
    3: 0x03   # alternate blink
}

# Configuration for CAN objects
CAN_OBJECTS = [
    {'name': 'fanKey', 'type': 'multi', 'levels': 4, 'key_number': 0x06, 'mode': 'latch', 'led_colors': {0: 0x07, 1: 0x03, 2: 0x02, 3: 0x04, 4: 0x01}, 'forward_address': 0x18FFA07A, 'pwm_pins': [18, 23]},  # white/off, blue, green, yellow, red
    {'name': '1Key', 'type': 'binary', 'key_number': 0x05, 'mode': 'latch', 'led_colors': {0: 0x00, 1: 0x02}, 'pwm_pins': [24]},  # off, green
    {'name': 'BoostKey', 'type': 'multi', 'levels': 2, 'key_number': 0x04, 'mode': 'latch', 'led_colors': {0: 0x00, 1: 0x03, 2: 0x06}, 'forward_address': 0x07C, 'forward_messages': {0: b'\x00\x00', 1: (250 * 10).to_bytes(2, byteorder='little'), 2: b'\x00\x00'}},  # off, blue, magenta
    {'name': 'TCKey', 'type': 'multi', 'levels': 4, 'key_number': 0x03, 'mode': 'momentary', 'led_colors': {0: 0x00, 1: 0x04, 2: 0x05, 3: 0x07, 4: 0x08}},  # off, yellow, cyan, white/light blue, amber/orange
    {'name': 'StartKey', 'type': 'momentary', 'key_number': 0x0C, 'mode': 'momentary', 'led_colors': {0: 0x00, 1: 0x05}, 'forward_address': 0x07B, 'forward_messages': {0: b'\x00', 1: b'\x01'}},  # off, cyan, forward to 0x7B
    {'name': '2Key', 'type': 'binary', 'key_number': 0x0b, 'mode': 'momentary', 'led_colors': {0: 0x00, 1: 0x06}, 'pwm_pins': [25]},  # off, magenta
    {'name': 'LCKey', 'type': 'binary', 'key_number': 0x0a, 'mode': 'latch', 'led_colors': {0: 0x00, 1: 0x07}},  # off, white/light blue
    {'name': 'ALSKey', 'type': 'binary', 'key_number': 0x09, 'mode': 'momentary', 'led_colors': {0: 0x00, 1: 0x08}}  # off, amber/orange
]

class GPIOManager:
    """Class to manage GPIO setup and PWM."""
    def __init__(self, pwm_frequency):
        self.pwm_frequency = pwm_frequency
        self.pwms = {}

    def initialize(self, can_objects):
        GPIO.setmode(GPIO.BCM)
        for obj in can_objects:
            if 'pwm_pins' in obj:
                for pin in obj['pwm_pins']:
                    GPIO.setup(pin, GPIO.OUT)
                    pwm = GPIO.PWM(pin, self.pwm_frequency)
                    pwm.start(0)
                    self.pwms[pin] = pwm
        logging.info("PWM initialized successfully")

    def set_duty_cycle(self, pin, duty_cycle):
        if pin in self.pwms:
            self.pwms[pin].ChangeDutyCycle(duty_cycle)
            logging.info(f"Duty cycle set to {duty_cycle}% on pin {pin}")

    def reset_all(self):
        for pwm in self.pwms.values():
            pwm.ChangeDutyCycle(0)
        logging.info("All duty cycles reset")

    def cleanup(self):
        for pwm in self.pwms.values():
            pwm.stop()
        GPIO.cleanup()
        logging.info("GPIO cleanup complete")

class CANBusGateway:
    def __init__(self, can_interface, bitrate, can_objects, gpio_manager):
        self.bus = self.initialize_can_bus(can_interface, bitrate)  # Initialize the CAN bus
        self.can_objects = can_objects
        self.gpio_manager = gpio_manager
        self.message_counts = {obj['name']: 0 for obj in can_objects}  # Message count for each key
        self.timestamps = {obj['name']: 0.0 for obj in can_objects}  # Timestamps for each key
        self.states = {obj['key_number']: 0 for obj in can_objects}  # State for each key, keyed by key number
        self.lock = threading.Lock()  # Lock for thread safety
        signal.signal(signal.SIGINT, self.cleanup)  # Handle SIGINT for cleanup
        self.last_key_state = {obj['key_number']: 0x00 for obj in can_objects}  # Store the last key state for multi-level keys

    def initialize_can_bus(self, interface, bitrate):
        """Initialize the CAN bus interface."""
        try:
            bus = can.interface.Bus(channel=interface, bustype='socketcan', bitrate=bitrate)
            logging.info("CAN bus initialized successfully")
            return bus
        except OSError as e:
            logging.error(f"Error: Could not find CAN interface {interface}. Check configuration. {e}")
            sys.exit(1)

    def calculate_duty_cycle(self, key_number, data):
        """Calculate the PWM duty cycle based on the key state."""
        with self.lock:
            state = self.states[key_number]
            obj = next((obj for obj in self.can_objects if obj['key_number'] == key_number), None)
            if obj:
                if obj['type'] == 'binary':
                    state = 1 if data == 0x01 else 0
                elif obj['type'] == 'multi':
                    if obj['mode'] == 'latch':
                        if data == 0x01:
                            last_state = self.last_key_state.get(key_number, None)
                            if last_state == 0x00:
                                state = (state + 1) % (obj['levels'] + 1)
                    else:
                        if data == 0x01 and state < obj['levels']:
                            state += 1
                        elif data == 0x00:
                            state = 0
                self.states[key_number] = state
                self.last_key_state[key_number] = data
                return DUTY_CYCLE_MAP[state]

    def handle_message(self, message):
        """Handle incoming CAN messages."""
        logging.debug(f"Received raw CAN message: {message}")
        if message.data[0] == 0x04 and message.data[1] == 0x1B:
            key_number = message.data[3]
            key_state = message.data[4]
            obj = next((obj for obj in self.can_objects if obj['key_number'] == key_number), None)
            if obj:
                key_id = obj['name']
                logging.debug(f"Received message for {key_id} with key_number {key_number} and key_state {key_state}")

                current_time = time.time()
                with self.lock:
                    self.message_counts[key_id] += 1
                    time_difference = current_time - self.timestamps[key_id]
                    self.timestamps[key_id] = current_time

                    time_threshold = 0.02  # 20 ms. Adjust as needed
                    if time_difference < time_threshold and self.message_counts[key_id] > 1:
                        logging.warning(f"Multiple messages for {key_id} received rapidly!")

                    long_time_threshold = 0.2  # 200ms. Adjust as needed
                    if time_difference > long_time_threshold:
                        self.message_counts[key_id] = 1

                self.process_key(obj, key_id, key_state)
            else:
                logging.debug(f"Received other data with key_number {key_number} and key_state {key_state}")

    def process_key(self, obj, key_id, key_state):
        """Process the key event based on its mode (momentary or latch)."""
        logging.debug(f"Processing key {key_id} with key_state {key_state}")
        if obj['mode'] == 'momentary':
            self.handle_momentary_key(obj, key_id, key_state)
        elif obj['mode'] == 'latch':
            self.handle_latch_key(obj, key_id, key_state)

        duty_cycle = self.calculate_duty_cycle(obj['key_number'], key_state)
        self.update_pwm_duty_cycle(obj, duty_cycle)
        self.send_can_message(key_id, self.states[obj['key_number']])
        self.forward_message_if_needed(obj)

    def handle_momentary_key(self, obj, key_id, key_state):
        """Handle momentary key events."""
        logging.debug(f"Handling momentary key {key_id} with key_state {key_state}")
        if key_state == 0x01:
            self.set_led_color(obj['key_number'], obj['led_colors'][1], 1, 0)
        else:
            self.set_led_color(obj['key_number'], obj['led_colors'][0], 0, 0)

    def handle_latch_key(self, obj, key_id, key_state):
        """Handle latch key events."""
        logging.debug(f"Handling latch key {key_id} with key_state {key_state}")
        if key_state == 0x01:
            if obj['type'] == 'multi' and 'levels' in obj:
                state = (self.states[obj['key_number']] + 1) % (obj['levels'] + 1)
            else:
                state = 1 - self.states[obj['key_number']]
            self.states[obj['key_number']] = state
            self.set_led_color(obj['key_number'], obj['led_colors'][state], 1 if state > 0 else 0, 0)

    def update_pwm_duty_cycle(self, obj, duty_cycle):
        """Update the PWM duty cycle for the specified key."""
        if 'pwm_pins' in obj:
            for pin in obj['pwm_pins']:
                self.gpio_manager.set_duty_cycle(pin, duty_cycle)

    def receive_messages(self):
        """Receive and handle CAN messages."""
        message_filters = [
            {"can_id": 0x18EF2100, "can_mask": 0x1FFFFFFF, "extended": True},
            {"can_id": 0x18EFFF21, "can_mask": 0x1FFFFFFF, "extended": True}
        ]
        self.bus.set_filters(message_filters)

        while True:
            try:
                message = self.bus.recv(timeout=1.0)
                if message is not None:
                    self.handle_message(message)
            except can.CanError as e:
                logging.error(f"CAN Error: {e}")
                break

    def send_can_message(self, key_id, state):
        """Send a CAN message based on the key state."""
        try:
            obj = next((obj for obj in self.can_objects if obj['name'] == key_id), None)
            if obj is not None:
                if 'forward_address' in obj:
                    data = obj.get('forward_messages', {}).get(state, [0x00])
                    arbitration_id = obj['forward_address']

                    msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
                    self.bus.send(msg)
                    logging.info(f"Sent CAN message for {key_id} with state {state}: {msg}")
        except can.CanError as e:
            logging.error(f"CAN Error: {e}")

    def forward_message_if_needed(self, obj):
        """Forward the CAN message if forwarding is configured."""
        if 'forward_address' in obj and 'forward_messages' in obj:
            self.forward_message(obj['forward_address'], obj['forward_messages'][self.states[obj['key_number']]])

    def forward_message(self, address, data):
        """Forward a CAN message to the specified address."""
        try:
            msg = can.Message(arbitration_id=address, data=data, dlc=8, is_extended_id=False)
            self.bus.send(msg)
            logging.info(f"Forwarded CAN message to address {address} with data {data}: {msg}")
        except can.CanError as e:
            logging.error(f"CAN Error: {e}")

    def set_led_color(self, key_number, color, state, secondary_color):
        """Set the LED color and state for the specified key."""
        try:
            data = [
                0x04,  # Header byte 1
                0x1B,  # Header byte 2
                0x01,  # Set single LED state
                key_number,  # PKP Key number
                LED_COLORS.get(color, 0x00),  # LED Color
                LED_STATES.get(state, 0x00),  # LED State
                LED_COLORS.get(secondary_color, 0x00),  # LED Secondary Color (only for alt blink)
                0xFF   # Not used
            ]
            arbitration_id = 0x18EF2100  # Example PGN for LED control
            msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
            self.bus.send(msg)
            logging.info(f"Sent CAN message to set LED for key {key_number} with color {color} and state {state}")
        except can.CanError as e:
            logging.error(f"CAN Error: {e}")

    def run(self):
        """Run the CAN bus gateway to receive and process messages."""
        receive_thread = threading.Thread(target=self.receive_messages, daemon=True)
        receive_thread.start()

        try:
            while True:
                time.sleep(0.1)  # Keep the main thread alive and responsive
        except KeyboardInterrupt:
            self.cleanup()

    def cleanup(self, signum=None, frame=None):
        """Clean up resources on exit."""
        self.gpio_manager.cleanup()
        self.bus.shutdown()
        logging.info("Shutdown complete")
        sys.exit(0)

if __name__ == '__main__':
    gpio_manager = GPIOManager(PWM_FREQUENCY)
    gpio_manager.initialize(CAN_OBJECTS)
    gateway = CANBusGateway(CAN_INTERFACE, BITRATE, CAN_OBJECTS, gpio_manager)
    gateway.run()
