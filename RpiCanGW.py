import RPi.GPIO as GPIO
import can
import time
import threading
import logging

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# CAN bus configuration
CAN_INTERFACE = 'can0'
BITRATE = 500000  # Set your desired bitrate for J1939

# Define CAN bus message IDs for keys
CAN_OBJECTS = [
    {'name': 'fanKey', 'state': 0, 'type': 'multi', 'levels': 5, 'key_number': 0x01, 'mode': 'latch', 'led_colors': {0: 0x07, 1: 0x03, 2: 0x02, 3: 0x04, 4: 0x01}, 'forward_address': 0x18FFA07A, 'pwm_pins': [18, 23]},  # white/off, blue, green, yellow, red
    {'name': '1Key', 'state': 0, 'type': 'binary', 'key_number': 0x02, 'mode': 'latch', 'led_colors': {0: 0x00, 1: 0x02}, 'pwm_pins': [24]},  # off, green
    {'name': 'BoostKey', 'state': 0, 'type': 'multi', 'levels': 2, 'key_number': 0x03, 'mode': 'latch', 'led_colors': {0: 0x00, 1: 0x03, 2: 0x06}, 'forward_address': 0x07C, 'forward_messages': {0: b'\x00\x00', 1: (250 * 10).to_bytes(2, byteorder='little')}},  # off, blue, magenta
    {'name': 'TCKey', 'state': 0, 'type': 'multi', 'levels': 4, 'key_number': 0x04, 'mode': 'momentary', 'led_colors': {0: 0x00, 1: 0x04, 2: 0x05, 3: 0x07, 4: 0x08}},  # off, yellow, cyan, white/light blue, amber/orange
    {'name': 'StartKey', 'state': 0, 'type': 'binary', 'key_number': 0x07, 'mode': 'latch', 'led_colors': {0: 0x00, 1: 0x05}, 'forward_address': 0x07B, 'forward_messages': {0: b'\x00', 1: b'\x01'}},  # off, cyan, forward to 0x7B
    {'name': '2Key', 'state': 0, 'type': 'binary', 'key_number': 0x08, 'mode': 'momentary', 'led_colors': {0: 0x00, 1: 0x06}, 'pwm_pins': [25]},  # off, magenta
    {'name': 'LCKey', 'state': 0, 'type': 'binary', 'key_number': 0x09, 'mode': 'latch', 'led_colors': {0: 0x00, 1: 0x07}},  # off, white/light blue
    {'name': 'ALSKey', 'state': 0, 'type': 'binary', 'key_number': 0x0A, 'mode': 'momentary', 'led_colors': {0: 0x00, 1: 0x08}}  # off, amber/orange
]

# GPIO setup for PWM
DEFAULT_PWM_PINS = [18, 23, 24, 25, 12, 16]  # Example GPIO pins
PWM_FREQUENCY = 1000  # Set frequency to 1kHz

# Duty cycle mapping for multi-level keys (example for 4 levels)
DUTY_CYCLE_MAP = {
    0: 0,    # 0% duty cycle
    1: 25,   # 25% duty cycle
    2: 50,   # 50% duty cycle
    3: 75,   # 75% duty cycle
    4: 100   # 100% duty cycle
}

# LED color and state definitions
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

LED_STATES = {
    0: 0x00,  # off
    1: 0x01,  # on
    2: 0x02,  # blink
    3: 0x03   # alternate blink
}

class CANBusGateway:
    def __init__(self):
        self.bus = self.initialize_can_bus()
        self.pwms = self.initialize_pwms()
        self.message_counts = {obj['name']: 0 for obj in CAN_OBJECTS}
        self.timestamps = {obj['name']: 0.0 for obj in CAN_OBJECTS}
        self.lock = threading.Lock()

    def initialize_can_bus(self):
        try:
            bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan', bitrate=BITRATE)
            logging.info("CAN bus initialized successfully")
            return bus
        except OSError as e:
            logging.error(f"Error: Could not find CAN interface {CAN_INTERFACE}. Check configuration. {e}")
            exit()

    def initialize_pwms(self):
        GPIO.setmode(GPIO.BCM)
        pwms = {}
        for obj in CAN_OBJECTS:
            if 'pwm_pins' in obj:
                for pin in obj['pwm_pins']:
                    GPIO.setup(pin, GPIO.OUT)
                    pwm = GPIO.PWM(pin, PWM_FREQUENCY)
                    pwm.start(0)
                    pwms[pin] = pwm
        logging.info("PWM initialized successfully")
        return pwms

    def calculate_duty_cycle(self, key_id, data):
        with self.lock:
            for obj in CAN_OBJECTS:
                if obj['name'] == key_id:
                    if obj['type'] == 'binary':
                        obj['state'] = 1 if data == '01' else 0
                    elif obj['type'] == 'multi':
                        if data == '01' and obj['state'] < obj['levels']:
                            obj['state'] += 1
                        elif data == '00':
                            obj['state'] = 0
                    return DUTY_CYCLE_MAP[obj['state']]

    def reset_all_duty_cycles(self):
        with self.lock:
            for obj in CAN_OBJECTS:
                obj['state'] = 0
        for pwm in self.pwms.values():
            pwm.ChangeDutyCycle(0)
        logging.info("All duty cycles reset")

    def handle_message(self, message):
        if message.arbitration_id in [0x18EF2100, 0x18EFFF21]:  # Example PGN for key press and release events
            key_number = message.data[3]
            data = message.data[4]
            for obj in CAN_OBJECTS:
                if obj['key_number'] == key_number:
                    key_id = obj['name']

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

                    if obj['mode'] == 'momentary':
                        if data == 0x01:
                            # Turn on the LED while the button is pressed
                            self.set_led_color(obj['key_number'], obj['led_colors'][1], 1, 0)
                        else:
                            # Turn off the LED when the button is released
                            self.set_led_color(obj['key_number'], 0, 0, 0)
                    elif obj['mode'] == 'latch':
                        if data == 0x01:
                            # Increment the state and cycle through the levels
                            obj['state'] = (obj['state'] + 1) % (obj['levels'] + 1)
                            self.set_led_color(obj['key_number'], obj['led_colors'][obj['state']], obj['state'], 0)

                    duty_cycle = self.calculate_duty_cycle(key_id, data)
                    if 'pwm_pins' in obj:
                        for pin in obj['pwm_pins']:
                            if pin in self.pwms:
                                self.pwms[pin].ChangeDutyCycle(duty_cycle)
                                logging.info(f"Duty cycle for {key_id} set to {duty_cycle}% on pin {pin}")
                
                    # Send CAN message based on key and state
                    self.send_can_message(key_id, obj['state'])
                
                    # Forward message to ECU if forward_address is present
                    if 'forward_address' in obj and 'forward_messages' in obj:
                        self.forward_message(obj['forward_address'], obj['forward_messages'][obj['state']])
                    
                    break

    def receive_messages(self):
        while True:
            try:
                message = self.bus.recv(timeout=1.0)
                if message is not None:
                    self.handle_message(message)
            except can.CanError as e:
                logging.error(f"CAN Error: {e}")
                break

    def send_can_message(self, key_id, state):
        try:
            obj = next((obj for obj in CAN_OBJECTS if obj['name'] == key_id), None)
            if obj is not None:
                if 'forward_messages' in obj:
                    data = obj['forward_messages'].get(state, [0x00])
                else:
                    data = [0x01] if state == 1 else [0x00]
                arbitration_id = obj['forward_address']
                
                msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
                self.bus.send(msg)
                logging.info(f"Sent CAN message for {key_id} with state {state}: {msg}")
        except can.CanError as e:
            logging.error(f"CAN Error: {e}")

    def forward_message(self, address, data):
        try:
            msg = can.Message(arbitration_id=address, data=data, dlc=8, is_extended_id=False)
            self.bus.send(msg)
            logging.info(f"Forwarded CAN message to address {address} with data {data}: {msg}")
        except can.CanError as e:
            logging.error(f"CAN Error: {e}")

    def set_led_color(self, key_number, color, state, secondary_color):
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
        receive_thread = threading.Thread(target=self.receive_messages, daemon=True)
        receive_thread.start()

        try:
            while True:
                pass  # Keep the main thread alive (or add other tasks here)
        except KeyboardInterrupt:
            # Clean up
            for pwm in self.pwms.values():
                pwm.stop()
            GPIO.cleanup()
            self.bus.shutdown()
            logging.info("Shutdown complete")


if __name__ == '__main__':
    gateway = CANBusGateway()
    gateway.run()
