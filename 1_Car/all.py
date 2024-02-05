# System-related modules
import threading
import datetime
import time
import sys
import os
import io

# GPIO and SPI communication
import RPi.GPIO as GPIO
import pymysql
import spidev
import pigpio

# Controller communication
from pyPS4Controller.controller import Controller
from lib_nrf24 import NRF24
import json

# Camera communication
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput

# Servo operations
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import AngularServo

# Camera server
import socketserver

# Measurement - CPU Temperature - Battery ADC
from gpiozero import CPUTemperature
from gpiozero import MCP3008




# GPIO mode
GPIO.setmode(GPIO.BCM)  

# Logging ride data
is_logging = False
start_time = 0
ride_data = {
    "motor_speed": [],
    "servo_angle_small": [],
    "servo_angle_big": [],
    "infrared": []
}

# Connect pymysql - Grafana
conn = pymysql.connect(host="localhost", unix_socket="/var/run/mysqld/mysqld.sock", user="Mexx", passwd="Dirkx1", db="gip_f1_car")
cur = conn.cursor()

# NRF24L01 configuration
nrf_pipes = [[0xE8, 0xE8, 0xF0, 0xF0, 0xE1], [0xF0, 0xF0, 0xF0, 0xF1, 0xE1]]
nrf_channel = 110
use_nrf = True

# Initialize NRF24L01
radio = NRF24(GPIO, spidev.SpiDev())
radio.begin(0, 25)

radio.setPayloadSize(32)
radio.setChannel(nrf_channel)
radio.setDataRate(NRF24.BR_1MBPS)
radio.setPALevel(NRF24.PA_HIGH)

radio.setAutoAck(True)
radio.enableDynamicPayloads()
radio.enableAckPayload()

radio.openReadingPipe(1, nrf_pipes[1])
radio.printDetails()
radio.startListening()

# Motor configuration
temp_motor_speed = 0
MIN_MOTOR_SPEED = 1100
MED_MOTOR_SPEED = 1500
MAX_MOTOR_SPEED = 1800
ESC_PIN = 4

os.system("sudo pigpiod")
time.sleep(1)
pi = pigpio.pi()
pi.set_servo_pulsewidth(ESC_PIN, MED_MOTOR_SPEED)

# Servo configuration
SERVO_PIN_SMALL = 18
SERVO_PIN_BIG = 15
MIN_SERVO_ANGLE = -90
MED_SERVO_ANGLE = 0
MAX_SERVO_ANGLE = 90

pi_gpio_factory = PiGPIOFactory()
servo_controller_small = AngularServo(SERVO_PIN_SMALL, min_angle = MIN_SERVO_ANGLE, max_angle = MAX_SERVO_ANGLE, pin_factory = pi_gpio_factory)
servo_controller_big = AngularServo(SERVO_PIN_BIG, min_angle = MIN_SERVO_ANGLE, max_angle = MAX_SERVO_ANGLE, pin_factory = pi_gpio_factory, min_pulse_width=0.0005, max_pulse_width=0.0025)
servo_angle = 0

# Battery configuration
mcp3008 = MCP3008(channel=0, select_pin=7)

VDD_VOLTAGE = 3.3
R1 = 10000
R2 = 4700

# KY-005 / Infrared configuration
IR_SENDER_PIN = 20
GPIO.setup(IR_SENDER_PIN, GPIO.OUT)


# CPU temperature configuration
cpu = CPUTemperature()

# Camera configuration
CAMERA_FPS = 10

# Global flag termination
terminate_threads = False

def receive_nrf():
    # Constants
    SLEEP_INTERVAL = 1 / 100
    ASCII_LOWER_BOUND = 32
    ASCII_UPPER_BOUND = 126

    # Local variables
    js_nrf_neutral = 1800
    js1X_value = js1Y_value = js2X_value = js2Y_value = js_nrf_neutral
    pb_values = [0, 0, 0, 0]
    json_data = {}    

    # Global variables
    global is_logging, start_time
    global servo_angle
    global use_nrf

    while not terminate_threads and use_nrf:
        while not radio.available(0):
            time.sleep(SLEEP_INTERVAL)

        # Receive data
        receivedMessage = []
        radio.read(receivedMessage, radio.getDynamicPayloadSize())

        # Convert data
        string = ""
        for n in receivedMessage:
            if ASCII_LOWER_BOUND <= n <= ASCII_UPPER_BOUND:
                string += chr(n)

        # Load data in JSON
        try:
            json_data = json.loads(string)
        except Exception as error:
            print(f"Error parsing JSON: {error}")
            stop()

        print(f"Received data: {json_data}")

        # Handle actions
        if 'action' in json_data:
            action = json_data['action']

            if action == "IR":
                Inverted_value = not GPIO.input(IR_SENDER_PIN)
                GPIO.output(IR_SENDER_PIN, Inverted_value)

                if is_logging:
                    ride_data["infrared"].append({"timestamp": elapsed_time(), "value": Inverted_value})

            elif action == "Stop":
                stop()

            elif action == "Record":
                if is_logging:
                    is_logging = False

                    ride_number = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
                    file_name = f"Ride_{ride_number}.json"
                    with open(file_name, 'w') as json_file:
                        json.dump(ride_data, json_file, indent=4)

                    print(f"Ride data saved to {file_name}")

                else:
                    is_logging = True                    
                    start_time = current_milliseconds()
                    print("Start ride data")

                    ride_data["infrared"].append({"timestamp": elapsed_time(), "value": 0})
                    ride_data["motor_speed"].append({"timestamp": elapsed_time(), "value": motor_speed})                    
                    ride_data["servo_angle_small"].append({"timestamp": elapsed_time(), "value": 0})
                    ride_data["servo_angle_big"].append({"timestamp": elapsed_time(), "value": 0})

            elif action == "PS4":
                use_nrf = False

        # Handle rotary encoder values
        elif 'Re' in json_data:
                Re_value = json_data['Re']

        # Handle joystick_1 values
        elif 'js1X' in json_data:
                js1X_value = json_data['js1X']
                js1Y_value = json_data['js1Y']

                motor_speed = calculate_motor_speed(js1Y_value, js_nrf_neutral)
                set_motor_speed(motor_speed)

                servo_angle = calculate_servo_angle(js1X_value)
                set_servo_angle(servo_angle)

        # Handle joystick_2 values
        elif 'js2X' in json_data:
                js2X_value = json_data['js2X']

                servo_angle = calculate_servo_angle(js2X_value)
                set_servo_angle(servo_angle, "big")

def ps4_controller_control():
    # Local variables
    js_psn_neutral = 0

    class MyController(Controller):
        def __init__(self, **kwargs):
            Controller.__init__(self, **kwargs)

        # X button
        def on_x_press(self):
            global use_nrf

            self.stop = True
            use_nrf = True

        # Left joystick movement motor
        def on_L3_up(self, js1Y_value):
            print("L3 up: " + str(js1Y_value))

            motor_speed = calculate_motor_speed(-js1Y_value, js_psn_neutral)
            set_motor_speed(motor_speed)

        def on_L3_down(self, js1Y_value):
            print("L3 down: " + str(js1Y_value))

            motor_speed = calculate_motor_speed(-js1Y_value, js_psn_neutral)
            set_motor_speed(motor_speed)

        def on_L3_y_at_rest(self):
            print("L3 Y Rest")

            motor_speed = calculate_motor_speed(0, 0)
            set_motor_speed(motor_speed)


        # Left joystick movement servo
        def on_L3_left(self, js1X_value):
            print("L3 left: " + str(js1X_value))

            servo_angle = calculate_servo_angle(js1X_value)
            set_servo_angle(servo_angle)

        def on_L3_right(self, js1X_value):
            print("L3 Right: " + str(js1X_value))

            servo_angle = calculate_servo_angle(js1X_value)
            set_servo_angle(servo_angle)

        def on_L3_x_at_rest(self):
            print("L3 X Rest")

            servo_angle = calculate_servo_angle(0)
            set_servo_angle(servo_angle)


        # R1 and L1 buttons
        def on_R1_press(self):
            pass

        def on_L1_press(self):
            pass

        def disconnect():
            stop()

    controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen(timeout=60)

def start_camera_streaming():
    from http import server
    PAGE = """\
    <html>
    <head>
    <title>Formula 1 - Camera</title>
    </head>
    <body>
    <h1>Formula 1 - Camera</h1>
    <img src="stream.mjpg" width="640" height="480" />
    </body>
    </html>
    """


    class StreamingOutput(io.BufferedIOBase):
        def __init__(self):
            self.frame = None
            self.condition = threading.Condition()

        def write(self, buf):
            with self.condition:
                self.frame = buf
                self.condition.notify_all()


    class StreamingHandler(server.BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/':
                self.send_response(301)
                self.send_header('Location', '/index.html')
                self.end_headers()
            elif self.path == '/index.html':
                content = PAGE.encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.send_header('Content-Length', len(content))
                self.end_headers()
                self.wfile.write(content)
            elif self.path == '/stream.mjpg':
                self.send_response(200)
                self.send_header('Age', 0)
                self.send_header('Cache-Control', 'no-cache, private')
                self.send_header('Pragma', 'no-cache')
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
                self.end_headers()
                try:
                    while not terminate_threads:
                        with output.condition:
                            output.condition.wait()
                            frame = output.frame
                        self.wfile.write(b'--FRAME\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', len(frame))
                        self.end_headers()
                        self.wfile.write(frame)
                        self.wfile.write(b'\r\n')
                        time.sleep(1 / CAMERA_FPS)
                except Exception as e:
                    print(f'Removed streaming client {self.client_address}: {str(e)}')
            else:
                self.send_error(404)
                self.end_headers()


    class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
        allow_reuse_address = True
        daemon_threads = True


    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
    output = StreamingOutput()
    picam2.start_recording(JpegEncoder(), FileOutput(output))

    try:
        address = ('', 8000)
        server = StreamingServer(address, StreamingHandler)
        server.serve_forever()
    finally:
        picam2.stop_recording()


# Time functions
def current_milliseconds():
    return int(round(time.time() * 1000))
def elapsed_time():
    return current_milliseconds() - start_time

# Motor and Servo Control
def calculate_motor_speed(joystick_value, neutral_value):
    # Using NRF24L01
    if use_nrf:
        OFFSET = 50

        if abs(joystick_value - neutral_value) > OFFSET:
            slope_nrf_motor = (MAX_MOTOR_SPEED - MIN_MOTOR_SPEED)/(4095 - 0)
            speed = int(slope_nrf_motor * (joystick_value - 4095) + MAX_MOTOR_SPEED)
        else:
            speed = MED_MOTOR_SPEED

    # Using playstation controller 
    else:
        OFFSET = 1000

        if abs(joystick_value - neutral_value) > OFFSET:
            slope_psn_motor = (MAX_MOTOR_SPEED - MIN_MOTOR_SPEED)/(32000 - ( - 32000))
            speed = int(slope_psn_motor * (joystick_value - 32000) + MAX_MOTOR_SPEED)
        else:
            speed = MED_MOTOR_SPEED
    
    return max(min(speed, MAX_MOTOR_SPEED), MIN_MOTOR_SPEED)

def set_motor_speed(motor_speed):
    global temp_motor_speed

    OFFSET = 25

    if abs(motor_speed - temp_motor_speed) > OFFSET:
        pi.set_servo_pulsewidth(ESC_PIN, motor_speed)
        print(f"Motor speed: {motor_speed}")

        temp_motor_speed = motor_speed

        if is_logging:
            ride_data["motor_speed"].append({"timestamp": elapsed_time(), "value": motor_speed})


def calculate_servo_angle(joystick_value):
    if use_nrf:
        slope_nrf_servo = (MAX_SERVO_ANGLE - MIN_SERVO_ANGLE)/(4095 - 0)
        servo_angle = int(slope_nrf_servo * (joystick_value - 4095) + MAX_SERVO_ANGLE)
    else:
        slope_psn_servo = (MAX_SERVO_ANGLE - MIN_SERVO_ANGLE)/(32000 - ( - 32000))
        servo_angle = -int(slope_psn_servo * (joystick_value - 32000) + MAX_SERVO_ANGLE)
    return servo_angle

def set_servo_angle(angle, servo = "small"):
    OFFSET = 2

    if abs(angle) > OFFSET:
        angle = max(min(angle, MAX_SERVO_ANGLE), MIN_SERVO_ANGLE)
    else:
        angle = 0

    print(f"Angle: {angle}")

    if servo == "small":
        servo_controller_small.angle = angle
        print(f"Angle servo {SERVO_PIN_SMALL}: {angle}°")

        if is_logging:
            ride_data["servo_angle_small"].append({"timestamp": elapsed_time(), "value": angle})

    else:
        servo_controller_big.angle = angle
        print(f"Angle servo {SERVO_PIN_BIG}: {angle}°")

        if is_logging:
            ride_data["servo_angle_big"].append({"timestamp": elapsed_time(), "value": angle})


# To-log data - Grafana
def get_cpu_temperature():
    cpu_temp = cpu.temperature
    return cpu_temp
def get_battery_voltage():
    battery_voltage = mcp3008.value * VDD_VOLTAGE * ((R1 + R2) / R2)
    return battery_voltage
def get_time_string():
    return time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())

# Database functions
def clear_database():
    cur.execute("TRUNCATE TABLE Car")

def update_database():
    clear_database()
    while not terminate_threads:
        cur_time = get_time_string()
        cpu_temp = get_cpu_temperature()
        battery_voltage = get_battery_voltage()

        if(battery_voltage < 7):
            print("Voltage too low!!")
            stop()

        cur.execute("INSERT INTO Car(time, cpu_temp, battery_voltage) VALUES(%s,%s,%s)", (cur_time, cpu_temp, battery_voltage))
        conn.commit()
        
        time.sleep(5)


# Main functions
def control():
    while not terminate_threads:
        if use_nrf:
            receive_nrf()
        else:
            ps4_controller_control()

def stop():
    global terminate_threads

    # Force stop everything
    print("FORCE STOP!!")
    terminate_threads = True    
    pi.set_servo_pulsewidth(ESC_PIN, MED_MOTOR_SPEED)
    GPIO.cleanup()
    sys.exit(0)
        

# Start threads
control_thread = threading.Thread(target = control)
control_thread.start()

database_thread = threading.Thread(target = update_database)
database_thread.start()

#camera_thread = threading.Thread(target = start_camera_streaming)
#camera_thread.start()

try:
    while True:
        time.sleep(30)

except KeyboardInterrupt:
    stop()