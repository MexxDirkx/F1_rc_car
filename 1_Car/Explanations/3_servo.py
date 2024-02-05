from gpiozero import AngularServo

# Servo settings
servo_pin = 18
servo = AngularServo(servo_pin, min_angle=0, max_angle=180)

def set_servo_angle(angle):
    servo.angle = angle
    print(f"Set servo at pin {servo_pin} to angle {angle}")

def get_valid_angle_input():
    while True:
        try:
            angle = int(input("Enter an angle (0 to 1800 degrees): "))
            if 0 <= angle <= 180:
                return angle
            else:
                print("Angle out of range (0 to 180 degrees). Try again.")
        except ValueError:
            print("Invalid input. Please enter a number.")

while True:
    angle = get_valid_angle_input()
    set_servo_angle(angle)