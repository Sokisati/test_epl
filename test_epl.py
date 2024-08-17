import RPi.GPIO as GPIO
import time
import sys

"""
look to the tree for sys. arguments

for example, if you want to test the 360 servo; write:
python3 test_epl.py 360

change params. in run function

"""


def warnAndExit():

    print("Stop trying to break the program.")
    exit()
    
def validateArguments():
    if not len(sys.argv)==2:
        warnAndExit();

    if sys.argv[1] not in ['360', 'detach', 'range', 'input', 'output', 'all', 'gps', 'bme', 'mpu', 'rtc']:
        warnAndExit();

def testInput(inputPin):
    GPIO.setmode(GPIO.BCM)

    pin = inputPin
    GPIO.setup(pin, GPIO.IN)

    try:
        while True:
            input_state = GPIO.input(pin)
            if input_state == GPIO.HIGH:
                print("Pin is HIGH")
            else:
                print("Pin is LOW")
            time.sleep(0.1)

    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Exiting and cleaning up GPIO...")

    except Exception as e:
        GPIO.cleanup()
        print(f"An error occurred: {e}")

    finally:
        GPIO.cleanup()
        
def testOutput(outputPin):
    GPIO.setmode(GPIO.BCM)

    led_pin = outputPin
    GPIO.setup(led_pin, GPIO.OUT)

    try:
        GPIO.output(led_pin, GPIO.HIGH)
        print("LED is ON")

        time.sleep(2)

        GPIO.output(led_pin, GPIO.LOW)
        print("LED is OFF")

    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Exiting and cleaning up GPIO...")

    finally:
        GPIO.cleanup()

def testRange(normalServoPin):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(normalServoPin, GPIO.OUT)

    pwm = GPIO.PWM(normalServoPin, 50) 
    pwm.start(0)         

    def angleToDutyCycle(angle):
        min_angle = 0
        max_angle = 180
        min_duty_cycle = 2
        max_duty_cycle = 12
    
        if angle < min_angle:
            angle = min_angle
        elif angle > max_angle:
            angle = max_angle

        duty_cycle = ((angle - min_angle) / (max_angle - min_angle)) * (max_duty_cycle - min_duty_cycle) + min_duty_cycle
        return duty_cycle

    try:
        while True:
            for angle in range(0, 181):  
                duty_cycle = angleToDutyCycle(angle)
                pwm.ChangeDutyCycle(duty_cycle)
                time.sleep(0.02) 

            time.sleep(1)  

            for angle in range(180, -1, -1):  
                duty_cycle = angleToDutyCycle(angle)
                pwm.ChangeDutyCycle(duty_cycle)
                time.sleep(0.02)  

            time.sleep(1)  

    except KeyboardInterrupt:
        pass

    pwm.stop()
    GPIO.cleanup()

def run():
    
    inputPin = 27
    outputPin = 23

    normalServoPin = 18
    detachmentAngle = 666
    defaultAngle = 90


    hackedServoPin = 666
    


    if sys.argv[1]=='input':
        testInput(inputPin)
    
    if sys.argv[1]=='output':
        testOutput(outputPin)
        
    if sys.argv[1]=='range':
        testRange(normalServoPin)
      
        
run();