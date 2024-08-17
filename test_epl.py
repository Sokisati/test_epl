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


def run():
    
    inputPin = 27
    outputPin = 23

    normalServoPin = 66
    detachmentAngle = 666
    defaultAngle = 666


    hackedServoPin = 666

    if sys.argv[1]=='input':
        testInput(inputPin)
    
    if sys.argv[1]=='output':
        testOutput(outputPin)