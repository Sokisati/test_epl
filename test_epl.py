
import RPi.GPIO as GPIO
import time
import sys
import time
import math
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
import bme680
import gpsd
from datetime import datetime
from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory

"""
look to the tree for sys. arguments

for example, if you want to test the 360 servo; write:
python3 test_epl.py 360

change params. in run function

"""
class TimeSensor:
    #I have NO idea why we even use an RTC module. RPI's own RTC works just fine...
    def getDateAndTime(self):
        currentTime = datetime.now()
        return currentTime.strftime("%Y/%m/%d-%H:%M:%S")
    def test(self):
        currentTime = datetime.now()
        print(currentTime.strftime("%Y/%m/%d-%H:%M:%S"))

class MPUSensor:
    def __init__(self, dt=1.0):
        self.broken = False
        try:

            self.dt = dt  
            self.yaw = 0  
            self.gyroZBias = 0  

            self.mpu = MPU9250(
                address_ak=AK8963_ADDRESS,
                address_mpu_master=MPU9050_ADDRESS_68,
                address_mpu_slave=None,
                bus=1,
                gfs=GFS_250, 
                afs=AFS_2G,  
                mfs=AK8963_BIT_16,  
                mode=AK8963_MODE_C100HZ 
            )
            self.mpu.configure()
            self.calculateGyroBias()
            
        except Exception as e:
            self.broken = True
            print("Problem with mpu creation")

    def calculateGyroBias(self):
        biasSum = 0
        numSamples = 100  
        for _ in range(numSamples):
            gyroData = self.mpu.readGyroscopeMaster()
            biasSum += gyroData[2]  
            time.sleep(0.01)
        self.gyroZBias = biasSum / numSamples

    def getRoll(self):
        try:
            accelData = self.mpu.readAccelerometerMaster()
            ax, ay, az = accelData
            roll = math.atan2(ay, az) * (180 / math.pi)
            return roll
        except Exception as e:
            print(f"Error reading roll: {e}")
            return -666

    def getPitch(self):
        try:
            accelData = self.mpu.readAccelerometerMaster()
            ax, ay, az = accelData
            pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * (180 / math.pi)
            return pitch
        except Exception as e:
            print(f"Error reading pitch: {e}")
            return -666

    def getYaw(self):
        try:
            gyroData = self.mpu.readGyroscopeMaster()
            gyroZ = gyroData[2]
            self.yaw += (gyroZ - self.gyroZBias) * self.dt
            return self.yaw
        except Exception as e:
            print(f"Error reading yaw: {e}")
            return -666
    
    def test(self):
        print("Roll:", self.getRoll())
        print("Pitch:", self.getPitch())
        print("Yaw:", self.getYaw())

class BMESensor:
    def __init__(self):
        self.broken = False        
        try:

            try:
                self.sensor = bme680.BME680(bme680.I2C_ADDR_PRIMARY)
            except (RuntimeError, IOError):
                self.sensor = bme680.BME680(bme680.I2C_ADDR_SECONDARY)

            self.sensor.set_humidity_oversample(bme680.OS_2X)
            self.sensor.set_pressure_oversample(bme680.OS_4X)
            self.sensor.set_temperature_oversample(bme680.OS_8X)
            self.sensor.set_filter(bme680.FILTER_SIZE_3)
        
            # Disable gas measurements to prevent overheating (we don't need it)
            self.sensor.set_gas_status(bme680.DISABLE_GAS_MEAS)

            self.temperature = None
            self.pressure = None
            self.humidity = None
            self.altitude = None
        
        except Exception as e:
            self.broken = True
            print("Problem with bme creation")
        
    def readSensorData(self):
        
        if self.broken:
            return -666
        
        try:
            if self.sensor.get_sensor_data():
                self.temperature = self.sensor.data.temperature
                self.pressure = self.sensor.data.pressure * 100  
                self.humidity = self.sensor.data.humidity
                self.altitude = 44330 * (1 - (self.pressure / 101325) ** (1 / 5.255)) 
                return True
        except Exception as e:
            print(f"Error reading BME680 data: {e}")
        return False

    def getTemp(self):
        
        if self.broken:
            return -666
        
        try:
            if self.sensor.get_sensor_data():
                return self.sensor.data.temperature
        except Exception as e:
            print(f"Error reading temperature: {e}")
            return -666

    def getPressure(self):
        
        if self.broken:
            return -666
        
        try:
            if self.sensor.get_sensor_data():
                return self.sensor.data.pressure * 100 
        except Exception as e:
            print(f"Error reading pressure: {e}")
            return -666

    def getAlt(self):
        if self.broken:
            return -666
        
        try:
            if self.sensor.get_sensor_data():
                pressure = self.getPressure();
                return (44330 * (1 - (pressure / 101325) ** (1 / 5.255)))
        except Exception as e:
            print(f"Error reading altitude: {e}")
            return -666
    
    def test(self):
        if self.broken:
            return -666
        
        if self.sensor.get_sensor_data():
            print("temp:", self.getTemp())
            print("pressure:", self.getPressure())
            print("alt:", self.getAlt())
        else:
            print("error")
            
class GPSSensor:
    def __init__(self):
        self.broken = False
        try:
            gpsd.connect()
        except Exception as e:
            self.broken = True
            print(f"Error connecting to GPSD: {e}")
    
    def getLat(self):
        if self.broken:
            return -666

        try:
            packet = gpsd.get_current()
            return packet.lat
        except Exception as e:
            
            return -666

    def getLong(self):
        if self.broken:
            return -666 
        try:
            packet = gpsd.get_current()
            return packet.lon
        except Exception as e:
          
            return -666

    def getAlt(self):
        if self.broken:
            return -666
        
        try:
            packet = gpsd.get_current()
            return packet.alt
        except Exception as e:
         
            return -666

    def test(self):
        if self.broken:
            return
        print("Latitude:", self.getLat())
        print("Longitude:", self.getLong())
        print("Altitude:", self.getAlt())
        
class SensorDataPack:
    def __init__(self):
        
        self.lat = 0
        self.long = 0
        self.alt = 0
        self.temperature = 0
        self.pressure = 0
        self.altitude = 0
        self.voltage = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.dateAndTime = '1/1/2038-31:52:12'
        self.current = 0
        
        time.sleep(2);
        
class SensorPack:
    
    def __init__(self):
        self.bme = BMESensor()
        self.gyro = MPUSensor()
        self.gps = GPSSensor()
        self.time = TimeSensor()
        self.sensorDataPack = SensorDataPack()
        
        self.bmeAltOffset = 0
        self.gpsAltOffset = 0
        
        self.calcOffset();
         
    def calcOffset(self):
        gpsCalcSum = 0
        bmeCalcSum = 0
        
        for i in range(5):
            bmeCalcSum += self.bme.getAlt();
            gpsCalcSum += self.gps.getAlt();
            time.sleep(0.1)
            
        self.bmeAltOffset = bmeCalcSum / 5 
        self.gpsAltOffset = gpsCalcSum / 5

    def test(self):
        self.bme.test()
        self.gyro.test()
        self.gps.test()
        self.time.test();

    def updateSensorDataPack(self):
        self.sensorDataPack.lat = self.gps.getLat()
        self.sensorDataPack.long = self.gps.getLong()
        self.sensorDataPack.alt = self.gps.getAlt()
        self.sensorDataPack.roll = self.gyro.getRoll()
        self.sensorDataPack.pitch = self.gyro.getPitch()
        self.sensorDataPack.yaw = self.gyro.getYaw()
        self.sensorDataPack.temperature = self.bme.getTemp()
        self.sensorDataPack.pressure = self.bme.getPressure()
        self.sensorDataPack.altitude = self.bme.getAlt() - self.bmeAltOffset
        self.sensorDataPack.dateAndTime = self.time.getDateAndTime();
    
    def printDataPack(self):
        print(f"Latitude: {self.sensorDataPack.lat}")
        print(f"Longitude: {self.sensorDataPack.long}")
        print(f"Altitude (GPS): {self.sensorDataPack.alt}")
        print(f"Temperature: {self.sensorDataPack.temperature}")
        print(f"Pressure: {self.sensorDataPack.pressure}")
        print(f"Altitude (BME): {self.sensorDataPack.altitude}")
        print(f"Voltage: {self.sensorDataPack.voltage}")
        print(f"Roll: {self.sensorDataPack.roll}")
        print(f"Pitch: {self.sensorDataPack.pitch}")
        print(f"Yaw: {self.sensorDataPack.yaw}")
        print(f"Date and Time: {self.sensorDataPack.dateAndTime}")
        
class Servo:
    def __init__(self,servoPin,servoDefaultAngle,servoDetachmentAngle):
        factory = PiGPIOFactory();
        self.servoDetachmentAngle = servoDetachmentAngle
        self.servoDefaultAngle = servoDefaultAngle
        self.servo = AngularServo(servoPin, min_pulse_width=0.0006, max_pulse_width=0.0023, pin_factory=factory);
        self.failedAttemptCounter=0;
    
    def detach(self):
        print("Servo detaching")
        try:
            self.servo.angle = self.servoDetachmentAngle
        except Exception as e:
            pass
        
    def lock(self):
        print("Servo locking")
        try:
            self.servo.angle = self.servoDefaultAngle
        except Exception as e:
            pass


def warnAndExit():

    print("Stop trying to break the program.")
    exit()
    
def validateArguments():
    if not len(sys.argv)==2:
        warnAndExit();

    if sys.argv[1] not in ['360', 'detach', 'range', 'input', 'output', 'all', 'gps', 'bme', 'mpu', 'rtc','zero']:
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
                time.sleep(0.01) 

            time.sleep(1)  

            for angle in range(180, -1, -1):  
                duty_cycle = angleToDutyCycle(angle)
                pwm.ChangeDutyCycle(duty_cycle)
                time.sleep(0.01)  

            time.sleep(1)  

    except KeyboardInterrupt:
        pass

    pwm.stop()
    GPIO.cleanup()

def testDetachment(normalServoPin,defaultAngle,detachmentAngle):
    testServo = Servo(normalServoPin,defaultAngle,detachmentAngle)
    
    testServo.lock()
    time.sleep(2)
    testServo.detach()
    time.sleep(3)
    testServo.lock()
    time.sleep(1)
    

def testHackedServo(hackedServoPin):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(hackedServoPin, GPIO.OUT)

    pwm = GPIO.PWM(hackedServoPin, 50)  
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
        duty_cycle = angleToDutyCycle(130)
        pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(3)
        pwm.stop()

    except KeyboardInterrupt:
        pass

    GPIO.cleanup()    
    
def testAllSensors():
    sensorPack = SensorPack();

    while True:
        time.sleep(1)
        sensorPack.test();

def testBme():
    sensorPack = SensorPack();

    while True:
        sensorPack.bme.test()
        time.sleep(1)
        
def testGps():
    
    sensorPack = SensorPack();

    while True:
        sensorPack.gps.test()           
        time.sleep(1)
        
def testMpu():
    sensorPack = SensorPack();

    while True:
        sensorPack.gyro.test()
        time.sleep(1)
        
def testRtc():
    timeSensor = TimeSensor()

    while True:
        timeSensor.test()
        time.sleep(1)
        
def testToZero(normalServoPin):
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
        duty_cycle = angleToDutyCycle(0)
        pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(1) 

    except KeyboardInterrupt:
        pass    
        
def run():
    
    validateArguments();

    inputPin = 22
    outputPin = 27

    normalServoPin = 13
    detachmentAngle = 85
    defaultAngle = 40

    hackedServoPin = 13

    if sys.argv[1]=='input':
        testInput(inputPin)
    
    if sys.argv[1]=='output':
        testOutput(outputPin)
        
    if sys.argv[1]=='range':
        testRange(normalServoPin)
        
    if sys.argv[1]=='detach':
        testDetachment(normalServoPin,defaultAngle,detachmentAngle)
     
    if str(sys.argv[1])=='360':
        testHackedServo(hackedServoPin)

    if sys.argv[1]=='all':
        testAllSensors()

    if sys.argv[1]=='bme':
        testBme()
    
    if sys.argv[1]=='gps':
        testGps()
        
    if sys.argv[1]=='rtc':
        testRtc()
        
    if sys.argv[1]=='mpu':
        testMpu()
        
    if sys.argv[1]=='zero':
        testToZero(normalServoPin);
      
        
run();