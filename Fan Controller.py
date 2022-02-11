import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import atexit
from simple_pid import PID
import time
import math
import RPi.GPIO as GPIO
from influxdb import InfluxDBClient


# Code for MC3008

spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.D8)
mcp = MCP.MCP3008(spi, cs)

client = InfluxDBClient(host='localhost', port=8086)
client.create_database('temperatures')
client.switch_database('temperatures')

Ro = 100000
To = 25.0
beta = 4190

global lasttempf0 = 20.0
global lasttempf1 = 20.0
global lasttempf2 = 20.0
global lasttempf3 = 20.0
global lasttempf4 = 20.0
global lasttempf5 = 20.0
global lasttempf6 = 20.0
global lasttempf7 = 20.0
# Thermistor reading function
def temp_get():
    global lasttempf0
    global lasttempf1
    global lasttempf2
    global lasttempf3
    global lasttempf4
    global lasttempf5
    global lasttempf6
    global lasttempf7
    tempf0 = lasttempf0
    tempf1 = lasttempf1
    tempf2 = lasttempf2
    tempf3 = lasttempf3
    tempf4 = lasttempf4
    tempf5 = lasttempf5
    tempf6 = lasttempf6
    tempf7 = lasttempf7
    float(tempf0)
    float(tempf1)
    float(tempf2)
    float(tempf3)
    float(tempf4)
    float(tempf5)
    float(tempf6)
    float(tempf7)
    print ("-----------------------------------")
    #Probe 0
    value0 = AnalogIn(mcp, MCP.P0).value #read the adc
    print ("Read: %f on pin 0" % value0)
    if value0 != 0 and value0 != 65535:
        ohms = 9950  / (65535/value0 -1) #calculate the ohms of the thermistor
        steinhart = math.log(ohms / Ro) / beta
        steinhart += 1.0 / (To + 273.15)
        tempc = (1.0 / steinhart) - 273.15
        tempf0 = tempc*(9/5) +32

    #Probe 1
    value1 = AnalogIn(mcp, MCP.P1).value #read the adc
    print ("Read %f on pin 1" % value1)
    if value1 != 0 and value1 != 65535:
        ohms = 9960 / (65535/value1 -1) #calculate the ohms of the thermistor
        steinhart = math.log(ohms / Ro) / beta
        steinhart += 1.0 / (To + 273.15)
        tempc = (1.0 / steinhart) - 273.15
        tempf1 = tempc*(9/5) +32


    #Probe 2
    value2 = AnalogIn(mcp, MCP.P2).value #read the adc
    print ("Read %f on pin 2" % value2)
    if value2 != 0 and value2 != 65535:
        ohms = 10000 / (65535/value2 -1) #calculate the ohms of the thermistor
        steinhart = math.log(ohms / Ro) / beta
        steinhart += 1.0 / (To + 273.15)
        tempc = (1.0 / steinhart) - 273.15
        tempf2 = tempc*(9/5) +32

    #Probe 3
    value3 = AnalogIn(mcp, MCP.P3).value #read the adc
    print ("Read %f on pin 3" % value3)
    if value3 != 0 and value3 != 65535:
        ohms = 9990 / (65535/value3 -1) #calculate the ohms of the thermistor
        steinhart = math.log(ohms / Ro) / beta
        steinhart += 1.0 / (To + 273.15)
        tempc = (1.0 / steinhart) - 273.15
        tempf3 = tempc*(9/5) +32

    #Probe 4
    value4 = AnalogIn(mcp, MCP.P4).value #read the adc
    print ("Read %f on pin 4" % value4)
    if value4 != 0 and value4 != 65535:
        ohms = 9970 / (65535/value4 -1) #calculate the ohms of the thermistor
        steinhart = math.log(ohms / Ro) / beta
        steinhart += 1.0 / (To + 273.15)
        tempc = (1.0 / steinhart) - 273.15
        tempf4 = tempc*(9/5) +32

    #Probe 5
    value5 = AnalogIn(mcp, MCP.P5).value #read the adc
    print ("Read %f on pin 5" % value5)
    if value5 != 0 and value5 != 65535:
        ohms = 9950 / (65535/value5 -1) #calculate the ohms of the thermistor
        steinhart = math.log(ohms / Ro) / beta
        steinhart += 1.0 / (To + 273.15)
        tempc = (1.0 / steinhart) - 273.15
        tempf5 = tempc*(9/5) +32

    #Probe 6
    value6 = AnalogIn(mcp, MCP.P6).value #read the adc
    print ("Read %f on pin 6" % value6)
    if value6 != 0 and value6 != 65535:
        ohms = 9970 / (65535/value6 -1) #calculate the ohms of the thermistor
        steinhart = math.log(ohms / Ro) / beta
        steinhart += 1.0 / (To + 273.15)
        tempc = (1.0 / steinhart) - 273.15
        tempf6 = tempc*(9/5) +32

    #Probe 7
    value7 = AnalogIn(mcp, MCP.P7).value #read the adc
    print ("Read %f on pin 7" % value7)
    if value7 != 0 and value7 != 65535:
        ohms = 10020 / (65535/value7 -1) #calculate the ohms of the thermistor
        steinhart = math.log(ohms / Ro) / beta
        steinhart += 1.0 / (To + 273.15)
        tempc = (1.0 / steinhart) - 273.15
        tempf7 = tempc*(9/5) +32

    if tempf0 > 0 and tempf0 < 500 and abs(tempf0 - lasttempf0) < 100: #check if temp is reasonable and hasn't made unreasonable jump in a tenth of a second x8
        temp_readings0 = [
            {
                "measurement": "Probe 0",
                "fields": {
                    "value": tempf0,
                }
            }
        ]
        client.write_points(temp_readings0) #send reading if valid

    if tempf1 > 0 and tempf1 < 500 and abs(tempf1 - lasttempf1) < 100:
        temp_readings1 = [
            {
                "measurement": "Probe 1",
                "fields": {
                    "value": tempf1,
                }
            }
        ]
        client.write_points(temp_readings1)

    if tempf2 > 0 and tempf2 < 500 and abs(tempf2 - lasttempf2) < 100:
        temp_readings2 = [
            {
                "measurement": "Probe 2",
                "fields": {
                    "value": tempf2,
                }
            }
        ]
        client.write_points(temp_readings2)

    if tempf3 > 0 and tempf3 < 500 and abs(tempf3 - lasttempf3) < 100:
        temp_readings3 = [
            {
                "measurement": "Probe 3",
                "fields": {
                    "value": tempf3,
                }
            }
        ]
        client.write_points(temp_readings3)

    if tempf4 > 0 and tempf4 < 500 and abs(tempf4 - lasttempf4) < 100:
        temp_readings4 = [
            {
                "measurement": "Probe 4",
                "fields": {
                    "value": tempf4,
                }
            }
        ]
        client.write_points(temp_readings4)

    if tempf5 > 0 and tempf5 < 500 and abs(tempf5 - lasttempf5) < 100:
        temp_readings5 = [
            {
                "measurement": "Probe 5",
                "fields": {
                    "value": tempf5,
                }
            }
        ]
        client.write_points(temp_readings5)

    if tempf6 > 0 and tempf6 < 500 and abs(tempf6 - lasttempf6) < 100:
        temp_readings6 = [
            {
                "measurement": "Probe 6",
                "fields": {
                    "value": tempf6,
                }
            }
        ]
        client.write_points(temp_readings6)

    if tempf7 > 0 and tempf7 < 500 and abs(tempf7 - lasttempf7) < 100:
        temp_readings7 = [
            {
                "measurement": "Probe 7",
                "fields": {
                    "value": tempf7,
                }
            }
        ]
        client.write_points(temp_readings7)

    lasttempf0 = tempf0 #update lasttemp
    lasttempf1 = tempf1
    lasttempf2 = tempf2
    lasttempf3 = tempf3
    lasttempf4 = tempf4
    lasttempf5 = tempf5
    lasttempf6 = tempf6
    lasttempf7 = tempf7

    return tempf0

def get_Setpoint(): #read settings from webpage
    f = open("/home/pi/Desktop/Temperature.txt", "r")
    setpoint = f.read()
    f.close()
    return setpoint

def get_Status():
    f = open("/home/pi/Desktop/Status.txt", "r")
    status = f.read()
    f.close()
    return status

def get_pSet():
    f = open("/home/pi/Desktop/pSet.txt", "r")
    pSet = f.read()
    f.close()
    return pSet

def get_iSet():
    f = open("/home/pi/Desktop/iSet.txt", "r")
    iSet = f.read()
    f.close()
    return iSet

# Set up fan output on pin 14
GPIO.setmode(GPIO.BCM)
GPIO.setup(14,GPIO.OUT)

#chanList = [0, 5, 6, 13, 19, 20, 21, 26]
#GPIO.setup(chanList, GPIO.OUT)
#GPIO.output(chanList, GPIO.HIGH)

pwmOut = GPIO.PWM(14, 200)

pwmOut.start(100)
dutyCycle = 0

control = 0

pSet = get_pSet()
while isinstance(pSet, float) == False:
    try:
        pSet = float(pSet)
    except:
        print("Error, refloating")
        pSet = get_pSet()
    else:
        pSet = float(pSet)
iSet = get_iSet()
while isinstance(iSet, float) == False:
    try:
        iSet = float(iSet)
    except:
        print("Error, refloating")
        iSet = get_iSet()
    else:
        iSet = float(iSet)

pid = PID(pSet, iSet, 1, setpoint=get_Setpoint())

# Initial temp reading in F
currentTemp = temp_get()

# Update time in seconds
pid.sample_time = 2

# Use Proportional on measurement instead of traditional PID
pid.proportional_on_measurement = True

# Set output limit
pid.output_limits = (0, 100)

currentTime = time.time()
lastTime = currentTime
Restart=0

while 1 == 1:
    Running = get_Status()
    currentTime = time.time()
    #print ("last time = %f" % (lastTime))
    #print ("current time = %f" % (currentTime))
    if (currentTime - lastTime) > 1:
        lastTime = currentTime
        temp_get()
        print ("New Temps")
        pSet = get_pSet()
        while isinstance(pSet, float) == False:
            try:
                pSet = float(pSet)
            except:
                print("Error, refloating")
                pSet = get_pSet()
            else:
                pSet = float(pSet)
        iSet = get_iSet()
        while isinstance(iSet, float) == False:
            try:
                iSet = float(iSet)
            except:
                print("Error, refloating")
                iSet = get_iSet()
            else:
                iSet = float(iSet)
        pid.tunings = (pSet, iSet, 0)
    if Running == '1':
            print("Controller Started")
            if Restart == 1:
                pid.auto_mode = True
            while Running == '1':
                currentTime = time.time()
                if (currentTime - lastTime) > 1:
                    lastTime = currentTime
                    Running = get_Status()
                    newSetpoint = get_Setpoint()
                    while isinstance(newSetpoint, float) == False:
                        try:
                            newSetpoint = float(newSetpoint)
                        except:
                            print("Error, refloating")
                            newSetpoint = get_Setpoint()
                        else:
                            newSetpoint = float(newSetpoint)
                    pid.setpoint = newSetpoint
                    print ("Temperature setpoint is %f" % (pid.setpoint))
                    print ("Duty cycle is %f" % (dutyCycle))
                    print ("control is %f" % (control))
                    control = pid(currentTemp)
                    print ("new control is %f" % (control))
                    dutyCycle = control
                    print ("New duty cycle is %f" % (dutyCycle))
                    print ("current temp is %f" % (currentTemp))
                    pwmOut.ChangeDutyCycle(dutyCycle)
                    lastTime = currentTime
                    currentTime = time.time()
                    currentTemp = temp_get()
                    print ("new temp is %f" % (currentTemp))
                    print ("---------------")

                    if Running == '0':
                        Restart = 1
                        pid.auto_mode = False
                        pwmOut.ChangeDutyCycle(0)
                        print("Controller Stopped")
