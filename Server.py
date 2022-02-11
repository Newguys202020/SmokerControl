from flask import Flask
from flask import request
from flask import render_template
import busio
import digitalio
import board
import RPi.GPIO as GPIO
import time

def is_number(n):
    is_number = 1
    try:
        num = float(n)
        is_number = num == num
    except ValueError:
        is_number = 0
    return is_number

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

def get_Setpoint():
    f = open("/home/pi/Desktop/Temperature.txt", "r")
    setpoint = f.read()
    f.close()
    return setpoint

app = Flask(__name__)

iLast = get_iSet()
while isinstance(iLast, float) == False:
    try:
        iLast = float(iLast)
    except:
        iLast = get_iSet()
    else:
        iLast = float(iLast)
pLast = get_pSet()
while isinstance(pLast, float) == False:
    try:
        pLast = float(pLast)
    except:
        pLast = get_pSet()
    else:
        pLast = float(pLast)
setTemp = get_Setpoint()
while isinstance(setTemp, float) == False:
    try:
        setTemp = float(setTemp)
    except:
        setTemp = get_Setpoint()
    else:
        setTemp = float(setTemp)
print ("pset: %f" % (pLast))
print ("iset: %f" % (iLast))
print ("tset: %f" % (setTemp))

f = open("/home/pi/Desktop/Status.txt", "w")
f.write("0")
f.close()

if __name__ == "__main__":
  @app.route("/")
  def web_interface():
    iLast = get_iSet()
    while isinstance(iLast, float) == False:
        try:
            iLast = float(iLast)
        except:
            iLast = get_iSet()
        else:
            iLast = float(iLast)
    pLast = get_pSet()
    while isinstance(pLast, float) == False:
        try:
            pLast = float(pLast)
        except:
            pLast = get_pSet()
        else:
            pLast = float(pLast)
    setTemp = get_Setpoint()
    while isinstance(setTemp, float) == False:
        try:
            setTemp = float(setTemp)
        except:
            setTemp = get_Setpoint()
        else:
            setTemp = float(setTemp)
    return render_template('web_interface.html', iLast = iLast, pLast = pLast, tLast = setTemp)

  @app.route("/set_temp")
  def set_temp():
    setTemp = request.args.get("temp")
    f = open("/home/pi/Desktop/Temperature.txt", "w")
    if is_number(setTemp):
        setTemp = float(setTemp)
        if setTemp > 49.9 and setTemp < 500.1:
            f.write("%s" % (setTemp))
            f.close()
            print ("received %s" % (setTemp))
        else:
            print ("Input Denied")
    else:
        print ("Input Denied")
    return "Received"

  @app.route("/enable_pid")
  def enable_pid():
    command = request.args.get("command")
    print ("recieved %s" % (command))
    f = open("/home/pi/Desktop/Status.txt", "w")
    f.write("%s" % (command))
    f.close()
    return "Received"

  @app.route("/set_p")
  def set_p():
    pSet = request.args.get("p")
    f = open("/home/pi/Desktop/pSet.txt", "w")
    if is_number(pSet):
        pSet = float(pSet)
        if pSet > -0.1 and pSet < 200.1:
            f.write("%s" % (pSet))
            f.close()
            print ("received %s" % (pSet))
        else:
            print ("Input Denied")
    else:
        print ("Input Denied")
    return "Received"

  @app.route("/set_i")
  def set_i():
    iSet = request.args.get("i")
    f = open("/home/pi/Desktop/iSet.txt", "w")
    if is_number(iSet):
        iSet = float(iSet)
        if iSet > -0.1 and iSet < 200.1:
            f.write("%s" % (iSet))
            f.close()
            print ("received %s" % (iSet))
        else:
            print ("Input Denied")
    else:
        print ("Input Denied")
    return "Received"

  app.run(host= '0.0.0.0')
