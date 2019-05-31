import math
import time
import RPi.GPIO as GPIO
from UDPComms import Subscriber, timeout

cmd = Subscriber(8590, timeout = 5)

greenPin = 13
redPin = 12
bluePin = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(redPin, GPIO.OUT)
GPIO.setup(greenPin, GPIO.OUT)
GPIO.setup(bluePin, GPIO.OUT)

pwmRed = GPIO.PWM(redPin, 490)
pwmGreen = GPIO.PWM(greenPin, 490)
pwmBlue = GPIO.PWM(bluePin, 490)

pwmRed.start(0)
pwmGreen.start(0)
pwmBlue.start(0)

maxPower = 50

r = 0
g = 0
b = 1.0

def gamma(x, gamma):
    return math.pow(x, gamma)
 
def rounder(value):
    maxVal = math.pow(1.0, 2.2)
    newMax = 100
    scale = float(value/maxVal)
    newVal = round(scale*newMax)
    if newVal > 50:
        newVal = 50
    return newVal
    
def turnColor(red, green, blue, gammaVal = 2.2):
    r = rounder(gamma(red, gammaVal))
    g = rounder(gamma(green, gammaVal))
    b = rounder(gamma(blue, gammaVal))

    sum = r+g+b

    if sum > maxPower:
	scale = float(sum)/maxPower
	r = r/scale
	g = g/scale
	b = b/scale

    pwmRed.ChangeDutyCycle(r)
    pwmGreen.ChangeDutyCycle(g)
    pwmBlue.ChangeDutyCycle(b)


def processMsg():
    try:
        msg = cmd.get()
        print(msg)

        r = msg['r']
        g = msg['g']
        b = msg['b']
        turnColor(r, g, b)
            
            
        
    except timeout:
        print("displaying last command")

    except:
        print("turning off")
        pwmBlue.stop()
        pwmRed.stop()
        pwmGreen.stop()
        GPIO.cleanup()
        raise
    
def main():
    while True:
        processMsg()
        
if __name__ == "__main__":
    main()
