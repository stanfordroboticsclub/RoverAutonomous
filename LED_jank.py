import math
import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

from UDPComms import Subscriber, timeout

cmd = Subscriber(8310, timeout = 5)
status = Subscriber(8311, timeout = 5)

time.sleep(2)

greenPin = 20
redPin = 21

GPIO.setup(redPin, GPIO.OUT)
GPIO.setup(greenPin, GPIO.OUT)

GPIO.output(redPin, 0)
GPIO.output(greenPin, 0)


flash_time = time.monotonic()
green_led = False

while 1:
    try:
        msg = cmd.get()
        if msg['command'] == 'auto':
            try:
                status_msg = status.get()
            except timeout:
                status_msg = "working"
            if status_msg == "done":
                GPIO.output(redPin, 0)
                if (time.monotonic() - flash_time) > 0.5:
                    green_led = not green_led
                    GPIO.output(greenPin, green_led)
                    flash_time = time.monotonic()

            else:
                GPIO.output(redPin, 1)
                GPIO.output(greenPin, 0)
        else:
            green_led = 0
            GPIO.output(redPin, 0)
            GPIO.output(greenPin, 0)
    except:
        GPIO.output(redPin, 0)
        GPIO.output(greenPin, 0)
        GPIO.cleanup()
        raise


