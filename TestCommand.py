import time
from UDPComms import Publisher

LEDcmd = Publisher(8830)

for i in range(3):
    LEDcmd.send({"r": 1.0, "g": 0, "b": 0})
    time.sleep(2)
    LEDcmd.send({"r": 0, "g": 1.0, "b": 0})
    time.sleep(2)
    LEDcmd.send({"r": 0, "g": 0, "b": 1.0})
    time.sleep(2)
