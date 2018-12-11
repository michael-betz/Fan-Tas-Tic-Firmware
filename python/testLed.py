#!/usr/bin/python3
"""
turn on a specific led
usage:
testLed.py /dev/ttyACM0 3200000 0 42
to turn on led 42 on channel 0.
set communication speed to 3200000 (high speed mode)
try 1700000 or 3200000 here
turns off all other leds on that channel
"""
from serial import Serial
from sys import argv, exit

if len(argv) != 5:
    print(__doc__)
    exit()

aTTY = argv[1]
aSPEED = argv[2]
aCHANNEL = argv[3]
aLED = int(argv[4], 0)

with Serial(aTTY, timeout=1) as s:
    s.read_all()           # Clear receive buffer
    s.write(b"\n*IDN?\n")  # First \n clears send buffer
    print(s.read_until())
    configStr = "LEC {} {}\n".format(aSPEED, aCHANNEL)
    s.write(bytearray(configStr, "ascii"))
    z = bytearray(1024 * 3)
    z[aLED * 3: aLED * 3 + 3] = [255] * 3
    sDat = bytes("LED {0} {1}\n".format(aCHANNEL, len(z)), "utf8") + z
    s.write(sDat)
