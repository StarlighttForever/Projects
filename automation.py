import random
import time
from pymouse import PyMouse
from pykeyboard import PyKeyboard

ms = PyMouse()
k = PyKeyboard()
rtime = (1/int(random.randrange(1,3)))

for i in range(1,6):
	k.type_string(str("56514911301363"))
	k.tap_key("\r")
	time.sleep(rtime)
