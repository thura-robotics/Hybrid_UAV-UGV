#read servo position 
from st3215 import ST3215
import time

s = ST3215('/dev/ttyUSB0')

#read al servo position
for i in range(1,10):
    print('servo',i,':',s.ReadPosition(i))
    time.sleep(0.1)