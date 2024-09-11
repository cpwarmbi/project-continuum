import matlab.engine
import time

import matlab.engine
eng = matlab.engine.start_matlab()
eng.BuildMap(nargout=0)
print("Running")
while True:
    time.sleep(0.1)