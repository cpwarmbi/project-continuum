import matlab.engine
import time

import matlab.engine
eng = matlab.engine.start_matlab()
eng.BuildMap(nargout=0)
while True:
    print("Running")
    time.sleep(0.5)