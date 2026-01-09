import numpy as np



class MsgDelta:

    def __init__(self):
        
        #aileron control (-1.0,1.0)
        self.delta_a = 0.0
        #elevator control (-1.0,1.0)
        self.delta_e = 0.0
        #rudder control (-1.0,1.0)
        self.delta_r = 0.0
        #throttle control (0.0, 1.0)
        self.delta_t = 0.0
