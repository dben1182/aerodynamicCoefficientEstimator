import numpy as np



class MsgState:

    def __init__(self):
        self.north = float(0.) 
        self.east = float(0.)      
        self.altitude = float(0.)       
        self.phi = float(0.)     
        self.theta = float(0.)
        self.psi = float(0.)
        self.Va = float(25.)
        self.alpha = float(0.)
        self.beta = float(0.)
        self.p = float(0.)
        self.q = float(0.)
        self.r = float(0.)
        self.Vg = float(25.)
        self.gamma = float(0.)
        self.chi = float(0.)
