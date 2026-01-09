import numpy as np
import os, sys

from pathlib import Path
from msg_sensors import MsgSensors
from recursiveLMSFilter import recursiveLMSFilter
from msg_state import MsgState
from msg_delta import MsgDelta
import parameters as PARAM


# creates the class for the generalized aerodynamic estimator


# creates the aerodynamic estimator for all of the sections
class total3DAerodynamicsEstimator:
    def __init__(self):
        # creates the Y estimators
        self.Y_estimator = recursiveLMSFilter(
            x_length=PARAM.numCoefficients_Y,
            y_width=PARAM.y_length_Y)

        self.LD_estimator = recursiveLMSFilter(x_length=PARAM.numCoefficients_Lift,
                                               y_width=PARAM.y_length_LD)

        self.ell_estimator = recursiveLMSFilter(x_length=PARAM.numCoefficients_ell,
                                                            y_width=PARAM.y_length_ell)

        self.m_estimator = recursiveLMSFilter(x_length=PARAM.numCoefficients_m,
                                                          y_width=PARAM.y_length_m)
        
        self.n_estimator = recursiveLMSFilter(x_length=PARAM.numCoefficients_n,
                                                          y_width=PARAM.y_length_n)

    def update(self,
               state: MsgState,
               sensorMeasurements: MsgSensors,
               delta: MsgDelta,
               thrust: float):
         
        #gets the a_n and y_n for the y estimator
        a_n_Y, y_n_Y = self.y_matrices(state=state,
                                       sensorMeasurements=sensorMeasurements,
                                       delta=delta)

        self.Y_estimator.update(a_n=a_n_Y, y_n=y_n_Y)

        a_n_LD, y_n_LD = self.LD_matrices(state=state,
                                          sensorMeasurements=sensorMeasurements,
                                          delta=delta,
                                          thrust=thrust)        

        self.LD_estimator.update(a_n=a_n_LD, y_n=y_n_LD)


    



    #section for all functions that create the a_n and y_n matrices for each filter

    def y_matrices(self,
                   state: MsgState,
                   sensorMeasurements: MsgSensors,
                   delta: MsgDelta):
        #creates the scaling factor, which is just used for the a_n
        scalingFactor = ((1.0)/(2.0*PARAM.mass))*PARAM.rho*(state.Va**2)*PARAM.S_wing
        #creates the mixing matrix
        mixingMatrix = np.array([[1.0, state.beta, ((PARAM.b*state.p)/(2*state.Va)), ((PARAM.b*state.r)/(2*state.Va)), delta.delta_a, delta.delta_r]])
        
        #creates the a_n vector
        a_n = np.transpose(scalingFactor*mixingMatrix)
        
        #creates the y_n vector
        y_n = np.array([[sensorMeasurements.accel_y]])
        
        return a_n, y_n

    #for Lift Drag coefficients
    def LD_matrices(self,
                    state: MsgState,
                    sensorMeasurements: MsgSensors,
                    delta: MsgDelta,
                    thrust: float):

        scalingFactor = ((1.0)/(2.0*PARAM.mass))*PARAM.rho*(state.Va**2)*PARAM.S_wing

        alphaRotation = np.array([[np.sin(state.alpha), -np.cos(state.alpha)],
                                  [-np.cos(state.alpha), -np.sin(state.alpha)]])

        primaryMixing = np.array([[1.0, state.alpha, (PARAM.c*state.q)/(2.0*state.Va), delta.delta_e, 0.0, 0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0, 0.0, 1.0, state.alpha, (PARAM.c*state.q)/(2.0*state.Va), delta.delta_e]])
        #gets the rotated mixing matrix
        mixingMatrix = alphaRotation @ primaryMixing
        #gets the a_n
        a_n = np.transpose(scalingFactor*mixingMatrix)

        #gets the y_n
        y_n = np.array([[sensorMeasurements.accel_x],[sensorMeasurements.accel_z]]) - np.array([[thrust/PARAM.mass],[0.0]])

        return a_n, y_n

