# this is the file containing the parameters needed for the estimator to work
# We are trying to estimate the parameters of aerodynamics, but we assume
# that we know the other parameters, such as rotational inertia, and


# sets the number of parameters for each of the five filters
numCoefficients_Y = 6
numCoefficients_Lift = 4
numCoefficients_Drag = 4
numCoefficients_ell = 6
numCoefficients_m = 4
numCoefficients_n = 6

# sets the size of the y vector for each estimator
y_length_Y = 1
y_length_LD = 2
y_length_ell = 1
y_length_m = 1
y_length_n = 1



#creates the Rotational inertia values
Jx = 0.8244 #kg m^2
Jy = 1.135
Jz = 1.759
Jxz = 0.1204

#other parameters of the vehicle that are known and
#are necessary for the calculations herein
mass = 11.0 #kg
S_wing = 0.55 #surface are of the wing
b = 2.8956 #wingspan
c = 0.18994 #mean chord of the MAV Wing
S_prop = 0.2027 #surface area of the propellor
rho = 1.2682 #air density
e = 0.9
AR = (b**2) / S_wing
gravity = 9.81
