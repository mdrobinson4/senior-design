import numpy as np
import math

def simplify(num, denom):
    commDenom = math.gcd(num, denom)
    return (num / commDenom, denom / commDenom)

pointCount = 1000
x = [0 for i in range(pointCount)]
y = [0 for i in range(pointCount)]
z = [0 for i in range(pointCount)]
theta = [0 for i in range(pointCount)]
phi = [0 for i in range(pointCount)]

# half angle from the axis of propagation for transmissions.
# The angle of field-of-view is 2 * beta
fullAngleDiv = 56
# width of convergence (y)
convWidth = (fullAngleDiv / 2) * math.sqrt(2)
# number of rotations necessary to scan 3d area
n = 180 / convWidth
# transmission angular velocity [ degrees / second ]
wT = 130
# reception angular velocity [ degrees / second ]
wR = 110
# Receiver (p) rounds and transmission (q) rounds
(p, q) = simplify(wR, wT)

def createPath():
    lin = np.linspace(-math.pi, math.pi, num=pointCount)
    p = 0
    for i in range(1, pointCount):
        p = math.cos(lin[i]/2)
        x[i] = p*math.sin(n*lin[i])
        y[i] = p*math.cos(n*lin[i])
        z[i] = math.sin(lin[i]/2)
        # calculate the new x, y, and z values
        #x[i] = math.cos(s[i] / 2) * math.sin(s[i] * n)
        #y[i] = math.cos(s[i] / 2) * math.cos(s[i] * n)
        #z[i] = math.sin(s[i] / 2)

        # calculate the radius of the sphere
        r = (x[i]**2 + y[i]**2 + z[i]**2)**(1/2)
        # calculate theta the z-axis angle [ in degrees ]
        theta[i] = math.degrees(math.acos(z[i] / r))
        # calculate phi the x-axis angle [ in degrees ]
        phi[i] = math.degrees(math.atan(y[i] / x[i]))
        
        
        # needed since we can only rotate 180 degrees
        #phi[i] = translate(phi[i],-90,90,0,180)
        
        if x[i] < 0 and y[i] < 0:
            phi[i] = 180 - phi[i]
        elif x[i] > 0 and y[i] < 0:
            phi[i] *= -1
        elif x[i] < 0 and y[i] > 0:
            phi[i] += 180.0
        

        # create an array of the previous coordinates
        prev = np.array([x[i - 1] or 0, y[i - 1] or 0, z[i - 1] or 0])
        # create an array of the current coordinates
        curr = np.array([x[i], y[i], z[i]])
        # calculate the change in the angle
        if np.linalg.norm(prev) == 0 or np.linalg.norm(curr) == 0:
            angleChange = 0
        else:
            angleChange = math.degrees(((math.acos(np.dot(prev, curr) / (np.linalg.norm(prev) * np.linalg.norm(curr))))))
        # calculate the amount of time for the servo to rest
        # so we maintain constant transmission and receiving mode speeds
        # tranStep[i-1] = angleChange / wT
        # recStep[i-1] = angleChange / wR
