from numpy import (array, dot, arccos, clip, zeros, linspace)
from numpy.linalg import norm
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
import math
import pickle
import os
from dotenv import load_dotenv

def main():
    load_dotenv()
    id = os.getenv('id')
    diverg = 56
    wT = 290
    wR = 70
    pts = 1000
    ackWait = 0.00842599630355835
    (coverage, n) = calcCoverage(diverg)
    slotTime = calcSlotTime(wR, wT, n)
    (x, y, z) = genCoords(n, pts, id)
    (phi, theta, tranWait, recWait) = genPath(x, y, z, wT, wR, pts)
    path = {
            "phi": phi, 
            "theta": theta, 
            "tranWait": tranWait, 
            "recWait": recWait, 
            "slotTime": slotTime, 
            "ackWait": ackWait, 
            "wT": wT, 
            "wR": wR
    }
    pickle.dump(path, open("path.p", "wb"))

# generate the x, y, z coordinates
def genCoords(n, pts, id):
    x = []
    y = []
    z = []
    lin = []
    if id == '0':
        lin = linspace(-math.pi, 0, num=pts)
    else:
        lin = linspace(0.0000001, math.pi, num=pts)
    for i in range(0,pts-1):
        r = math.cos(lin[i]/2)
        x.append(r*math.sin(n*lin[i]))
        y.append(r*math.cos(n*lin[i]))
        z.append(math.sin(lin[i]/2))
    return (x, y, z)

# calculate the polar coordinates and wait times
def genPath(x, y, z, wT, wR, pts):
    phi = []
    theta = []
    tranWait = []
    recWait = []
    for i in range(0,pts-1):
        radius = (x[i]**2 + y[i]**2 + z[i]**2)
        theta.append(math.degrees(math.acos(z[i] / radius)))
        phi.append(math.degrees(math.atan(y[i] / x[i])) + 90)
        if i > 0:
            u = array([x[i],y[i],0])
            v = ([x[i-1],y[i-1],0])
            c = dot(u,v)/norm(u)/norm(v) # -> cosine of the angle
            angle = math.degrees(math.acos(clip(c, -1, 1))) # if you really want the angle
            #angle = phi[i] - phi[i-1]
            tranWait.append(angle / wT)
            recWait.append(angle / wR)
        else:
            tranWait.append(0)
            recWait.append(0)
    return (phi, theta, tranWait, recWait)

# convert polar to cartesian
def polar2Coord(phi, theta):
    x = []
    y = []
    z = []
    for i in range(0, len(phi)):
        phiRad = math.radians(phi[i])
        thetaRad = math.radians(theta[i])
        x.append(math.sin(thetaRad)*math.cos(phiRad))
        y.append(math.sin(thetaRad)*math.sin(phiRad))
        z.append(math.cos(thetaRad))
    return (x, y, z)

def plot(x, y, z, phi, theta):
    (x1, y1, z1) = polar2Coord(phi, theta)
    fig = plt.figure(figsize=plt.figaspect(0.5))
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    ax.set_title('Ideal Sphere')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.plot3D(x,y,z,'gray')
    ax = fig.add_subplot(1, 2, 2, projection='3d')
    ax.set_title('Half Sphere')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.plot3D(x1,y1,z1,'gray')
    plt.show()

def reduce(num, denom):
    commDenom = math.gcd(num, denom)
    return (num / commDenom, denom / commDenom)

def calcSlotTime(wR, wT, n):
    (p, q) = reduce(wR, wT)
    return (2*1.28*n*180*q) / (wT)

def calcCoverage(diverg):
    coverage = (diverg / 2) * math.sqrt(2)  # coverage width
    n = (180 / coverage)
    return (coverage, n)

if __name__ == "__main__":
    main()
