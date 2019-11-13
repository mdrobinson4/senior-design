import numpy as np
import math
pointCount = 1000
x = [0 for i in range(pointCount)]
y = [0 for i in range(pointCount)]
z = [0 for i in range(pointCount)]
theta = [0 for i in range(pointCount)]
phi = [0 for i in range(pointCount)]
n = 4.545686450484949

def createPath():
        lin = np.linspace(-math.pi, math.pi, num=pointCount)
        p = 0

        lastBPoint = -1

        for i in range(1, pointCount):
            p = math.cos(lin[i]/2)
            x[i] = p*math.sin(n*lin[i])
            y[i] = p*math.cos(n*lin[i])
            z[i] = math.sin(lin[i]/2)
            # print(x[i],y[i],z[i])
            # if (y[i] < 0):
            #     if lastBPoint == -1:
            #         lastBPoint = i
            # else:
            #     if lastBPoint != -1:
            #         for j in range(lastBPoint, i):
            #             x[j] = x[i]
            #             y[j] = y[i]
            #             z[j] = z[i]
            #         lastBPoint = -1 



        for i in range(1, pointCount):
            # calculate the radius of the sphere
            # r = (x[i]**2 + y[i]**2 + z[i]**2)**(1/2)
            # calculate theta the z-axis angle [ in degrees ]
            theta[i] = math.degrees(math.atan2(y[i], z[i]))
            
            # calculate phi the x-axis angle [ in degrees ]
            phi[i] = math.degrees(math.atan(z[i]/ x[i])) + 90

            # print(theta[i], phi[i])


            # needed since we can only rotate 180 degrees
            # if x[i] < 0 and y[i] < 0:
            #    phi[i] = 180 - phi[i]
            #elif x[i] > 0 and y[i] < 0:
            #   phi[i] *= -1
            #elif x[i] < 0 and y[i] > 0:
            #   phi[i] += 180.0
            #else:
            #   status[i] = 1  # facing the front, not the back

            # create an array of the previous coordinates
        #     prev = np.array([x[i - 1] or 0, y[i - 1] or 0, z[i - 1] or 0])
        #     # create an array of the current coordinates
        #     curr = np.array([x[i], y[i], z[i]])
        #     # calculate the change in the angle
        #     if np.linalg.norm(prev) == 0 or np.linalg.norm(curr) == 0:
        #         angleChange = 0
        #     else:
        #         angleChange = math.degrees(((math.acos(np.dot(prev, curr) / (np.linalg.norm(prev) * np.linalg.norm(curr))))))
        #     # calculate the amount of time for the servo to rest
        #     # so we maintain constant transmission and receiving mode speeds
        #     tranStep[i-1] = angleChange / wT
        #     recStep[i-1] = angleChange / wR
        # # reverse the theta/phi angles and transmission/reception sleep times
        # phi = np.append(phi, phi[::-1])
        # theta = np.append(theta, theta[::-1])
        # tranStep = np.append(tranStep, tranStep[::-1])
        # recStep = np.append(recStep, recStep[::-1])
        # status = np.append(status, status[::-1])