import math

def checkPrime(x):
    if x >= 2:
        for y in range(2,x):
            if not ( x % y ):
                return False
    else:
        return False
    return True

theta = (48)
# half angle divergence
beta = (24)

convWidth = beta * math.sqrt(2)
n = 180 / convWidth
print(n)
x = []

for p in range(0,100):
  for q in range(0,100):
    if p != q and checkPrime(p) and checkPrime(q) and (theta)*(p+q) > (1.28*n*180):
      x.append([p, q])
print(x)
