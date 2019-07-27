import math

n = 1.32582527954
x = []
div = math.pi / 3.75
for p in range(0,10):
  for q in range(0,10):
    if math.gcd(p, q) == 1 and (p*div) + (q*div) > (1.28*n):
      x.append([p, q])
print(x)
