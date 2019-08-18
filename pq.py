import math

theta = math.radians(48)
# half angle divergence
beta = math.radians(24)

convWidth = beta * math.sqrt(2)
n = math.pi / convWidth
print(n)
x = []

for p in range(0,10):
  for q in range(0,10):
    print('{} [?] {}'.format(theta*(p+q), 1.28*n*math.pi))
    if math.gcd(p, q) == 1 and (theta)*(p+q) > (1.28*n*math.pi):
      x.append([p, q])
print(x)
