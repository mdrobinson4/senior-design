import math

def checkPrime(num):
    prime = True
    for i in range(2, num):
        if (num % i) == 0:
            prime = False
    if num <= 1:
        prime = False
    return prime


def getPrimes(n):
    prime = []
    for i in range(100):
        if checkPrime(i) == True:
            prime.append(i)
    return prime

primes = getPrimes(300)
vals = []
for p in primes:
    for q in primes:
        if ((56 * (p + q)) > (1.28 * 4.54569 * 180)) and (p != q):
            vals.append([p,q])
print()
print(vals)
