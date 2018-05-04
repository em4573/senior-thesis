import math

N = 302
P = 3
K = P

res = [[0 if i == 0 else -1 for i in range(K + 1)] for j in range(N + 1)]

res[0] = [0 for i in range(K + 1)]
res[1][1] = 1
res[2][1] = 1

def get(n, k):
	if k > n:
		return 0
	elif n < 0 or k < 0:
		return 0
	else:
		if res[n][k] >= 0:
			return res[n][k]
		else:
			res[n][k] = partition(n, k)
			return res[n][k]

def partition(n, k):
	a = get(n - 1, k - 1)
	b = get(n - k, k)

	return a + b

s = 0
for i in range(P):
	p = i + 1

	q = math.factorial(P) / math.factorial(P - i)
	s += partition(N, p) * q

print s
print partition(N, P)
print N**(P-1)*(N-1)
