from pointerheap import *

def printNode(n):
	print("ID: ", GetID(n))
	print("Dependents: ", GetDependents(n))
	print("Step: ", GetStep(n))
	print("Decision: ",  GetDecision(n))
	print("Parent: ", GetParent(n))
	print("Time: ", GetTime(n))
	print("Velocity: ", GetVelocity(n))
	print("Gear: ", GetGear(n))
	print("Delta: ", GetDelta(n))
	print("Time to shift: ", GetTimeToShift(n))
	print("Score: ", GetScore(n))

def nodeToList(n):
	return [GetID(n),
			GetDependents(n),
			GetStep(n),
			GetDecision(n),
			GetParent(n),
			GetTime(n),
			GetVelocity(n),
			(GetGear(n), GetDelta(n), GetTimeToShift(n)),
			GetScore(n)]

def listToNode(l):
	g = list(l[7])
	return MakeNode(l[0], l[1], l[3], l[2], l[4], l[5], l[6], g[0], g[1], g[2])

Init(10)

n = MakeNode(0, 0, 0, None, 0, 0.0, 0.0, 0, 0, 0.0)
m = listToNode(nodeToList(n))

SetDependents(n, 5)
print(nodeToList(n))
print(nodeToList(m))
print(n)

KillNode(n)
KillNode(m)
Destroy()