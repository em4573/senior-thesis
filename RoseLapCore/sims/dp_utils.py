import numpy as np
from itertools import permutations
import time

def f7(seq):
    seen = set()
    seen_add = seen.add
    return [x for x in seq if not (x in seen or seen_add(x))]

class nd_structure:
	def __init__(self, n, p):
		self.n = n
		self.p = p
		self.np = [self.n ** i for i in range(self.p)]

		self.maxi = n**(p - 1) * (n - 1)
		self.edges = None
		self.iS = None

		self.arr = np.empty([self.maxi + 1, 1], dtype=object)

	def get_element(self, i):
		return self.arr[i, 0]

	def set_element(self, i, item):
		self.arr[i, 0] = item

	def get_parents(self, iS):
		parents = []

		for i in range(self.p):
			if iS[i] > 0:
				parent = list(iS)
				parent[i] = iS[i] - 1
				parents.append((i, self.convert_to_index(parent)))

		return parents

	def generate_iS(self):
		print "time start"
		t0 = time.time()
		if self.iS != None:
			return self.iS

		iS = []
		added = self.get_edges()

		colp = self.p
		coln = self.n
		np = self.np
		def col(i):
			r = []
			for x in range(colp):
				r.append(i % coln)
				i = (i - r[-1]) / coln
			return r

		def coi(l):
			i = 0
			for x in range(colp):
				i += np[x] * l[x]
			return i

		def gp(l):
			parents = []
			for i in range(colp):
				if l[i] > 0:
					parent = list(l)
					parent[i] = l[i] - 1
					parents.append((i, coi(parent)))
			return parents

		print time.time() - t0

		while(len(added) > 0):
			new = set()

			for i in added:
				iS.append(i)

				parents = gp(col(i))
				for p in parents:
					new.add(p[1])

			added = new

		print time.time() - t0

		self.iS = iS[::-1]

		print time.time() - t0

		return self.iS

	def get_edges(self):
		if self.edges != None:
			return self.edges

		print "\ttime start"
		t0 = time.time()

		def get_edges_recursive(n, p):
			if p == 1:
				return [tuple([n])]
			else:
				result = []

				for i in range(n):
					tails = get_edges_recursive(n - i, p - 1)

					for tail in tails:
						result.append(tail + tuple([i]))

				return result

		iE = get_edges_recursive(self.n - 1, self.p)
		print "\t", time.time() - t0
		perms = [list(permutations(e)) for e in iE]
		print "\t", time.time() - t0
		edges = []
		
		for perm in perms:
			edges.extend(perm)

		print "\t", time.time() - t0

		p = self.p
		np = self.np
		def coi(iS):
			i = 0
			for x in range(p):
				i += np[x] * iS[x]
			return i

		final = []
		for edge in edges:
			final.append(coi(edge))

		print "\t", time.time() - t0

		self.edges = set(final)

		return self.edges

	def convert_to_index(self, iS):
		i = 0

		for x in range(self.p):
			i += self.np[x] * iS[x]

		return i

	def convert_to_list(self, i):
		iS = []

		for x in range(self.p):
			iS.append(i % self.n)
			i = (i - iS[-1]) / self.n

		return iS