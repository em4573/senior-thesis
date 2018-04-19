import numpy as np
import dp_utils as dpu
np.set_printoptions(threshold=np.inf, linewidth=np.inf, precision=4)
import math

class State:
	def __init__(self, exists=True, t=0., v=0., l=-1, k=-1, i=0):
		self.decision = None
		self.exists = exists
		self.parent = None
		self.t = t
		self.v = v
		self.l = l
		self.k = k
		self.i = i

def invalid_state():
	return State(exists=False, t=np.inf)

class sim_dp_braking:
	def __init__(self):
		self.axes = 3

		self.D_ACCELERATE = 2
		self.D_SUSTAIN = 1
		self.D_BRAKE = 0

		self.policies = [self.accel_policy, self.sustain_policy, self.brake_policy]

	def accel_policy(self, S):
		c, Sp = self.compute_RT(S, self.D_ACCELERATE)
		Sp.decision = self.D_ACCELERATE
		Sp.parent = S

		return Sp

	def sustain_policy(self, S):
		return invalid_state()

	def brake_policy(self, S):
		return invalid_state()

	def fill_edges(self):
		for i in range(self.axes):
			policy = self.policies[i]
			S = self.sg.get_element(0)
			index = [0 for x in range(self.axes)]

			for j in range(1, self.n):
				index[i] = j
				iS = self.sg.convert_index(index)
				S = policy(S)

				self.sg.set_element(iS, S)

	def generate_iS(self, axes):
		iS = []
		for i in range(3):
			for j in range(1, self.n):
				for k in range(1, self.n - j):
					if i == 0:
						iS.append(tuple([0, j, k]))
					elif i == 2:
						iS.append(tuple([j, k, 0]))
					else:
						iS.append(tuple([j, 0, k]))

		iS2 = set()

		for i in range(1, self.n):
			for j in range(1, self.n - i):
				for k in range(1, self.n - i - j):
					iS2.add(tuple([i, j, k]))
					iS2.add(tuple([i, k, j]))
					iS2.add(tuple([j, i, k]))
					iS2.add(tuple([j, k, i]))
					iS2.add(tuple([k, i, j]))
					iS2.add(tuple([k, j, i]))

		iS = iS + sorted(iS2)

		def internal_gen():
			for i in iS:
				yield i

		return internal_gen

	def compute_RT(self, S, d):
		if not S.exists:
			return (np.inf, invalid_state())

		Sp = State()
		Sp.i = S.i + 1

		if Sp.i > len(self.segments):
			return (np.inf, invalid_state())

		Sp.l = self.segments[Sp.i - 1].length
		Sp.k = self.segments[Sp.i - 1].curvature

		a_long = self.compute_a_long(S, d)
		if a_long == None:
			return (np.inf, invalid_state())

		try:
			Sp.v = math.sqrt(S.v**2 + 2 * a_long * S.l)
			vavg = (S.v + Sp.v) / 2.
			Sp.t = S.t + (S.l / vavg)
		except:
			Sp.v = 0.0
			Sp.t = np.inf

		return (Sp.t, Sp)

	def compute_a_long(self, S, d):
		N = (self.vp.mass * self.vp.g) + (self.vp.alpha_downforce() * S.v**2)

		f_tire_lim = (self.vp.mu * N)
		f_tire_lat = (S.k * self.vp.mass * S.v**2)
		if f_tire_lat > f_tire_lim:
			return None
		f_tire_rem = np.sqrt(f_tire_lim**2 - f_tire_lat**2)

		eng, rpm = self.vp.eng_force(S.v, 0)
		engine_force = eng if d == self.D_ACCELERATE else 0

		f_tire_long = np.min([engine_force, f_tire_rem]) if d != self.D_BRAKE else -f_tire_rem
		f_drag = self.vp.alpha_drag() * S.v**2
		f_long = f_tire_long - f_drag

		return float(f_long) / self.vp.mass

	def test_parents(self, index):
		parents = self.sg.get_parents(index)
		min_c = np.inf
		min_S = invalid_state()

		for i, iP in enumerate(parents):
			if iP >= 0:
				p = self.sg.get_element(iP)

				c, S = self.compute_RT(p, i)

				if c < min_c:
					min_c = c
					min_S = S
					min_S.decision = i
					min_S.parent = p

		i = self.sg.convert_index(index)
		self.sg.set_element(i, min_S)

	def find_optimum(self):
		min_t = np.inf
		min_S = invalid_state()

		for iE in self.sg.get_edges():
			edge = self.sg.get_element(iE)

			if edge.t < min_t:
				min_t = edge.t
				min_S = edge

		path = []
		while min_S.decision != None:
			out = []
			out.append(min_S.i)
			out.append(min_S.decision)
			out.append(min_S.t)
			out.append(min_S.v)
			print min_S.i, ":", out
			path.append(out)
			min_S = min_S.parent

		return path[::-1]

	def solve(self, vehicle, segments):
		self.vp = vehicle
		self.segments = segments
		self.n = len(segments) + 1

		self.sg = dpu.nd_structure(self.n, self.axes)
		self.sg.set_element(0, State(l=self.segments[0].length, k=self.segments[0].curvature))

		self.fill_edges()

		iS = self.sg.generate_iS(self.axes)	

		for index in iS:
			self.test_parents(index)

		path = self.find_optimum()

		# add the visualization tool

		exit()

	def steady_solve(self, vehicle, segments):
		return self.solve(vehicle, segments)