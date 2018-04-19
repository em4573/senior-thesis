import numpy as np
np.set_printoptions(threshold=np.inf, linewidth=np.inf, precision=4)
import math

class State:
	def __init__(self, exists=True, t=0., v=0., l=-1, k=-1, i=0):
		self.decision = None
		self.parent = None
		self.exists = exists
		self.t = t
		self.v = v
		self.l = l
		self.k = k
		self.i = i

class sim_dp_circle:
	def __init__(self):
		self.D_ACCELERATE = 0
		self.D_SUSTAIN = 1

	def populate_mats(self):
		for i in range(self.n):
			if i == 0:
				self.dmat[:, 0] = np.inf
				self.dmat[0, 0] = 0

				self.smat[:, 0] = State(exists=False)
				self.smat[0, 0] = State(v=0., l=self.segments[0].length, k=self.segments[0].curvature)

				for j in range(1, self.n):
					c, S = self.compute_RT([0, j - 1], self.D_ACCELERATE)
					S.decision = self.D_ACCELERATE
					S.parent = self.smat[0, j - 1]
					self.dmat[0, j] = c
					self.smat[0, j] = S
			else:
				for j in range(1, self.n):
					acc_c, acc_S = self.compute_RT([i, j - 1], self.D_ACCELERATE)
					sus_c, sus_S = self.compute_RT([i - 1, j], self.D_SUSTAIN)

					if sus_c <= acc_c:
						sus_S.decision = self.D_SUSTAIN
						sus_S.parent = self.smat[i - 1, j]
						self.dmat[i, j] = sus_c
						self.smat[i, j] = sus_S
					else:
						acc_S.decision = self.D_ACCELERATE
						acc_S.parent = self.smat[i, j - 1]
						self.dmat[i, j] = acc_c
						self.smat[i, j] = acc_S

			print(i)


	def compute_RT(self, iS, d):
		S = self.smat[iS[0], iS[1]]

		if not S.exists:
			return (np.inf, State(exists=False))

		Sp = State()
		Sp.i = S.i + 1

		if Sp.i > len(self.segments):
			return (np.inf, State(exists=False))

		Sp.l = self.segments[Sp.i - 1].length
		Sp.k = self.segments[Sp.i - 1].curvature

		a_long = self.compute_a_long(S, d)
		if a_long == None:
			return (np.inf, State(exists=False))

		try:
			Sp.v = math.sqrt(S.v**2 + 2 * a_long * S.l)
			# print "should have been: " + str(math.sqrt(math.fabs(S.v**2 + 2 * a_long * S.l)))
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

		f_tire_long = np.min([engine_force, f_tire_rem])
		f_drag = self.vp.alpha_drag() * S.v**2
		f_long = f_tire_long - f_drag
		if f_long < 0:
			print "aha!"

		return float(f_long) / self.vp.mass

	def find_boundary(self):
		min_i = 0
		min_j = self.n - 1
		min_c = self.dmat[min_i, min_j]

		for i in range(self.n):
			test_c = self.dmat[i, self.n - i - 1]
			if test_c < min_c:
				min_c = test_c
				min_i = i
				min_j = self.n - i - 1

		Sc = self.smat[min_i, min_j]
		d_path = []

		while Sc.decision != None:
			print str(Sc.i) + "\t" + str(Sc.v)
			d_path.append(Sc.decision)
			Sc = Sc.parent

		return d_path[::-1]

	def solve(self, vehicle, segments):
		self.vp = vehicle
		self.segments = segments
		self.n = len(segments) + 1

		self.dmat = np.zeros([self.n, self.n])
		self.smat = np.empty([self.n, self.n], dtype=object)

		self.populate_mats()
		path = self.find_boundary()
		print path
		print self.dmat

		exit()

	def steady_solve(self, vehicle, segments):
		return solve(vehicle, segments)