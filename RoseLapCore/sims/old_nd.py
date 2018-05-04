import numpy as np
import dp_utils as dpu
np.set_printoptions(threshold=np.inf, linewidth=np.inf, precision=4)
import math
from constants import *

class State:
	def __init__(self, exists=True, t=0., v=0., l=-1, k=-1, i=0, g=0):
		self.decision = None
		self.exists = exists
		self.parent = None
		self.t = t
		self.v = v
		self.g = g
		self.l = l
		self.k = k
		self.i = i

def invalid_state():
	return State(exists=False, t=np.inf)

class sim_dp_nd_template:
	def __init__(self):
		self.axes = 3
		self.policies = [None for x in range(self.axes)]
		
		self.done = set()
		self.done.add(0)

		# self.policies[D_SHIFT_UP] = self.shift_up_policy
		self.policies[D_ACCELERATE] = self.accel_policy
		# self.policies[D_SHIFT_DOWN] = self.shift_down_policy
		self.policies[D_SUSTAIN] = self.sustain_policy
		self.policies[D_BRAKE] = self.brake_policy

	def accel_policy(self, S):
		c, Sp = self.compute_RT(S, D_ACCELERATE)
		Sp.decision = D_ACCELERATE
		Sp.parent = S

		return Sp

	def sustain_policy(self, S):
		return invalid_state()

	def brake_policy(self, S):
		return invalid_state()

	def shift_up_policy(self, S):
		return invalid_state()

	def shift_down_policy(self, S):
		return invalid_state()

	def fill_axes(self):
		for i in range(self.axes):
			policy = self.policies[i]
			S = self.sg.get_element(0)
			index = [0 for x in range(self.axes)]

			for j in range(1, self.n):
				index[i] = j
				iS = self.sg.convert_to_index(index)
				S = policy(S)

				self.sg.set_element(iS, S)
				self.done.add(iS)

	def compute_RT(self, S, d):
		if not S.exists:
			return (np.inf, invalid_state())
		elif S.t > self.max_time:
			return (np.inf, invalid_state())

		Sp = State()
		Sp.i = S.i + 1

		if Sp.i > len(self.segments):
			return (np.inf, invalid_state())

		Sp.l = self.segments[Sp.i - 1].length
		Sp.k = self.segments[Sp.i - 1].curvature

		a_long, g = self.compute_a_long(S, d)
		Sp.g = g

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
			return (None, S.g)
		f_tire_rem = np.sqrt(f_tire_lim**2 - f_tire_lat**2)

		if d == D_SHIFT_UP:
			S.g = np.min([S.g + 1, len(self.vp.gears) - 1])
		if d == D_SHIFT_DOWN:
			S.g = np.max([S.g - 1, 0])

		eng, rpm = self.vp.eng_force(S.v, S.g)
		engine_force = eng if d == D_ACCELERATE else 0

		f_tire_long = np.min([engine_force, f_tire_rem]) if d != D_BRAKE else -f_tire_rem
		f_drag = self.vp.alpha_drag() * S.v**2
		f_long = f_tire_long - f_drag

		return (float(f_long) / self.vp.mass, S.g)

	def test_parents(self, index):
		if index in self.done:
			return

		parents = self.sg.get_parents(self.sg.convert_to_list(index))
		min_c = np.inf
		min_S = invalid_state()

		for tP in parents:
			i, iP = tP

			p = self.sg.get_element(iP)

			c, S = self.compute_RT(p, i)

			if c < min_c:
				min_c = c
				min_S = S
				min_S.decision = i
				min_S.parent = p

		self.sg.set_element(index, min_S)
		self.done.add(index)

	def find_optimum(self):
		min_t = np.inf
		min_S = invalid_state()

		for iE in self.sg.get_edges():
			edge = self.sg.get_element(iE)

			if edge.t < min_t:
				min_t = edge.t
				min_S = edge

		path = []
		output = np.zeros([self.n - 1, O_MATRIX_COLS])
		i = self.n - 2

		while min_S.decision != None:
			path.append(min_S.decision)

			output[i, :] = np.array([
				min_S.t,
				min_S.i,
				min_S.v,
				0,
				0,
				0,
				min_S.decision,
				min_S.g,
				0,
				0,
				0,
				0,
				min_S.k,
				0,
				0
			])

			i = i - 1

			min_S = min_S.parent

		return (path[::-1], output)

	def solve(self, vehicle, segments):
		self.vp = vehicle
		self.segments = segments
		self.n = len(segments) + 1

		self.max_time = ((self.n * segments[4].length / 5280) / 20) * 60 * 60
		print self.max_time

		self.sg = dpu.nd_structure(self.n, self.axes)
		self.sg.set_element(0, State(l=self.segments[0].length, k=self.segments[0].curvature))

		self.fill_axes()

		iS = self.sg.generate_iS()	

		for i, index in enumerate(iS):
			if i % 10000 == 0:
				print i
			self.test_parents(index)

		path, output = self.find_optimum()

		return output

	def steady_solve(self, vehicle, segments):
		return self.solve(vehicle, segments)