from sim_pointmass import *
from sim_twotires import *
from sim_dp_circle import *
from sim_dp_braking import *
from sim_dp_shift_up import *
from sim_dp_nd_template import *

class Simulation:
	def __init__(self, model_type="point_mass"):
		self.name = model_type
		
		if model_type == "point_mass":
			self.model = sim_pointmass()
		elif model_type == "two_tires":
			self.model = sim_twotires()
		elif model_type == "dp_circle":
			self.model = sim_dp_circle()
		elif model_type == "dp_braking":
			self.model = sim_dp_braking()
		elif model_type == "dp_shift_up":
			self.model = sim_dp_shift_up()
		elif model_type == "dp_nd":
			self.model = sim_dp_nd_template()
		else:
			raise ValueError("Please provide a valid simulation model.")

	def solve(self, vehicle, segments):
		return self.model.solve(vehicle, segments)

	def steady_solve(self, vehicle, segments):
		return self.model.steady_solve(vehicle, segments)