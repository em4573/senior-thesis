import numpy as np
import better_dp_utils as dpu
import math, time, pdb
from constants import *
from sim_pointmass import *
import pointerheap as heap

# import gc
# gc.disable()

# dpalign doublearray

# import pdb; pdb.set_trace()

class sim_dp_nd_template:
	def __init__(self):	
		self.maxint = heap.cvar.maxint

		self.axes = 3
		self.policies = [None for x in range(self.axes)]
		self.pre_pop = []
		
		# self.done = set()
		# self.done.add(0)

		#self.policies[D_SHIFT_UP] = self.shift_up_policy
		self.policies[D_ACCELERATE] = self.accel_policy
		# self.policies[D_SHIFT_DOWN] = self.shift_down_policy
		self.policies[D_SUSTAIN] = self.sustain_policy
		self.policies[D_BRAKE] = self.brake_policy

	def accel_policy(self, S):
		c, Sp = self.compute_RT(S, D_ACCELERATE)

		if Sp != None:
			Sp[G_DECISION] = D_ACCELERATE

		return Sp

	def sustain_policy(self, S):
		return None

	def brake_policy(self, S):
		return None

	def shift_up_policy(self, S):
		return None

	def shift_down_policy(self, S):
		return None

	def fill_axes(self):
		for i in range(self.axes):
			policy = self.policies[i]
			S = self.sg.get_element(0)
			index = [0 for x in range(self.axes)]

			for j in range(1, self.n):
				index[i] = j
				iS = self.sg.convert_to_index(index)
				S = policy(S)

				if S == None:
					break;
				else:
					S[G_ID] = iS
					S[G_INDEX] = list(index)
					self.sg.set_element(iS, tuple(S))
					self.pre_pop.append(S[G_INDEX])
					# self.done.add(iS)

	def compute_RT(self, S, d):
		#print("computing rt for ", d, S)
		if not self.good_to_go(S):
			return (self.maxint, None)
		elif S[G_STEP] + 1 > len(self.segments):
			return (self.maxint, None)
		elif S[G_GEAR_DATA][1] != 0:
			if S[G_GEAR_DATA][1] == 1 and d != D_SHIFT_UP:
				return (self.maxint, None)
			elif S[G_GEAR_DATA][1] == -1 and d != D_SHIFT_DOWN:
				return (self.maxint, None)

		Sp = [0, 
			0,
			S[G_STEP] + 1,
			None,
			None,
			self.maxint ,
			0.0,
			[],
			self.maxint
		]

		a_long, g = self.compute_a_long(S, d)
		Sp[G_GEAR_DATA] = g

		if a_long == None:
			return (np.inf, None)

		dx = self.segments[S[G_STEP] - 1].length
		
		try:
			Sp[G_VELOCITY] = math.sqrt(S[G_VELOCITY]**2 + 2 * a_long * dx)
		except:
			return (np.inf, None)

		vavg = (S[G_VELOCITY] + Sp[G_VELOCITY]) / float(2)

		if vavg <= 0:
			return (np.inf, None)

		Sp[G_TIME] = S[G_TIME] + (dx / float(vavg))
		# Sp[G_SCORE] = Sp[G_TIME]

		return (Sp[G_TIME], Sp)

	def compute_a_long(self, S, d):
		v = S[G_VELOCITY]
		N = (self.vp.mass * self.vp.g) + (self.vp.alpha_downforce() * v**2)
		k = self.segments[S[G_STEP] - 1].curvature

		f_tire_lim = (self.vp.mu * N)
		f_tire_lat = (k * self.vp.mass * v**2)
		if f_tire_lat > f_tire_lim:
			return (None, S[G_GEAR_DATA])
		f_tire_rem = np.sqrt(f_tire_lim**2 - f_tire_lat**2)


		gd = list(S[G_GEAR_DATA])
		# 0 = gear, 1 = delta, 2 = time to shift
		if d == D_SHIFT_UP or d == D_SHIFT_DOWN:
			if gd[1] == 0:
				gd[1] = 1 if d == D_SHIFT_UP else -1
				gd[2] = S[G_TIME]
			else:
				dt = S[G_TIME] - S[G_GEAR_DATA][2]

				if dt >= self.vp.shift_time:
					gd[1] = 0

					gd[0] = np.min([gd[0] + gd[1], len(self.vp.gears) - 1])
					gd[0] = np.max([gd[0], 0])

		eng, rpm = self.vp.eng_force(v, gd[0])
		engine_force = eng if d == D_ACCELERATE else 0

		f_tire_long = np.min([engine_force, f_tire_rem]) if d != D_BRAKE else -f_tire_rem
		f_drag = self.vp.alpha_drag() * v**2
		f_long = f_tire_long - f_drag

		return (float(f_long) / self.vp.mass, tuple(gd))

	def good_to_go(self, S):
		# return S[G_TIME] <= self.incumbent[S[G_STEP] - 1, O_TIME] + (self.max_cost / 7)
		return True

	def find_optimum(self, endStates):
		if len(endStates) <= 0:
			print("did not succeed")
			exit()
		min_t = self.maxint
		min_S = None

		for statePointer in endStates:
			#if statePointer == None:
			#print("fo:", statePointer)
			if heap.GetTime(statePointer) < min_t:
				min_t = heap.GetTime(statePointer)
				min_S = statePointer

		path = []
		output = np.zeros([self.n, O_MATRIX_COLS])
		#if min_S == None:
		#print("foo:", min_S)
		output[0, :] = np.array([
				heap.GetTime(min_S),
				heap.GetStep(min_S),
				heap.GetVelocity(min_S),
				0,
				0,
				0,
				heap.GetDecision(min_S),
				heap.GetGear(min_S),
				0,
				0,
				0,
				0,
				self.segments[heap.GetStep(min_S) - 1].curvature,
				0,
				0
			])

		i = self.n - 1

		while heap.GetParent(min_S) != None:
			#print(nodeToList(min_S))
			min_S = heap.GetParent(min_S)
			#if min_S == None:
			#print("fow:", min_S)
			#print(min_S)
			path.append(heap.GetDecision(min_S))

			output[i, :] = np.array([
				heap.GetTime(min_S),
				heap.GetStep(min_S),
				heap.GetVelocity(min_S),
				0,
				0,
				0,
				heap.GetDecision(min_S),
				heap.GetGear(min_S),
				0,
				0,
				0,
				0,
				self.segments[heap.GetStep(min_S) - 1].curvature,
				0,
				0
			])

			i = i - 1

		print(output[:, O_STATUS])
		return (path[::-1], output)

	def solve(self, vehicle, segments):
		# constants/structures
		self.vp = vehicle
		self.segments = segments
		self.n = len(segments) + 1
		self.rk = RelationshipKeeper(self.axes, self.n)

		# get incumbent solution
		i = sim_pointmass()
		self.incumbent = i.solve(vehicle, segments)
		self.max_cost = self.incumbent[-1, O_TIME]
		#print("max time:", self.max_cost)
		#print("max int: ", self.maxint)

		# set up base case/start node
		heap.Init(self.n ** (self.axes - 1))
		init_state = heap.MakeInitNode()
		if not init_state:
			#print("init fuqqed")
			exit()
		#print("initialized: ", init_state)
		self.rk.addNode(0, init_state)

		# prepare for loop
		heapIndex = 0
		processed = -1		
		end_states = []
		axisLeaders = [heap.MakeInitNode() for a in range(self.axes)]
		for a in axisLeaders:
			if not a:
				#print("axis fuqqed")
				exit()
			heap.SetParent(a, init_state)
		#print("AL0: ", axisLeaders)

		for i, leader in enumerate(axisLeaders):
			if leader != None:
				listLeader = nodeToList(leader)
				newNode = self.policies[i](listLeader)
				#print("newleaders: ", i, newNode)

				if newNode == None:
					axisLeaders[i] = None
					# heap.KillNode(leader)
				else:
					newNode[G_ID] = listLeader[G_ID] + (self.n ** (self.axes - i - 1))
					newNode[G_PARENT] = leader
					axisLeaders[i] = listToNode(newNode)
					self.rk.addNode(newNode[G_ID], axisLeaders[i])

					children = self.rk.makeChildren(newNode[G_ID])
					print(children)
					for child in children:
						print(heap.GetID(child))
						#if child == None:
						#print("temp:", child)
						#if axisLeaders[i] == None:
						#print("temp2:", axisLeaders[i])
						ret = heap.Insert(child, heapIndex, newNode[G_TIME])
						if ret > 0:
							print("HEY LOOK: " + str(ret))
							print(child, heapIndex, newNode[G_TIME])
					added = True
		### AAAAAAAH
		levels = 0
		while(added):
			added = False
			debugon = False
			levels += 1
			print("processed: ", levels, processed, heap.cvar.nodesMade, heap.cvar.workingHeapSize + heap.cvar.reserveHeapSize)

			# if levels == 359:
			# 	debugon = True
			# 	pdb.set_trace()

			# move reserved nodes to working heap
			# heap.SwapHeaps()
			#print("working: ", heap.cvar.workingHeapSize)
			#print("reserve: ", heap.cvar.reserveHeapSize)
			#print("heapInd: ", heapIndex)

			workingNode = heap.DeleteMin(heapIndex)

			while(workingNode != None):
				# handy counter
				processed += 1					

				# constants
				workingID = heap.GetID(workingNode)
				#print("working on: ", workingID, workingNode, nodeToList(workingNode))
				optimalState = None
				minCost = self.maxint
				parents = self.rk.getParentPointers(workingID)
				#print("parents: ", parents)

				# find best parent if any
				for parent in parents:
					decisionID, parentID = parent
					parentPointer = self.rk.getNode(parentID)

					if parentPointer != None:
						parentList = nodeToList(parentPointer)

						cost, newState = self.compute_RT(parentList, decisionID)
						#print("NS: ", cost, newState)

						if newState != None and cost < minCost:
							minCost = cost
							optimalState = newState

							optimalState[G_PARENT] = parentPointer
							optimalState[G_DECISION] = decisionID
							optimalState[G_ID] = workingID

				#print("OS: ", optimalState)
				# assign parent or kill all the nodes
				if minCost < self.maxint:
					#print("success: ", minCost)
					#print("here")
					
					heap.FreeNode(workingNode)
					optimalNode = listToNode(optimalState)
					self.rk.addNode(workingID, optimalNode)

					if optimalState[G_STEP] == self.n - 1:
						end_states.append(optimalNode)
					else:
						children = self.rk.makeChildren(workingID)
						for child in children:
							added = True
							#if child == None:
							#print("minf:", child)
							#if workingNode == None:
							#print("minf2:", workingNode)
							#print("im okay")
							heap.Insert(child, abs(heapIndex - 1), minCost)
							if ret > 0:
								print("HEY LOOK: " + str(ret))
								print(child, heapIndex, optimalState[G_TIME])
							#print("im okay i swear")
				else:
					# print("or here", levels, heap.cvar.nodesMade, (heap.cvar.workingHeapSize + heap.cvar.reserveHeapSize))
					# heap.KillNode(workingNode)
					pass

				#print("could it be... me?")
				workingNode = heap.DeleteMin(heapIndex)
				#print("phew")

			# refresh axis leaders
			# for i, leader in enumerate(axisLeaders):
			# 	if leader != None:
			# 		#print("are we about to have a problem?")
			# 		#print(axisLeaders)
			# 		listLeader = nodeToList(leader)
			# 		newNode = self.policies[i](listLeader)
			# 		#print("newleaders: ", i, newNode)

			# 		if newNode == None:
			# 			axisLeaders[i] = None
			# 			# heap.KillNode(leader)
			# 		else:
			# 			newNode[G_ID] = listLeader[G_ID] + (self.n ** (self.axes - i - 1))
			# 			newNode[G_PARENT] = leader
			# 			axisLeaders[i] = listToNode(newNode)
			# 			self.rk.addNode(newNode[G_ID], axisLeaders[i])

			# 			children = self.rk.makeChildren(newNode[G_ID])
			# 			for child in children:
			# 				#if child == None:
			# 				#print("ref:", child)
			# 				#if axisLeaders[i] == None:
			# 				#print("ref2:", axisLeaders[i])
			# 				#print("it chill")
			# 				heap.Insert(child, abs(heapIndex - 1), newNode[G_TIME])
			# 				if ret > 0:
			# 					print("HEY LOOK: " + str(ret))
			# 					print(child, heapIndex, newNode[G_TIME])
			# 				#print("it so chill")
			# 			added = True
			#print("AL: ", axisLeaders)

			heapIndex = abs(heapIndex - 1)

		#print("ES: ", end_states)
		path, output = self.find_optimum(end_states)

		#print(path)

		return output

	def steady_solve(self, vehicle, segments):
		return self.solve(vehicle, segments)

def printNode(n):
	#if n == None:
	#print("print:", n)
	print("ID: ", heap.GetID(n))
	print("Dependents: ", heap.GetDependents(n))
	print("Step: ", heap.GetStep(n))
	print("Decision: ", heap.GetDecision(n))
	print("Parent: ", heap.GetParent(n))
	print("Time: ", heap.GetTime(n))
	print("Velocity: ", heap.GetVelocity(n))
	print("Gear: ", heap.GetGear(n))
	print("Delta: ", heap.GetDelta(n))
	print("Time to shift: ", heap.GetTimeToShift(n))
	print("Score: ", heap.GetScore(n))

def nodeToList(n):
	#if n == None:
	#print("ntl:", n)
	b1 = heap.GetID(n)
	#print("1", b1)
	b2 = heap.GetDependents(n)
	#print("2", b2)
	b3 = heap.GetStep(n)
	#print("3", b3)
	b4 = heap.GetDecision(n)
	#print("4", b4)
	b5 = heap.GetParent(n)
	#print("5", b5)
	b6 = float(heap.GetTime(n))
	#print("6", b6)
	b7 = float(heap.GetVelocity(n))
	#print("7", b7)
	b8 = (heap.GetGear(n), heap.GetDelta(n), heap.GetTimeToShift(n))
	#print("8", b8)
	b9 = float(heap.GetScore(n))
	#print("9", b9)
	b = [b1, b2, b3, b4, b5, b6, b7, b8, b9]

	#print(b)
	return b

def listToNode(l):
	#print("input list: ", l)
	g = list(l[7])
	node = heap.MakeNode(l[0], l[1], l[2], l[4], l[3], float(l[5]), float(l[6]), g[0], g[1], g[2])
	#printNode(node)
	# time.sleep(1)
	#if node == None:
	#print("l:", node)
	if not node:
			#print("ltn fuqqed")
			exit()
	#print("lmade: ", node)
	# print("output node: ", nodeToList(node))
	return node

class RelationshipKeeper:
	def __init__(self, axes, n):
		self.axes = axes
		self.n = n
		self.nodes = {}
		self.nToThe = [self.n ** i for i in range(self.axes)]

	def addNode(self, num, pointer):
		self.nodes[num] = pointer
		# print("added ", self.flatToList(num))

	def getNode(self, num):
		return self.nodes.get(num, None)

	def deleteNode(self, num):
		node = self.nodes.get(num, None)

		#if node == None:
		#print("rkdel:", n)
		if heap.GetDependents(node) <= 0:
			self.nodes.pop(num, None)

	def flatToList(self, num):
		index = []

		for i in range(self.axes):
			index.append(int(num % self.n))
			num = (num - index[-1]) / self.n

		return index

	def listToFlat(self, index) :
		num = 0

		for i in range(self.axes):
			num += self.nToThe[self.axes - i - 1] * index[i]

		return int(num)

	def getParentPointers(self, num):
		index = self.flatToList(num)
		parents = []

		for i in range(self.axes):
			if index[i] > 0:
				parent = list(index)
				parent[i] = index[i] - 1
				parents.append((i, self.listToFlat(parent)))

		return parents

	def makeChildren(self, num):
		index = self.flatToList(num)
		children = []

		for i in range(self.axes):
			if index[i] < self.n - 1:
				child = list(index)
				child[i] = index[i] + 1
				#if 0 not in child:
				children.append(self.listToFlat(child))

		childPointers = []
		for child in children:
			if child not in self.nodes:
				n = heap.MakeInitNode()
				#if child == None:
				#print("c:", n)
				if not n:
					#print("child fuqqed")
					exit()
				#print("cmade: ", n)
				heap.SetID(n, child)
				self.addNode(child, n)
				childPointers.append(n)

		return childPointers