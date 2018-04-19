import sys,os
sys.path.append(os.path.dirname(__file__))

import input_processing
import batcher

class Object(object):
    pass

def makeSegment():
	x = Object()
	x.length = 2.
	x.curvature = 0.
	x.sector = 0

	return x

def makeCurvedSegment(r):
	x = Object()
	x.length = 2.
	x.curvature = 1. / r
	x.sector = 0

	return x

circle = [makeCurvedSegment(15) for x in range(5)]
tight = [makeCurvedSegment(3) for x in range(5)]
line = [makeSegment() for x in range(5)]

hook = line + circle
double_loop = (line * 5) + (circle * 3) + (line * 5) + (tight * 4) + (tight)
ax = line * 30

tests, vehicle, tracks, model, out = input_processing.process_input("dp_braking.yaml")
tracks = [(ax, False, "Circle")]
results = batcher.batch(tests, vehicle, tracks, model, out[1] != 0)
