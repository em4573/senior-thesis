# $ swig -python example.i
# $ python setup.py build_ext --inplace

from distutils.core import setup, Extension

pointerheap_module = Extension('_pointerheap', sources=['pointerheap_wrap.c', 'pointerheap.c'])
setup(name='pointerheap', ext_modules=[pointerheap_module], py_modules=["pointerheap"])

# def dumpMin():
# 	n = ph.DeleteMin()
# 	if not n:
# 		return
# 	print str(ph.GetValue(n)) + ": " + str(ph.GetScore(n))
# 	ph.KillNode(n)