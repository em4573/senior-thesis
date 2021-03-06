# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.12
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

from sys import version_info as _swig_python_version_info
if _swig_python_version_info >= (2, 7, 0):
    def swig_import_helper():
        import importlib
        pkg = __name__.rpartition('.')[0]
        mname = '.'.join((pkg, '_pointerheap')).lstrip('.')
        try:
            return importlib.import_module(mname)
        except ImportError:
            return importlib.import_module('_pointerheap')
    _pointerheap = swig_import_helper()
    del swig_import_helper
elif _swig_python_version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_pointerheap', [dirname(__file__)])
        except ImportError:
            import _pointerheap
            return _pointerheap
        try:
            _mod = imp.load_module('_pointerheap', fp, pathname, description)
        finally:
            if fp is not None:
                fp.close()
        return _mod
    _pointerheap = swig_import_helper()
    del swig_import_helper
else:
    import _pointerheap
del _swig_python_version_info

try:
    _swig_property = property
except NameError:
    pass  # Python < 2.2 doesn't have 'property'.

try:
    import builtins as __builtin__
except ImportError:
    import __builtin__

def _swig_setattr_nondynamic(self, class_type, name, value, static=1):
    if (name == "thisown"):
        return self.this.own(value)
    if (name == "this"):
        if type(value).__name__ == 'SwigPyObject':
            self.__dict__[name] = value
            return
    method = class_type.__swig_setmethods__.get(name, None)
    if method:
        return method(self, value)
    if (not static):
        if _newclass:
            object.__setattr__(self, name, value)
        else:
            self.__dict__[name] = value
    else:
        raise AttributeError("You cannot add attributes to %s" % self)


def _swig_setattr(self, class_type, name, value):
    return _swig_setattr_nondynamic(self, class_type, name, value, 0)


def _swig_getattr(self, class_type, name):
    if (name == "thisown"):
        return self.this.own()
    method = class_type.__swig_getmethods__.get(name, None)
    if method:
        return method(self)
    raise AttributeError("'%s' object has no attribute '%s'" % (class_type.__name__, name))


def _swig_repr(self):
    try:
        strthis = "proxy of " + self.this.__repr__()
    except __builtin__.Exception:
        strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)

try:
    _object = object
    _newclass = 1
except __builtin__.Exception:
    class _object:
        pass
    _newclass = 0

class GearData(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, GearData, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, GearData, name)
    __repr__ = _swig_repr
    __swig_setmethods__["gear"] = _pointerheap.GearData_gear_set
    __swig_getmethods__["gear"] = _pointerheap.GearData_gear_get
    if _newclass:
        gear = _swig_property(_pointerheap.GearData_gear_get, _pointerheap.GearData_gear_set)
    __swig_setmethods__["delta"] = _pointerheap.GearData_delta_set
    __swig_getmethods__["delta"] = _pointerheap.GearData_delta_get
    if _newclass:
        delta = _swig_property(_pointerheap.GearData_delta_get, _pointerheap.GearData_delta_set)
    __swig_setmethods__["timeToShift"] = _pointerheap.GearData_timeToShift_set
    __swig_getmethods__["timeToShift"] = _pointerheap.GearData_timeToShift_get
    if _newclass:
        timeToShift = _swig_property(_pointerheap.GearData_timeToShift_get, _pointerheap.GearData_timeToShift_set)

    def __init__(self):
        this = _pointerheap.new_GearData()
        try:
            self.this.append(this)
        except __builtin__.Exception:
            self.this = this
    __swig_destroy__ = _pointerheap.delete_GearData
    __del__ = lambda self: None
GearData_swigregister = _pointerheap.GearData_swigregister
GearData_swigregister(GearData)

class HeapNode(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, HeapNode, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, HeapNode, name)
    __repr__ = _swig_repr
    __swig_setmethods__["score"] = _pointerheap.HeapNode_score_set
    __swig_getmethods__["score"] = _pointerheap.HeapNode_score_get
    if _newclass:
        score = _swig_property(_pointerheap.HeapNode_score_get, _pointerheap.HeapNode_score_set)
    __swig_setmethods__["dependents"] = _pointerheap.HeapNode_dependents_set
    __swig_getmethods__["dependents"] = _pointerheap.HeapNode_dependents_get
    if _newclass:
        dependents = _swig_property(_pointerheap.HeapNode_dependents_get, _pointerheap.HeapNode_dependents_set)
    __swig_setmethods__["step"] = _pointerheap.HeapNode_step_set
    __swig_getmethods__["step"] = _pointerheap.HeapNode_step_get
    if _newclass:
        step = _swig_property(_pointerheap.HeapNode_step_get, _pointerheap.HeapNode_step_set)
    __swig_setmethods__["parent"] = _pointerheap.HeapNode_parent_set
    __swig_getmethods__["parent"] = _pointerheap.HeapNode_parent_get
    if _newclass:
        parent = _swig_property(_pointerheap.HeapNode_parent_get, _pointerheap.HeapNode_parent_set)
    __swig_setmethods__["decisionID"] = _pointerheap.HeapNode_decisionID_set
    __swig_getmethods__["decisionID"] = _pointerheap.HeapNode_decisionID_get
    if _newclass:
        decisionID = _swig_property(_pointerheap.HeapNode_decisionID_get, _pointerheap.HeapNode_decisionID_set)
    __swig_setmethods__["time"] = _pointerheap.HeapNode_time_set
    __swig_getmethods__["time"] = _pointerheap.HeapNode_time_get
    if _newclass:
        time = _swig_property(_pointerheap.HeapNode_time_get, _pointerheap.HeapNode_time_set)
    __swig_setmethods__["velocity"] = _pointerheap.HeapNode_velocity_set
    __swig_getmethods__["velocity"] = _pointerheap.HeapNode_velocity_get
    if _newclass:
        velocity = _swig_property(_pointerheap.HeapNode_velocity_get, _pointerheap.HeapNode_velocity_set)
    __swig_setmethods__["gd"] = _pointerheap.HeapNode_gd_set
    __swig_getmethods__["gd"] = _pointerheap.HeapNode_gd_get
    if _newclass:
        gd = _swig_property(_pointerheap.HeapNode_gd_get, _pointerheap.HeapNode_gd_set)

    def __init__(self):
        this = _pointerheap.new_HeapNode()
        try:
            self.this.append(this)
        except __builtin__.Exception:
            self.this = this
    __swig_destroy__ = _pointerheap.delete_HeapNode
    __del__ = lambda self: None
HeapNode_swigregister = _pointerheap.HeapNode_swigregister
HeapNode_swigregister(HeapNode)


def Init(h):
    return _pointerheap.Init(h)
Init = _pointerheap.Init

def SwapHeaps():
    return _pointerheap.SwapHeaps()
SwapHeaps = _pointerheap.SwapHeaps

def Destroy():
    return _pointerheap.Destroy()
Destroy = _pointerheap.Destroy

def MakeNode(id, dependents, step, parent, decisionID, time, velocity, gear, delta, timeToShift):
    return _pointerheap.MakeNode(id, dependents, step, parent, decisionID, time, velocity, gear, delta, timeToShift)
MakeNode = _pointerheap.MakeNode

def MakeInitNode():
    return _pointerheap.MakeInitNode()
MakeInitNode = _pointerheap.MakeInitNode

def KillNode(node):
    return _pointerheap.KillNode(node)
KillNode = _pointerheap.KillNode

def Insert(node, level, score):
    return _pointerheap.Insert(node, level, score)
Insert = _pointerheap.Insert

def DeleteMin(level):
    return _pointerheap.DeleteMin(level)
DeleteMin = _pointerheap.DeleteMin

def GetDependents(node):
    return _pointerheap.GetDependents(node)
GetDependents = _pointerheap.GetDependents

def SetDependents(node, d):
    return _pointerheap.SetDependents(node, d)
SetDependents = _pointerheap.SetDependents

def GetStep(node):
    return _pointerheap.GetStep(node)
GetStep = _pointerheap.GetStep

def GetDecision(node):
    return _pointerheap.GetDecision(node)
GetDecision = _pointerheap.GetDecision

def GetParent(node):
    return _pointerheap.GetParent(node)
GetParent = _pointerheap.GetParent

def GetTime(node):
    return _pointerheap.GetTime(node)
GetTime = _pointerheap.GetTime

def GetVelocity(node):
    return _pointerheap.GetVelocity(node)
GetVelocity = _pointerheap.GetVelocity

def GetGear(node):
    return _pointerheap.GetGear(node)
GetGear = _pointerheap.GetGear

def GetDelta(node):
    return _pointerheap.GetDelta(node)
GetDelta = _pointerheap.GetDelta

def GetTimeToShift(node):
    return _pointerheap.GetTimeToShift(node)
GetTimeToShift = _pointerheap.GetTimeToShift

def GetScore(node):
    return _pointerheap.GetScore(node)
GetScore = _pointerheap.GetScore
# This file is compatible with both classic and new-style classes.

cvar = _pointerheap.cvar

