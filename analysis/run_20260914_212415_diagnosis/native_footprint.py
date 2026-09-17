"""Offline native collision prototype. No ROS node or hardware control."""
import ctypes as c
from pathlib import Path
import numpy as np
lib=c.CDLL(str(Path(__file__).with_name('footprint_native.so')))
f=lib.footprint;ptr=c.POINTER(c.c_double)
f.argtypes=[ptr,ptr,ptr,c.c_int64,ptr,c.c_int64]+[c.c_double]*4
f.restype=c.c_bool

def footprint_clear(points,surfaces,width,front,rear,margin):
    edges=[edge for s in surfaces for edge in s.segments()]
    if not edges or not points:return True
    e=np.asarray(edges,dtype=float);p=np.asarray(points,dtype=float)
    cs=np.cos(p[:,2]);ss=np.sin(p[:,2])
    return f(p.ctypes.data_as(ptr),cs.ctypes.data_as(ptr),ss.ctypes.data_as(ptr),len(p),
             e.ctypes.data_as(ptr),len(e),width,front,rear,margin)
