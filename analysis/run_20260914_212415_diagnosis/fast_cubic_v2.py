"""Offline prototype: identical Hermite operation order, array evaluation.
Full math.hypot/atan2 and scalar curvature calculation remain unchanged.
"""
import math
import numpy as np
from functools import lru_cache
from fast_cubic_prototype import basis

@lru_cache(maxsize=128)
def array_basis(count):
    a=np.asarray(basis(count),dtype=float).transpose(1,2,0)
    a.flags.writeable=False
    return a

def cubic_connector(start,goal,spacing=.05,scale=1.):
    distance=math.dist(start[:2],goal[:2])
    chord=(goal[0]-start[0],goal[1]-start[1])
    if distance<spacing or any(chord[0]*math.cos(yaw)+chord[1]*math.sin(yaw)<=0 for yaw in (start[2],goal[2])):
        return []
    tangent=distance*scale
    m0=(tangent*math.cos(start[2]),tangent*math.sin(start[2]))
    m1=(tangent*math.cos(goal[2]),tangent*math.sin(goal[2]))
    coords=np.asarray(((start[0],m0[0],goal[0],m1[0]),(start[1],m0[1],goal[1],m1[1])))
    count=max(2,math.ceil(distance*(2+2*scale)/spacing))
    b=array_basis(count)
    values=0+b[:,0,:,None]*coords[:,0]
    values+=b[:,1,:,None]*coords[:,1]
    values+=b[:,2,:,None]*coords[:,2]
    values+=b[:,3,:,None]*coords[:,3]
    out=[]
    for (x,y),(dx,dy),(ddx,ddy) in zip(*values.tolist()):
        speed=math.hypot(dx,dy)
        if speed<1e-8:return []
        out.append((x,y,math.atan2(dy,dx),(dx*ddy-dy*ddx)/speed**3))
    return out
