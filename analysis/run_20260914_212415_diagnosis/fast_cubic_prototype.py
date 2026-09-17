"""Offline-only v2 cubic optimization; same formulas, samples, and predicates.
Production code is untouched. Basis terms depend only on count, not the route.
"""
import math
from functools import lru_cache

@lru_cache(maxsize=128)
def basis(count):
    terms=[]
    for j in range(count+1):
        t=j/count
        h=(2*t**3-3*t*t+1,t**3-2*t*t+t,-2*t**3+3*t*t,t**3-t*t)
        d=(6*t*t-6*t,3*t*t-4*t+1,-6*t*t+6*t,3*t*t-2*t)
        dd=(12*t-6,6*t-4,-12*t+6,6*t-2)
        terms.append((h,d,dd))
    return tuple(terms)

def dot(a,b):
    # Keep sum()'s left-to-right order, including its initial zero.
    return (((0+a[0]*b[0])+a[1]*b[1])+a[2]*b[2])+a[3]*b[3]

def cubic_connector(start,goal,spacing=.05,scale=1.):
    distance=math.dist(start[:2],goal[:2])
    chord=(goal[0]-start[0],goal[1]-start[1])
    if distance<spacing or any(chord[0]*math.cos(yaw)+chord[1]*math.sin(yaw)<=0 for yaw in (start[2],goal[2])):
        return []
    tangent=distance*scale
    m0=(tangent*math.cos(start[2]),tangent*math.sin(start[2]))
    m1=(tangent*math.cos(goal[2]),tangent*math.sin(goal[2]))
    coords=((start[0],m0[0],goal[0],m1[0]),(start[1],m0[1],goal[1],m1[1]))
    count=max(2,math.ceil(distance*(2+2*scale)/spacing))
    out=[]
    for h,d,dd in basis(count):
        x,y=dot(h,coords[0]),dot(h,coords[1])
        dx,dy=dot(d,coords[0]),dot(d,coords[1])
        ddx,ddy=dot(dd,coords[0]),dot(dd,coords[1])
        speed=math.hypot(dx,dy)
        if speed<1e-8:return []
        out.append((x,y,math.atan2(dy,dx),(dx*ddy-dy*ddx)/speed**3))
    return out
