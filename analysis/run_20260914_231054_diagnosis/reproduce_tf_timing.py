"""Synthetic TF timing reproduction, not a replay of recorded /tf; no hardware."""
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from tf2_ros import Buffer
buf=Buffer()
def transform(parent,child,nanos,x=0.):
 t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child
 t.header.stamp=Time(nanoseconds=nanos).to_msg();t.transform.translation.x=x
 t.transform.rotation.w=1.;return t
buf.set_transform_static(transform('base_link','v2_vehicle_view',1_000_000_000),'test')
buf.set_transform(transform('map','base_link',1_000_000_000,10.),'test')
buf.set_transform(transform('map','base_link',1_100_000_000,10.1),'test')
for desc,t in [('render now 50ms after latest GPS TF',1_150_000_000),('GPS-aligned marker time',1_100_000_000),('latest available TF',0)]:
 try:
  buf.lookup_transform('map','v2_vehicle_view',Time(nanoseconds=t))
  print(desc+': OK')
 except Exception as e: print(desc+': '+type(e).__name__+': '+str(e))
