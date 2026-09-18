from types import SimpleNamespace
from fma_interfaces.msg import GpsPath
from stack_gps.node import StackGpsNode
from stack_gps.path_engine import PathEngine
from stack_gps.zones import ZoneDefinition, ZoneMap, ZoneType, MissionType


def test_published_zone_flags_agree_with_station_priority():
    zones=ZoneMap([ZoneDefinition(9,ZoneType.MISSION_ZONE,10,20,0,MissionType.T_PARKING),
                   ZoneDefinition(3,ZoneType.GPS_ONLY_ZONE,21,30)],40)
    node=SimpleNamespace(zone_map=zones, engine=SimpleNamespace(
        accel_ranges=[], _in_ranges=PathEngine._in_ranges))
    msg=GpsPath();msg.gps_only_zone=True
    StackGpsNode._fill_zone_context(node,msg,20,23)
    assert not msg.gps_only_zone
    assert {z.zone_id for z in msg.zones if z.in_zone}=={9}
    msg=GpsPath();msg.stop_zone=1
    StackGpsNode._fill_zone_context(node,msg,9,23)
    assert not msg.gps_only_zone and not any(z.in_zone for z in msg.zones)
    msg=GpsPath()
    StackGpsNode._fill_zone_context(node,msg,9,23)
    assert msg.gps_only_zone
