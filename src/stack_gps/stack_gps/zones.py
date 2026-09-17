"""Spatial Zone definitions layered on the existing GPS range classifier.

No path generation, waypoint reach detector, distance threshold or course order.
Mission membership uses only the current station index. GPS-only membership may
also use the discrete waypoint nearest the station preview.
"""
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path

import yaml

from .path_engine import PathEngine


class ZoneType(IntEnum):
    NORMAL_ZONE = 0
    GPS_ONLY_ZONE = 1
    MISSION_ZONE = 2
    LAST_MISSION_ZONE = 3


class MissionType(IntEnum):
    NONE = 0
    T_PARKING = 1
    PARALLEL_PARKING = 2


# Capacity comes from the uint8 wire/bus representation, not a driving threshold.
ID_CAPACITY = 256


@dataclass(frozen=True)
class ZoneDefinition:
    zone_id: int
    zone_type: ZoneType
    start_index: int
    end_index: int
    mission_id: int = 0
    mission_type: MissionType = MissionType.NONE


class ZoneMap:
    def __init__(self, definitions, track_size):
        self.definitions = tuple(sorted(definitions, key=lambda item: item.zone_id))
        seen = set()
        missions = {}
        for zone in self.definitions:
            if not 0 < zone.zone_id < ID_CAPACITY or zone.zone_id in seen:
                raise ValueError('zone_id must be unique and in 1..255')
            if not 0 <= zone.start_index <= zone.end_index < track_size:
                raise ValueError(f'zone {zone.zone_id}: index_range is outside the configured track')
            if not isinstance(zone.zone_type, ZoneType):
                raise ValueError(f'zone {zone.zone_id}: unsupported zone_type')
            if not 0 <= zone.mission_id < ID_CAPACITY:
                raise ValueError('mission_id must be in 0..255')
            if zone.zone_type == ZoneType.MISSION_ZONE:
                if zone.mission_type not in (MissionType.T_PARKING, MissionType.PARALLEL_PARKING):
                    raise ValueError('MISSION_ZONE requires a supported mission_type')
                if zone.mission_id in missions and missions[zone.mission_id] != zone.mission_type:
                    raise ValueError('one mission_id cannot refer to different mission types')
                missions[zone.mission_id] = zone.mission_type
            elif zone.mission_type != MissionType.NONE:
                raise ValueError('only MISSION_ZONE can carry a mission_type')
            seen.add(zone.zone_id)

    def snapshot(self, current_position_index, preview_position_index=None):
        """All memberships, including false levels for configured zones.

        A preview may extend only GPS-only navigation membership. Mission Zones
        stay tied to the current station so a lookahead cannot start parking.
        """
        memberships = []
        for zone in self.definitions:
            bounds = [(zone.start_index, zone.end_index)]
            in_zone = PathEngine._in_ranges(current_position_index, bounds)
            if (not in_zone and preview_position_index is not None and
                    zone.zone_type == ZoneType.GPS_ONLY_ZONE):
                in_zone = PathEngine._in_ranges(preview_position_index, bounds)
            memberships.append((zone, in_zone))
        return tuple(memberships)

    @classmethod
    def from_engine(cls, engine, explicit=()):
        """Reuse existing GPS-only and parking ranges; explicit IDs are reserved.

        Legacy range entries receive the lowest unused IDs in configured order.
        An explicit definition of the same type/range replaces that generated
        entry, so adding metadata cannot accidentally create a second mission.
        """
        definitions = list(explicit)
        used_zone_ids = {zone.zone_id for zone in definitions}
        used_mission_ids = {zone.mission_id for zone in definitions
                            if zone.zone_type == ZoneType.MISSION_ZONE}

        def unused(used, start):
            for value in range(start, ID_CAPACITY):
                if value not in used:
                    used.add(value)
                    return value
            raise ValueError('Zone/Mission ID capacity exceeded')

        for ranges, zone_type, mission_type in (
            (engine.gps_only_ranges, ZoneType.GPS_ONLY_ZONE, MissionType.NONE),
            (engine.parking_ranges, ZoneType.MISSION_ZONE, MissionType.T_PARKING),
            (engine.parallel_parking_ranges, ZoneType.MISSION_ZONE, MissionType.PARALLEL_PARKING),
        ):
            for first, last in ranges:
                if any(zone.zone_type == zone_type and zone.mission_type == mission_type
                       and (zone.start_index, zone.end_index) == (first, last)
                       for zone in definitions):
                    continue
                zone_id = unused(used_zone_ids, 1)
                mission_id = unused(used_mission_ids, 0) if zone_type == ZoneType.MISSION_ZONE else 0
                definitions.append(ZoneDefinition(
                    zone_id, zone_type, first, last, mission_id, mission_type))
        return cls(definitions, len(engine.e))


def load_zone_definitions(path, engine, snap_max_m, *, key="zones", turn_only=False):
    """Load optional 'zones' entries from the existing zones_file.

    Boundary forms: existing latitude/longitude start/end, or index_range.
    Latitude/longitude uses existing index_of + stop_zone_snap_max_m validation.
    No coordinates or boundary widths are supplied by this module.
    """
    if not path or not Path(path).is_file():
        return ()
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    definitions = []
    for entry in data.get(key) or []:
        try:
            zone_type = ZoneType.GPS_ONLY_ZONE if turn_only else ZoneType[entry['zone_type']]
            mission_type = MissionType[entry.get('mission_type', 'NONE')]
            if 'index_range' in entry:
                if 'start' in entry or 'end' in entry:
                    raise ValueError('choose one boundary representation')
                first, last = entry['index_range']
                if type(first) is not int or type(last) is not int:
                    raise ValueError('index_range must contain two integers')
            else:
                start, end = entry['start'], entry['end']
                first, d1 = engine.index_of(float(start['lat']), float(start['lon']))
                last, d2 = engine.index_of(float(end['lat']), float(end['lon']))
                if max(d1, d2) > snap_max_m:
                    raise ValueError('zone boundary exceeds existing stop_zone_snap_max_m')
                first, last = min(first, last), max(first, last)
            zone_id = entry['zone_id']
            mission_id = entry['mission_id'] if zone_type == ZoneType.MISSION_ZONE else 0
            if type(zone_id) is not int or type(mission_id) is not int:
                raise ValueError('zone_id and mission_id must be integers')
            definitions.append(ZoneDefinition(
                zone_id, zone_type, first, last, mission_id, mission_type))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f'invalid ZoneDefinition {entry!r}: {error}') from error
    # Validate explicit entries together before reserving IDs for legacy ranges.
    return ZoneMap(definitions, len(engine.e)).definitions


def turn_zone_map(path, engine, snap_max_m, existing):
    """Revised v2: exclude old GPS-only zones, retain mission IDs and bounds.

    Only explicit turn_zones drive BOTH GPS-only navigation and traffic lifecycle.
    Empty/missing turn_zones means no configured turns; coordinates are never guessed.
    """
    turns = load_zone_definitions(path, engine, snap_max_m, key='turn_zones', turn_only=True)
    retained = [z for z in existing.definitions if z.zone_type != ZoneType.GPS_ONLY_ZONE]
    result = ZoneMap([*retained, *turns], len(engine.e))
    engine.gps_only_ranges = [(z.start_index, z.end_index) for z in turns]
    return result
