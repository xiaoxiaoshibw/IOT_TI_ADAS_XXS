"""
Small, dependency-free OpenDRIVE metadata parser.

CARLA remains the authoritative geometry parser.  This module validates the raw
OpenDRIVE document and provides a content identity used to reject stale maps.
"""

from dataclasses import dataclass
import hashlib
import xml.etree.ElementTree as et


@dataclass(frozen=True)
class OpenDriveMetadata:
    """Validated identity and inventory of an OpenDRIVE document."""

    map_name: str
    content_hash: str
    road_count: int
    junction_count: int
    driving_lane_count: int


def _map_name(root, fallback):
    header = root.find('header')
    if header is not None:
        name = (header.get('name') or '').strip()
        if name:
            return name
    return fallback


def parse_opendrive(document, fallback_name=''):
    """
    Parse and validate raw OpenDRIVE XML.

    Raises ``ValueError`` for empty, malformed, or non-OpenDRIVE input.  The
    hash covers the exact UTF-8 document rather than only the CARLA town name.
    """
    if not isinstance(document, str) or not document.strip():
        raise ValueError('OpenDRIVE document is empty')
    try:
        root = et.fromstring(document)
    except et.ParseError as error:
        raise ValueError('OpenDRIVE XML is malformed: %s' % error) from error
    if root.tag.rsplit('}', 1)[-1] != 'OpenDRIVE':
        raise ValueError('root element is not OpenDRIVE')

    roads = root.findall('road')
    junctions = root.findall('junction')
    driving_lanes = root.findall(".//lane[@type='driving']")
    return OpenDriveMetadata(
        map_name=_map_name(root, fallback_name),
        content_hash=hashlib.sha256(document.encode('utf-8')).hexdigest(),
        road_count=len(roads),
        junction_count=len(junctions),
        driving_lane_count=len(driving_lanes),
    )
