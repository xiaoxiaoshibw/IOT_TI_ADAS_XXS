from adas_map.opendrive_parser import parse_opendrive
import pytest


DOCUMENT = """<OpenDRIVE>
  <header revMajor="1" revMinor="4" name="UnitTown"/>
  <road id="1"><lanes><laneSection><left>
    <lane id="1" type="driving"/>
  </left><right><lane id="-1" type="driving"/></right>
  </laneSection></lanes></road>
  <junction id="7"/>
</OpenDRIVE>"""


def test_parses_inventory_and_content_hash():
    first = parse_opendrive(DOCUMENT)
    second = parse_opendrive(DOCUMENT)
    assert first.map_name == 'UnitTown'
    assert first.road_count == 1
    assert first.junction_count == 1
    assert first.driving_lane_count == 2
    assert first.content_hash == second.content_hash
    assert len(first.content_hash) == 64


@pytest.mark.parametrize('document', ['', '<foo/>', '<OpenDRIVE>'])
def test_rejects_invalid_documents(document):
    with pytest.raises(ValueError):
        parse_opendrive(document)
