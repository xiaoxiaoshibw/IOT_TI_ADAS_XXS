from adas_dtc_recorder.node import diagnostic_name_matches_source


def test_diagnostic_source_matching_uses_component_boundaries():
    assert diagnostic_name_matches_source("vehicle_interface: runtime",
                                         "vehicle_interface")
    assert diagnostic_name_matches_source("vehicle_interface/runtime",
                                         "vehicle_interface")
    assert not diagnostic_name_matches_source("vehicle_interface_sim: runtime",
                                              "vehicle_interface")
    assert not diagnostic_name_matches_source("prefix_vehicle_interface: runtime",
                                              "vehicle_interface")
