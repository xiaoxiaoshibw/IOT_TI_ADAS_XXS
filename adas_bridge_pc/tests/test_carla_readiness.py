#!/usr/bin/env python3
"""Deterministic tests for the CARLA RPC/world readiness policy."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace


MODULE_PATH = Path(__file__).parents[1] / "scripts" / "carla_readiness.py"
SPEC = importlib.util.spec_from_file_location("carla_readiness", MODULE_PATH)
readiness = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(readiness)


class FakeWorld:
    def __init__(self, town="Carla/Maps/Town04", synchronous=False):
        self.town = town
        self.synchronous = synchronous
        self.frame = 0

    def get_map(self):
        return SimpleNamespace(name=self.town)

    def get_settings(self):
        return SimpleNamespace(
            synchronous_mode=self.synchronous, fixed_delta_seconds=None
        )

    def get_snapshot(self):
        if not self.synchronous:
            self.frame += 1
        return SimpleNamespace(
            frame=self.frame,
            timestamp=SimpleNamespace(elapsed_seconds=self.frame * 0.05),
        )


class FakeClient:
    def __init__(self, world=None, client_version="0.9.16", server_version="0.9.16"):
        self.world = world or FakeWorld()
        self.client_version = client_version
        self.server_version = server_version
        self.load_calls = []

    def get_client_version(self):
        return self.client_version

    def get_server_version(self):
        return self.server_version

    def get_world(self):
        return self.world

    def load_world(self, town):
        self.load_calls.append(town)
        self.world.town = f"Carla/Maps/{town}"
        return self.world


def test_matching_version_async_world_passes_sample():
    sample = readiness.validate_sample(FakeClient(), "Town04")
    assert sample["town_matches"] is True
    assert sample["frame"] == 1
    assert sample["synchronous_mode"] is False


def test_version_mismatch_is_rejected():
    client = FakeClient(client_version="0.9.16", server_version="0.9.15")
    try:
        readiness.validate_sample(client, "Town04")
    except RuntimeError as exc:
        assert "version mismatch" in str(exc)
    else:
        raise AssertionError("version mismatch was accepted")


def test_sync_world_without_tick_is_read_non_blocking():
    sample = readiness.validate_sample(FakeClient(FakeWorld(synchronous=True)), "Town04")
    assert sample["synchronous_mode"] is True
    assert sample["frame"] == 0


def test_town_normalization():
    assert readiness.normalize_town("Carla/Maps/Town04") == "Town04"
    assert readiness.normalize_town("Town04/") == "Town04"
