#!/usr/bin/env python3
"""Actuator-presence contracts without initialization or hardware startup."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drivers.common.assembly import RobotAssemblyBase
from drivers.customrobot.assembly import CustomRobotAssembly
from drivers.g1.assembly import G1RobotAssembly
from drivers.g1.providers import G1ProviderBundle
from drivers.go2.assembly import Go2RobotAssembly
from drivers.raspi_lidar_robot.assembly import RaspiLidarRobotAssembly
from drivers.raspi_lidar_robot.providers import RaspiLidarRobotProviderBundle


class ActuatorContractTests(unittest.TestCase):
    def scaffold(self, assembly_type, provider_type):
        assembly = object.__new__(assembly_type)
        assembly._adapter_registry = {}
        assembly.providers = provider_type()
        assembly.manifest = SimpleNamespace(capability_matrix=SimpleNamespace(metadata={"implemented": False}))
        return assembly

    def test_base_unknown(self):
        self.assertIsNone(RobotAssemblyBase.has_motion_actuator(object()))

    def test_custom_unbound_has_no_actuator(self):
        assembly = object.__new__(CustomRobotAssembly)
        assembly.motion_control = None
        assembly._adapter_registry = {}
        self.assertIs(assembly.has_motion_actuator(), False)

    def test_custom_injected_disconnected_motion_is_present(self):
        assembly = object.__new__(CustomRobotAssembly)
        assembly.motion_control = Mock()
        assembly.motion_control.is_available.return_value = False
        assembly._adapter_registry = {}
        assembly.providers = Mock()
        assembly.providers.is_available.return_value = False
        self.assertIs(assembly.has_motion_actuator(), True)

    def test_custom_external_adapter_unknown(self):
        assembly = object.__new__(CustomRobotAssembly)
        assembly.motion_control = None
        assembly._adapter_registry = {"external": Mock()}
        self.assertIsNone(assembly.has_motion_actuator())

    def test_go2_disconnection_does_not_erase_actuator(self):
        assembly = object.__new__(Go2RobotAssembly)
        assembly.providers = Mock()
        assembly.providers.is_available.return_value = False
        self.assertIs(assembly.has_motion_actuator(), True)

    def test_empty_registered_scaffolds(self):
        for cls, bundle in ((G1RobotAssembly, G1ProviderBundle),
                            (RaspiLidarRobotAssembly, RaspiLidarRobotProviderBundle)):
            with self.subTest(adapter=cls.__name__):
                assembly = self.scaffold(cls, bundle)
                self.assertIs(assembly.has_motion_actuator(), False)
                assembly._adapter_registry["external"] = Mock()
                self.assertIsNone(assembly.has_motion_actuator())

    def test_modified_scaffold_metadata_fails_closed(self):
        for cls, bundle in ((G1RobotAssembly, G1ProviderBundle),
                            (RaspiLidarRobotAssembly, RaspiLidarRobotProviderBundle)):
            for value in (True, None, 0, "false", ""):
                with self.subTest(adapter=cls.__name__, metadata=value):
                    assembly = self.scaffold(cls, bundle)
                    assembly.manifest.capability_matrix.metadata["implemented"] = value
                    self.assertIsNone(assembly.has_motion_actuator())
            assembly = self.scaffold(cls, bundle)
            assembly.manifest.capability_matrix.metadata.clear()
            self.assertIsNone(assembly.has_motion_actuator())

    def test_replaced_subclassed_or_mutated_provider_unknown(self):
        for cls, bundle in ((G1RobotAssembly, G1ProviderBundle),
                            (RaspiLidarRobotAssembly, RaspiLidarRobotProviderBundle)):
            class FutureProvider(bundle):
                pass

            for provider in (Mock(), FutureProvider(), None):
                with self.subTest(adapter=cls.__name__, provider=type(provider).__name__):
                    assembly = self.scaffold(cls, bundle)
                    assembly.providers = provider
                    self.assertIsNone(assembly.has_motion_actuator())
            assembly = self.scaffold(cls, bundle)
            assembly.providers.motion_control = Mock()
            self.assertIsNone(assembly.has_motion_actuator())


if __name__ == "__main__":
    unittest.main()
