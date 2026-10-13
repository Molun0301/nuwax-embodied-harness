#!/usr/bin/env python3
"""Bounded offline same-point adapter-count diagnostic; no production changes.

Candidate uses the concrete assembly list_adapters API at the original metadata
count sampling point. Service protocol extension needs independent review.
"""
import json
import statistics
import sys
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.check_robot_state_revisions import RobotStateService
from adapters.base import AdapterBase, AdapterConfig, AdapterLifecycleState
from contracts.robot_state import RobotState, RobotControlMode, SafetyState
from core.state_store import StateStore
from drivers.common.assembly import RobotAssemblyStatus

class Adapter(AdapterBase):
    def convert_payload(self, payload):
        return payload

class Robot:
    def __init__(self, count):
        self.manifest = SimpleNamespace(robot_name='synthetic', robot_model='test')
        self.providers = None
        self.calls = 0
        self.adapters = []
        for i in range(count):
            adapter = Adapter(AdapterConfig(name='adapter_%d' % i, category='perception',
                source_kind='test', contract_type='test', metadata={'nested': {'source': 'synthetic'}}))
            adapter.lifecycle_state = AdapterLifecycleState.INITIALIZED
            self.adapters.append(adapter)

    def get_adapter_health_statuses(self):
        self.calls += 1
        return tuple(adapter.health_check() for adapter in self.adapters)

    def list_adapters(self):
        return tuple(self.adapters)

    def get_status(self):
        statuses = self.get_adapter_health_statuses()
        return RobotAssemblyStatus('synthetic', True, 'high', False, False,
            len(statuses), sum(status.is_healthy for status in statuses))

source = (ROOT / 'services/robot_state_service.py').read_text()
old = '"adapter_count": len(self._robot.get_adapter_health_statuses()),'
new = '"adapter_count": len(self._robot.list_adapters()),'
assert source.count(old) == 1
source = source.replace(old, new)
namespace = {}
exec(source, RobotStateService.ingest_robot_state.__globals__, namespace)
Candidate = namespace['RobotStateService']
state = RobotState(robot_id='synthetic', frame_id='base_link', mode=RobotControlMode.IDLE,
                   safety=SafetyState(can_move=True))

def result_values(snapshot):
    data = snapshot.model_dump(mode='json')
    data.pop('timestamp')
    for diagnostic in data['diagnostics']:
        diagnostic.pop('timestamp', None)
    return data

def main():
    rows = []
    for count in (0, 4, 16):
        fixtures = {label: (cls(Robot(count), state_store=StateStore()))
                    for label, cls in [('baseline', RobotStateService), ('same_point_candidate', Candidate)]}
        outputs = {label: result_values(service.ingest_robot_state(state, source_name='benchmark'))
                   for label, service in fixtures.items()}
        assert outputs['baseline'] == outputs['same_point_candidate']
        assert fixtures['baseline']._robot.calls == 3
        assert fixtures['same_point_candidate']._robot.calls == 2
        times = {label: [] for label in fixtures}
        cpu = {label: [] for label in fixtures}
        for round_number in range(7):
            labels = list(fixtures)
            if round_number % 2:
                labels.reverse()
            for label in labels:
                service = fixtures[label]
                start_cpu, start = time.process_time(), time.perf_counter()
                for _ in range(200):
                    service.ingest_robot_state(state, source_name='benchmark')
                times[label].append((time.perf_counter() - start) / 200)
                cpu[label].append((time.process_time() - start_cpu) / 200)
        row = {'adapter_count': count, 'iterations_per_round': 200, 'rounds': 7,
               'seconds_per_ingest': times, 'cpu_seconds_per_ingest': cpu,
               'output_equal': True,
               'median_reduction_pct': 100 * (1 - statistics.median(times['same_point_candidate']) /
                                             statistics.median(times['baseline']))}
        rows.append(row)
        print(json.dumps(row), flush=True)
    # The count remains sampled after get_status even when the registry grows
    # during that call. Reusing the earlier diagnostics count would fail here.
    class GrowingRobot(Robot):
        def get_adapter_health_statuses(self):
            result = super().get_adapter_health_statuses()
            if self.calls == 2:
                self.adapters.extend(Robot(1).adapters)
            return result

    baseline = RobotStateService(GrowingRobot(1), state_store=StateStore()).ingest_robot_state(state, source_name='benchmark')
    candidate = Candidate(GrowingRobot(1), state_store=StateStore()).ingest_robot_state(state, source_name='benchmark')
    assert baseline.metadata['adapter_count'] == candidate.metadata['adapter_count'] == 2
    output = {'python': sys.version, 'rows': rows, 'production_modified': False,
              'dynamic_attachment_counts': {'baseline': 2, 'candidate': 2}}
    Path(sys.argv[1] if len(sys.argv) > 1 else '/tmp/neh-robot-state-health-sampling.json').write_text(json.dumps(output, indent=2))

if __name__ == '__main__':
    main()
