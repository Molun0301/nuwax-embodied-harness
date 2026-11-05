"""当前统一输入桥的生命周期和原始二维扫描线协议检查，不启动设备。"""
from pathlib import Path
import json
import struct
import sys
import threading
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from types import SimpleNamespace
from contracts.sensors import CanonicalLaserScan
from services.geometry.input_bridge import InputBridge
from services.geometry.planar_wire import decode_planar_scan


def run():
    emitted=threading.Event();payloads=[]
    class Client:
        def _close_inputs(self):pass
        def publish_laser_scan(self,payload):payloads.append(payload);emitted.set();return True
    class Provider:
        callback=None
        def subscribe_sensor_sample(self,callback):self.callback=callback;return callback
        def unsubscribe_sensor_sample(self,subscription):self.callback=None
    provider=Provider();bridge=InputBridge(client=Client(),info=SimpleNamespace(pipeline="slam_2d",planar_input_config_json="{}"),sensor_sample_provider=provider)
    scan=CanonicalLaserScan(sensor_id="laser",source_sequence=7,timestamp_ns=1791450000123456789,clock_domain="unix_ns",frame_id="laser",registration_state="sensor",registration_frame_id="laser",calibration_ref="synthetic",beam_count=2,angular_layout="uniform",angle_min_rad=-.1,angle_increment_rad=.2,ranges_f32_le=struct.pack("<2f",1.,2.),beam_states_u8=b"\x01\x01",range_min_m=.05,range_max_m=10.,scan_duration_ns=100000000,timestamp_quality="hardware")
    bridge.start()
    try:
        provider.callback(scan);assert emitted.wait(2),bridge.status()
        decoded=decode_planar_scan(payloads[0])
        assert decoded.timestamp_ns==scan.timestamp_ns
        assert decoded.ranges_f32_le==scan.ranges_f32_le
        assert decoded.source_sequence==7
        assert bridge.status()["dropped"]==0
    finally:bridge.stop()
    assert provider.callback is None and not bridge.status()["ready"] and not bridge._worker.is_alive()
    return {"passed":True,"scope":"假输入提供器、真实InputBridge和冻结二维线协议；无设备或原生算法"}

if __name__=="__main__":print(json.dumps(run(),ensure_ascii=False))
