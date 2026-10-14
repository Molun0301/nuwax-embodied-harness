"""真实二维图、导航守卫和HTTP设施桥接的双楼层本地闭环。"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time
from urllib.parse import unquote

from scripts.diagnostics.exploration_closed_loop import run


class FacilityWorld:
    """仅模拟设施状态，不替机器人产生导航结果或地图定位结果。"""
    def __init__(self,scenario="complete"):
        self.scenario=scenario;self.submissions=[];self.dropped_receipt=False
        self.lock=threading.RLock();self.session=None;self.floor="F1";self.sequence=0
        self.operations={};self.commands=[];self.runtime=None;self.world=None;self.errors=[]

    def state(self):
        with self.lock:
            self.sequence+=1
            return {"provider_id":"sim_facility","facility_id":"lift","session_id":self.session,"sequence":self.sequence,
                    "observed_at":datetime.now(timezone.utc).isoformat(),"available":True,"floor_id":self.floor,"door":"open","moving":False,
                    "occupied_by_other":False,"docking_calibration_ref":"synthetic-dock-v1","docking_verified":True}

    def apply(self, request):
        with self.lock:
            key=request["operation_id"]
            if key in self.operations:
                return self.operations[key]
            self.commands.append(dict(request))
            action=request["action"]
            if action=="reserve":self.session=request["session_id"]
            if request["session_id"]!=self.session:raise ValueError("设施会话错误")
            if action=="call":self.floor=request["target_floor_id"]
            if action=="transport":
                context=self.runtime.platform_geometry_runtime.get_mapping_session_context()
                assert context["active"] is False,"运载前必须停止旧层输入"
                assert max(abs(v)for v in self.world.snapshot()["velocity"])<1e-4,"运载前底盘必须实际停住"
                self.floor="F1" if self.scenario=="wrong-floor" else request["target_floor_id"]
            result={**request,"state":"succeeded","message":"本地设施已确认操作。"}
            self.operations[key]=result
            return result


def main(output,timeout,scenario="complete"):
    facility=FacilityWorld(scenario)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def reply(self,value,status=200):
            data=json.dumps(value).encode();self.send_response(status);self.send_header("Content-Type","application/json");self.send_header("Content-Length",str(len(data)));self.end_headers();self.wfile.write(data)
        def do_GET(self):
            parts=[unquote(p)for p in self.path.strip("/").split("/")]
            if parts==["v1","facilities","lift","state"]:self.reply(facility.state());return
            if len(parts)==7 and parts[:3]==["v1","facilities","lift"] and parts[3]=="sessions" and parts[5]=="operations":
                value=facility.operations.get(parts[6])
                if value is not None:self.reply(value);return
            self.reply({"error":"操作不存在或路径不匹配"},404)
        def do_PUT(self):
            try:
                request=json.loads(self.rfile.read(min(65537,int(self.headers.get("Content-Length","0")))))
                facility.submissions.append(dict(request))
                result=facility.apply(request)
                if facility.scenario=="lost-receipt" and request["action"]=="transport" and not facility.dropped_receipt:
                    facility.dropped_receipt=True;self.close_connection=True;return
                self.reply(result)
            except Exception as error:
                facility.errors.append(repr(error));self.reply({"error":str(error)},500)
    server=ThreadingHTTPServer(("127.0.0.1",0),Handler)
    worker=threading.Thread(target=server.serve_forever,name="本地设施HTTP",daemon=False);worker.start()

    def configure(config,world):
        config.geometry.navigation.facility_providers_json=json.dumps({"sim_facility":{"type":"http","base_url":f"http://127.0.0.1:{server.server_port}"}})
        facility.world=world

    def exercise(runtime,world,rpc,call,report,snapshots):
        facility.runtime=runtime
        report["components"].append("真实HTTP设施桥接与楼层状态机")
        report["simulation"]="两个同形五米房间的二维完整图；8米瞬时激光；真实有限加减速导航与守卫；设施仅模拟停层、开门和预约，不模拟电梯机械。"
        def wait_task(reply,acceptable=("succeeded",),monitor=None):
            task_id=reply.get("task_id") or reply.get("task",{}).get("task_id")
            if not task_id:return reply
            report["task_id"]=task_id
            deadline=time.monotonic()+timeout
            while time.monotonic()<deadline:
                result=call("get_task_status",{"task_id":task_id})
                status=result.get("status",{}).get("state")
                snapshots.append({"time":time.time(),"task":result,"world":world.snapshot()})
                if monitor is not None:monitor(task_id,result)
                if status in acceptable:return result
                if status in {"succeeded","failed","cancelled","timeout"}:raise RuntimeError(json.dumps({"status":result.get("status"),"error_payload":result.get("error_payload")},ensure_ascii=False))
                time.sleep(0.2)
            raise TimeoutError("楼层测试任务超时")
        assets=[]
        for floor_id,name in ([("F1","模拟一层")] if scenario=="explore-unmapped" else [("F1","模拟一层"),("F2","模拟二层")]):
            call("start_mapping",{"map_name":name})
            deadline=time.monotonic()+20
            while time.monotonic()<deadline:
                snapshot=runtime.mapping_service.get_latest_snapshot()
                if snapshot is not None and snapshot.map_name==name and snapshot.occupancy_grid is not None and sum(v>=0 for v in snapshot.occupancy_grid.data)>(1000 if scenario=="explore-cancel-resume" else 8000):break
                time.sleep(0.1)
            else:raise RuntimeError("未取得完整房间扫描地图")
            wait_task(call("finalize_active_map",{"map_name":name,"reason":"双楼层本地资产准备"}))
            asset=runtime.map_workspace_service._repository.get_map_asset(name)
            assert asset.finalized and asset.asset_manifest_sha256
            assets.append({"floor_id":floor_id,"display_name":name,"map_name":name,"asset_manifest_sha256":asset.asset_manifest_sha256})
            call("stop_mapping",{"reason":"准备下一层"})
        if scenario=="explore-unmapped":
            assets.append({"floor_id":"F2","display_name":"模拟二层","map_name":"模拟二层","asset_manifest_sha256":None})
        wait_task(call("activate_map",{"map_name":"模拟一层"}))
        wait_task(call("relocalize_active_map",{"map_name":"模拟一层","planar_seed":[0.0,0.0,0.0],"timeout_sec":20}))
        topology={"building_id":"模拟建筑","display_name":"模拟建筑","revision":1,"floors":assets,"connections":[{
            "connection_id":"一到二","from_floor_id":"F1","to_floor_id":"F2","provider_id":"sim_facility","facility_id":"lift",
            "waiting_pose":{"x":-0.8,"y":0.0,"yaw_rad":0.0},"boarding_pose":{"x":0.0,"y":0.0,"yaw_rad":0.0},
            "arrival_pose":{"x":0.0,"y":0.0,"yaw_rad":0.0},"exit_pose":{"x":0.8,"y":0.0,"yaw_rad":0.0},
            "docking_calibration_ref":"synthetic-dock-v1","expected_transfer_sec":2.0}]}
        call("register_building",{"topology":topology,"expected_revision":0})
        report["scenario"]="building-"+scenario
        if scenario in {"explore-unmapped","explore-cancel-resume"}:
            reply=rpc("tools/call",{"name":"explore_building","arguments":{"building_id":"模拟建筑","floor_ids":["F1","F2"],"stop_conditions":{"no_frontier_timeout_ms":1000},"per_floor_timeout_sec":120,"timeout_sec":timeout}})
            workflow_cancelled=False
            def monitor_workflow(task_id,result):
                nonlocal workflow_cancelled
                if scenario=="explore-cancel-resume" and not workflow_cancelled and world.snapshot()["distance"]>0.3:
                    call("cancel_task",{"task_id":task_id});workflow_cancelled=True
            report["terminal"]=wait_task(reply,acceptable=("cancelled",) if scenario=="explore-cancel-resume" else ("succeeded",),monitor=monitor_workflow)
            if scenario=="explore-cancel-resume":
                paused=call("get_building_exploration_state")["state"]
                assert workflow_cancelled and paused["needs_reconciliation"] and not paused["terminal"]
                report["paused_building_exploration"]=paused
                report["terminal"]=wait_task(rpc("tools/call",{"name":"resume_building_exploration","arguments":{"request_id":paused["request"]["request_id"]}}))
            report["building_exploration"]=call("get_building_exploration_state")["state"]
            assert report["building_exploration"]["terminal"] and set(report["building_exploration"]["completed_floors"])=={"F1","F2"}
            assert runtime._get_building_catalog().get("模拟建筑").floors[1].asset_manifest_sha256
            assert facility.floor=="F2" and not facility.errors
            report["world"]=world.snapshot();report["facility_commands"]=facility.commands;report["passed"]=True;return
        cancelled=False;wrong_since=None
        def monitor(task_id,result):
            nonlocal cancelled,wrong_since
            state=runtime._get_building_navigation_service().journal.load()
            if state is None or cancelled:return
            if scenario=="cancel-resume" and state.stage=="boarding" and world.snapshot()["x"]>-0.65:
                call("cancel_task",{"task_id":task_id});cancelled=True
            elif scenario=="wrong-floor" and state.stage=="waiting_arrival":
                assert runtime.map_workspace_service.get_active_map_name()=="模拟一层","错层不能激活目标图"
                wrong_since=wrong_since or time.monotonic()
                if time.monotonic()-wrong_since>1.0:
                    call("cancel_task",{"task_id":task_id});cancelled=True
        reply=rpc("tools/call",{"name":"navigate_to_floor","arguments":{"building_id":"模拟建筑","floor_id":"F2","x":1.2,"y":1.0,"yaw_rad":0.0,"timeout_sec":timeout}})
        report["terminal"]=wait_task(reply,acceptable=("cancelled",) if scenario in {"cancel-resume","wrong-floor"} else ("succeeded",),monitor=monitor)
        if scenario=="cancel-resume":
            state=runtime._get_building_navigation_service().journal.load()
            assert state.needs_reconciliation and not state.terminal and cancelled
            report["cancelled_request"]=state.model_dump(mode="json")
            report["terminal"]=wait_task(rpc("tools/call",{"name":"resume_floor_navigation","arguments":{"request_id":state.request.request_id}}))
        if scenario=="wrong-floor":
            state=runtime._get_building_navigation_service().journal.load()
            assert state.stage=="waiting_arrival" and state.needs_reconciliation and cancelled
            assert max(abs(v)for v in world.snapshot()["velocity"])<1e-6
            report["building_navigation"]=state.model_dump(mode="json");report["facility_commands"]=facility.commands
            report["world"]=world.snapshot();report["passed"]=True;return
        if scenario=="lost-receipt":
            assert facility.dropped_receipt and len([x for x in facility.submissions if x["action"]=="transport"])==1

        report["building_navigation"]=call("get_floor_navigation_state")["state"]
        assert report["building_navigation"]["stage"]=="succeeded"
        assert report["building_navigation"]["request"]["target_map_name"]=="模拟二层"
        assert runtime.map_workspace_service.get_active_map_name()=="模拟二层"
        assert facility.floor=="F2" and not facility.errors
        final=world.snapshot()
        assert (final["x"]-1.2)**2+(final["y"]-1.0)**2<0.15**2
        assert max(abs(v)for v in final["velocity"])<1e-6
        report["facility_commands"]=facility.commands
        report["world"]=final;report["passed"]=True
    try:
        return run(output,timeout,scenario="building",calibrated_motion=True,pipeline="slam_2d",exercise=exercise,configure_runtime=configure,sensor_range_m=2.8 if scenario=="explore-cancel-resume" else 8.0)
    finally:
        server.shutdown();worker.join(5);server.server_close()


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--output-dir",type=Path,required=True);parser.add_argument("--timeout",type=float,default=180);parser.add_argument("--scenario",choices=["complete","lost-receipt","cancel-resume","wrong-floor","explore-unmapped","explore-cancel-resume"],default="complete")
    args=parser.parse_args();args.output_dir.mkdir(parents=True,exist_ok=False)
    result=main(args.output_dir.resolve(),args.timeout,args.scenario)
    print(json.dumps({key:result.get(key) for key in ("passed","scenario","world","error","shutdown_passed")},ensure_ascii=False))
