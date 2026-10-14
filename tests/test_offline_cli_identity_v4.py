import contextlib
import copy
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest import mock

ROOT=Path(__file__).resolve().parents[1]
SOURCE=Path(os.environ.get('NEH_CLI_OWNER_SOURCE',str(ROOT/'services/mapping/offline_cli_owner.py')))
spec=importlib.util.spec_from_file_location('qualified_cli_owner_v4',str(SOURCE))
m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m)
ORIGINAL_SHA=m.hashlib.sha256
ORIGINAL_KILLPG=os.killpg
AUTH_CALLS=[]

def auth_only_sha(*args,**kwargs):
    frame=sys._getframe(1)
    while frame:
        if frame.f_code.co_name=='_state_entry_mac' and frame.f_globals.get('__name__')==m.__name__:
            AUTH_CALLS.append('state_mac')
            return ORIGINAL_SHA(*args,**kwargs)
        frame=frame.f_back
    raise AssertionError('Forbidden non-authentication hash construction')

def command_for(kind='summary'):
    if kind=='map': return m.MapCommand('input.jsonl',m._safe_map_segment('real-map'),'real-map','map')
    if kind=='pose-fill': return m.PoseFillCommand('input.jsonl','output.jsonl')
    if kind=='rename': return m.RenameCommand('input.jsonl','output.jsonl',renames=(('lidar','points'),))
    return m.SummaryCommand('input.jsonl','summary.json')

def record_for(kind='summary',legacy=False,request_id='request:original.1',observation=None):
    command=command_for(kind);cmd,_=m._command_contract(command)
    size,fmt,dev,ino,mtime,ctime=observation or (18,'jsonl',3,4,-1,0)
    r={'schema_version':4,'request_id':request_id,'operation_id':'9'*32,'command_kind':kind,'command':cmd,'state':'accepted','terminal':False,
       'dataset_size':size,'dataset_format':fmt,'dataset_device':dev,'dataset_inode':ino,'dataset_mtime_ns':mtime,'dataset_ctime_ns':ctime,
       'timeout_seconds':5.0,'submitted_at_unix_ns':10000000000,'deadline_at_unix_ns':15000000000,'started_at_unix_ns':None,'completed_at_unix_ns':None,
       'pid':None,'process_start_token':None,'pgid':None,'sid':None,'exit_code':None,'stdout':'','stderr':'','stdout_truncated':False,'stderr_truncated':False,
       'output_relative_path':cmd['output_path'],'output_size':None,'receipt':None,
       'release_generation':'a'*64,'release_manifest_sha256':'b'*64,'offline_cli_artifact_sha256':'c'*64,'offline_cli_artifact_size':12,'error_code':None,'message':'accepted'}
    if legacy:
        r['schema_version']=3;r.pop('dataset_mtime_ns');r.pop('dataset_ctime_ns');r.update(request_digest='d'*64,dataset_sha256='e'*64,output_sha256=None)
    return r

def failed(r):
    r.update(state='failed',terminal=True,completed_at_unix_ns=16000000000,error_code='fixture_failed',message='fixture')
    return r

def successful(r):
    r.update(state='succeeded',terminal=True,started_at_unix_ns=11000000000,completed_at_unix_ns=12000000000,pid=123,process_start_token=22,pgid=123,sid=123,exit_code=0,output_size=16)
    if r['schema_version']==3:r['output_sha256']='f'*64
    return r

class Fixture(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(prefix='neh-cli-v4-');self.root=Path(self.tmp.name)
        for name in ['state','input','output']:(self.root/name).mkdir(mode=0o700)
        (self.root/'input/input.jsonl').write_bytes(b'{"source":"real"}\n')
        self.owner=object.__new__(m.OfflineMappingCliOwner)
        o=self.owner;o._state_root=m._StableRoot(self.root/'state','state');o._input_root=m._StableRoot(self.root/'input','input');o._output_root=m._StableRoot(self.root/'output','output')
        kind,info=m._probe_process(os.getpid());assert info is not None
        o._owner_pid=os.getpid();o._owner_start_token=info.start_token;o._record_names={};o._volatile_records={};o._resources_closed=False;o._closing=False;o._poisoned=None;o._active=None;o._condition=threading.Condition(threading.RLock())
        o._release_generation='a'*64;o._release_manifest_sha256='b'*64;o._artifact_sha256='c'*64;o._artifact_size=12
        o._termination_grace_seconds=.3;o._kill_grace_seconds=.8;o._poll_interval_seconds=.01
        o._state_store=m._OwnerStateStore(o._state_root,owner_pid=o._owner_pid,owner_start_token=o._owner_start_token,state_auth_key=b'public-test-only-state-auth-key-000')
        o._state_store.acquire_lock()
        self.guard=mock.patch.object(m.hashlib,'sha256',side_effect=auth_only_sha);self.guard.start()
        self.owned=[];self.reapers=[];self.resolved_outputs=[];self.bundle_resolved={}

    def tearDown(self):
        self.guard.stop()
        for p in self.owned:
            if p.poll() is None:
                ORIGINAL_KILLPG(p.pid,signal.SIGKILL)
            try:p.wait(timeout=3)
            finally:
                if p.stdout:p.stdout.close()
                if p.stderr:p.stderr.close()
                if p.stdin and not p.stdin.closed:p.stdin.close()
        for t in self.reapers:t.join(timeout=3)
        for resolved in self.resolved_outputs:resolved.close()
        if self.owner._state_store is not None:self.owner._state_store.close()
        for attr in ['_state_root','_input_root','_output_root']:
            if getattr(self.owner,attr) is not None:getattr(self.owner,attr).close()
        self.tmp.cleanup()

    def store_record(self,r,name=None):
        name=name or ('.nuwax-offline-cli-request-v1-'+'1'*64+'.json' if r['schema_version']==3 else self.owner._state_store.request_name(r['request_id']))
        self.owner._state_store.write_json(name,r)
        return name

    def observe(self):
        resolved=self.owner._input_root.resolve_input('input.jsonl')
        try:return resolved.observe_input()
        finally:resolved.close()

    def reader_bundle(self,request_id='request:original.1',mutator=None):
        obs=self.observe();r=record_for('map',request_id=request_id,observation=obs);cmd=r['command']
        resolved=self.owner._output_root.resolve_output(cmd['output_path']);self.resolved_outputs.append(resolved);self.bundle_resolved[r['request_id']]=resolved
        output=self.root/'output'/cmd['output_path'];output.mkdir()
        asset={'schema_version':'nuwax.mapping.offline-map-asset.v2','map_name':cmd['map_name'],'frame_id':'map','request_id':request_id,
               'source':dict(zip(('dataset_size','input_format','dataset_device','dataset_inode','dataset_mtime_ns','dataset_ctime_ns'),obs)),
               'grid':{'width':2,'height':2,'resolution_m':0.1,'origin_xy_yaw':[0.0,0.0,0.0],'encoding':'pgm-p5-cost-inverted-v1','path':'map.pgm'},
               'producer':{'role':'offline_mapping_cli','release_generation':'a'*64,'release_manifest_sha256':'b'*64,'artifact_sha256':'c'*64}}
        if mutator:mutator(asset)
        ab=m._canonical_json_bytes(asset);pgm=b'P5\n2 2\n255\n'+bytes([0,20,50,255]);(output/'asset.json').write_bytes(ab);(output/'map.pgm').write_bytes(pgm)
        manifest={'schema_version':'nuwax.mapping.offline-map-manifest.v2','map_name':cmd['map_name'],'frame_id':'map','request_id':request_id,'asset_path':'asset.json','files':[{'path':'asset.json','size':len(ab)},{'path':'map.pgm','size':len(pgm)}]}
        (output/'manifest.json').write_bytes(m._canonical_json_bytes(manifest))
        return r,output,manifest

    def read_bundle(self,r):
        resolved=self.bundle_resolved[r['request_id']]
        return m._inspect_map_bundle(resolved,r['command'],r['request_id'],r['dataset_size'],r['dataset_format'],r['dataset_device'],r['dataset_inode'],r['dataset_mtime_ns'],r['dataset_ctime_ns'],r['release_generation'],r['release_manifest_sha256'],r['offline_cli_artifact_sha256'],r['offline_cli_artifact_size'])

class AdmissionAndFiles(Fixture):
    def test_new_status_all_four_commands_no_business_digest(self):
        for kind in ['summary','rename','pose-fill','map']:
            r=record_for(kind);m._validate_record_shape(r);status=m.OfflineCliStatus.from_record(r)
            self.assertEqual(status.schema_version,4)
            self.assertTrue({'request_digest','dataset_sha256','output_sha256'}.isdisjoint(status.__dict__))

    def test_nonmap_success_status_and_authenticated_restart(self):
        for i,kind in enumerate(['summary','rename','pose-fill']):
            r=successful(record_for(kind,request_id='request%d'%i));name=self.store_record(r);read=self.owner._state_store.read_json(name)
            status=m.OfflineCliStatus.from_record(read);self.assertEqual(status.output_size,16);self.assertIsNone(status.receipt)

    def test_nonmap_failure_has_no_output_receipt(self):
        for kind in ['summary','rename','pose-fill']:
            r=failed(record_for(kind));m._validate_record_shape(r);r['output_size']=10
            with self.assertRaises(m.OfflineCliRecoveryError):m._validate_record_shape(r)

    def test_typed_request_map_echo_and_no_digest(self):
        req=m.OfflineCliRequest('request:original.1',command_for('map'),5)
        payload,argv,timeout=m._request_contract(req,release_generation='a'*64,release_manifest_sha256='b'*64,offline_cli_artifact_sha256='c'*64,offline_cli_artifact_size=12,dataset_size=1,dataset_format='jsonl',dataset_device=0,dataset_inode=0,dataset_mtime_ns=-1,dataset_ctime_ns=0)
        self.assertEqual(argv[-2:],['--request-id','request:original.1']);self.assertEqual(payload['dataset_device'],0);self.assertNotIn('request_digest',payload)

    def test_direct_original_request_filename(self):
        for request_id in ['a:b','A.Z_9-1','x'*128]:
            name=self.owner._state_store.request_name(request_id);self.assertEqual(name,'.nuwax-offline-cli-request-v2-'+request_id+'.json')
        self.assertEqual(len(name),163)
        for bad in ['x/y','x'*129,'-x','x\\y','']:
            with self.assertRaises(m.OfflineCliRecoveryError):self.owner._state_store.request_name(bad)

    def test_actual_input_observation_no_content_reads(self):
        resolved=self.owner._input_root.resolve_input('input.jsonl')
        try:
            with mock.patch.object(m.os,'read',side_effect=AssertionError('content read')):
                observed=resolved.observe_input();resolved.verify_input_observation(*observed)
            self.assertEqual(observed[2],(self.root/'input/input.jsonl').stat().st_dev)
        finally:resolved.close()

    def test_observed_dev_inode_zero_and_signed_time_bounds(self):
        resolved=self.owner._input_root.resolve_input('input.jsonl')
        try:
            resolved.metadata=types.SimpleNamespace(st_size=1,st_dev=0,st_ino=0,st_mtime_ns=-(1<<63),st_ctime_ns=(1<<63)-1)
            with mock.patch.object(resolved,'verify_current_identity'):
                self.assertEqual(resolved.observe_input(),(1,'jsonl',0,0,-(1<<63),(1<<63)-1))
                resolved.metadata.st_mtime_ns=-(1<<63)-1
                with self.assertRaises(m.OfflineCliValidationError):resolved.observe_input()
        finally:resolved.close()

    def test_input_changed_metadata_rejected(self):
        resolved=self.owner._input_root.resolve_input('input.jsonl')
        try:
            obs=resolved.observe_input();(self.root/'input/input.jsonl').write_bytes(b'changed')
            with self.assertRaises(m.OfflineCliValidationError):resolved.verify_input_observation(*obs)
        finally:resolved.close()

    def test_input_replaced_inode_rejected(self):
        resolved=self.owner._input_root.resolve_input('input.jsonl')
        try:
            obs=resolved.observe_input();p=self.root/'input/input.jsonl';p.rename(p.with_suffix('.old'));p.write_bytes(b'{"source":"real"}\n')
            with self.assertRaises(m.OfflineCliValidationError):resolved.verify_input_observation(*obs)
        finally:resolved.close()

    def test_output_observation_no_content_reads(self):
        resolved=self.owner._output_root.resolve_output('summary.json')
        (self.root/'output/summary.json').write_bytes(b'{"ok":true}')
        try:
            with mock.patch.object(m.os,'read',side_effect=AssertionError('content read')):self.assertEqual(m._inspect_output(resolved,'summary.json'),(11,None))
        finally:resolved.close()

    def test_sqlite_sidecar_still_rejected(self):
        resolved=self.owner._output_root.resolve_output('result.db')
        (self.root/'output/result.db').write_bytes(b'opaque-format-owned-by-rust');(self.root/'output/result.db-wal').write_bytes(b'residue')
        try:
            with self.assertRaises(m.OfflineCliOwnerError):m._inspect_output(resolved,'result.db')
        finally:resolved.close()

    def test_map_v2_real_reader_receipt_and_durable_validation(self):
        r,output,manifest=self.reader_bundle();size,receipt=self.read_bundle(r)
        r=successful(r);r.update(output_size=size,receipt=receipt);m._validate_record_shape(r)
        self.assertEqual(receipt['schema_version'],2);self.assertNotIn('manifest_sha256',receipt)
        name=self.store_record(r);self.assertEqual(self.owner._state_store.read_json(name),r)

    def test_map_missing_managed_request_id_rejected(self):
        r,output,manifest=self.reader_bundle(mutator=lambda a:a.pop('request_id'))
        with self.assertRaises(m.OfflineCliOwnerError):self.read_bundle(r)

    def test_map_wrong_request_id_rejected(self):
        r,output,manifest=self.reader_bundle(mutator=lambda a:a.update(request_id='another'))
        with self.assertRaises(m.OfflineCliOwnerError):self.read_bundle(r)

    def test_map_legacy_schema_rejected(self):
        r,output,manifest=self.reader_bundle();manifest['schema_version']='nuwax.mapping.offline-map-manifest.v1';(output/'manifest.json').write_bytes(m._canonical_json_bytes(manifest))
        with self.assertRaises(m.OfflineCliOwnerError):self.read_bundle(r)

    def test_map_changed_input_observation_rejected(self):
        r,output,manifest=self.reader_bundle(mutator=lambda a:a['source'].update(dataset_inode=999))
        with self.assertRaises(m.OfflineCliOwnerError):self.read_bundle(r)

    def test_map_size_bool_cannot_match_integer(self):
        r,output,manifest=self.reader_bundle();manifest['files'][0]['size']=True;(output/'manifest.json').write_bytes(m._canonical_json_bytes(manifest))
        with self.assertRaises(m.OfflineCliOwnerError):self.read_bundle(r)

    def test_map_extra_member_rejected(self):
        r,output,manifest=self.reader_bundle();(output/'extra').write_bytes(b'extra')
        with self.assertRaises(m.OfflineCliOwnerError):self.read_bundle(r)

    def test_map_pgm_truncated_rejected(self):
        r,output,manifest=self.reader_bundle();(output/'map.pgm').write_bytes(b'P5\n2 2\n255\n\x00')
        with self.assertRaises(m.OfflineCliOwnerError):self.read_bundle(r)

    def test_static_valid_raster_change_not_checksum_detected(self):
        r,output,manifest=self.reader_bundle();p=output/'map.pgm';v=p.read_bytes();p.write_bytes(v[:-1]+b'\x05')
        self.assertEqual(self.read_bundle(r)[1]['request_id'],r['request_id'])

    def test_signed_observation_and_zero_device_validate_new_only(self):
        r=record_for(observation=(1,'jsonl',0,0,-(1<<63),0));m._validate_record_shape(r)
        for field in ['dataset_device','dataset_inode','dataset_mtime_ns','dataset_ctime_ns']:
            bad=copy.deepcopy(r);bad[field]=True
            with self.assertRaises(m.OfflineCliRecoveryError):m._validate_record_shape(bad)

    def test_legacy_opaque_status_hmac_read_without_business_hash(self):
        r=successful(record_for(legacy=True));name=self.store_record(r);self.owner._index_request_records()
        self.assertEqual(self.owner._read_record_locked(r['request_id']),r);status=self.owner.get_status(r['request_id'])
        self.assertIsInstance(status,m.LegacyOfflineCliStatus);self.assertEqual(status.request_digest,'d'*64)

    def test_legacy_duplicate_request_identity_rejected(self):
        r=failed(record_for(legacy=True));self.store_record(r);self.store_record(failed(record_for()),name=self.owner._state_store.request_name(r['request_id']))
        with self.assertRaises(m.OfflineCliRecoveryError):self.owner._index_request_records()

    def test_mac_tamper_rejected(self):
        r=failed(record_for());name=self.store_record(r);p=self.root/'state'/name;envelope=json.loads(p.read_text());envelope['payload']['message']='tampered';p.write_bytes(m._canonical_json_bytes(envelope));p.chmod(0o600)
        with self.assertRaises(m.OfflineCliRecoveryError):self.owner._state_store.read_json(name)

    def test_legacy_start_rejected_before_missing_historical_input(self):
        r=failed(record_for(legacy=True));self.store_record(r);self.owner._index_request_records();(self.root/'input/input.jsonl').unlink()
        with self.assertRaisesRegex(m.OfflineCliRequestConflict,'legacy request'):
            self.owner.start(m.OfflineCliRequest(r['request_id'],command_for(),5))

    def test_new_same_typed_request_returns_original_time(self):
        r=successful(record_for(observation=self.observe()));self.store_record(r);self.owner._index_request_records()
        status=self.owner.start(m.OfflineCliRequest(r['request_id'],command_for(),5))
        self.assertEqual(status.submitted_at_unix_ns,10000000000);self.assertEqual(status.operation_id,'9'*32)

    def test_same_id_changed_timeout_conflicts(self):
        r=successful(record_for(observation=self.observe()));self.store_record(r);self.owner._index_request_records()
        with self.assertRaises(m.OfflineCliRequestConflict):self.owner.start(m.OfflineCliRequest(r['request_id'],command_for(),6))

    def test_new_start_records_direct_identity_before_recording_launch(self):
        observed=[]
        def no_launch(**kwargs):observed.append(kwargs);raise RuntimeError('recording launch boundary')
        with mock.patch.object(self.owner,'_verify_binary_identity'),mock.patch.object(self.owner,'_launch_locked',side_effect=no_launch):
            status=self.owner.start(m.OfflineCliRequest('direct:new',command_for('map'),5))
        self.assertTrue(status.terminal);self.assertEqual(observed[0]['argv'][-2:],['--request-id','direct:new']);self.assertEqual(status.schema_version,4)
        self.assertNotIn('request_digest',status.__dict__)

class RecoveryProcesses(Fixture):
    def spawn_waiter(self):
        p=subprocess.Popen([sys.executable,'-c','import time; print("ready",flush=True); time.sleep(60)'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,start_new_session=True)
        self.owned.append(p);self.assertEqual(p.stdout.readline(),b'ready\n');kind,info=m._probe_process(p.pid);self.assertIsNotNone(info)
        t=threading.Thread(target=p.wait,daemon=True);t.start();self.reapers.append(t)
        return p,info

    def dead_owner(self):
        p=subprocess.Popen([sys.executable,'-c','import sys; print("ready",flush=True); sys.stdin.read()'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,start_new_session=True)
        self.owned.append(p);self.assertEqual(p.stdout.readline(),b'ready\n');kind,info=m._probe_process(p.pid);self.assertIsNotNone(info);p.stdin.close();p.wait(timeout=3)
        return p.pid,info.start_token

    @contextlib.contextmanager
    def owned_signals(self):
        def checked(pgid,sig):
            if pgid not in {p.pid for p in self.owned}:raise AssertionError('signal outside test owned session')
            return ORIGINAL_KILLPG(pgid,sig)
        with mock.patch.object(m.os,'killpg',side_effect=checked):yield

    def running_record(self,legacy=True):
        p,info=self.spawn_waiter();r=record_for(legacy=legacy)
        r.update(state='running',started_at_unix_ns=11000000000,pid=p.pid,process_start_token=info.start_token,pgid=info.pgrp,sid=info.session)
        return p,r

    def active_for(self,r,old_owner):
        active={'schema_version':1 if r['schema_version']==3 else 2,'owner_pid':old_owner[0],'owner_start_token':old_owner[1],'request_id':r['request_id'],'operation_id':r['operation_id'],'pid':r['pid'],'process_start_token':r['process_start_token'],'pgid':r['pgid'],'sid':r['sid']}
        if r['schema_version']==3:active['request_digest']=r['request_digest']
        return active

    def test_legacy_active_owned_process_recovered_without_new_input_metadata(self):
        p,r=self.running_record();old=self.dead_owner();before=copy.deepcopy(r);name=self.store_record(r);self.owner._state_store.write_json(m.ACTIVE_JOURNAL_NAME,self.active_for(r,old));self.owner._index_request_records()
        with self.owned_signals():self.owner._recover_stale_active();self.owner._recover_orphaned_records()
        self.assertIsNotNone(p.poll());after=self.owner._state_store.read_json(name);self.assertEqual(after['state'],'failed');self.assertEqual(after['error_code'],'owner_crash_outcome_unknown')
        for key in set(before)-{'state','terminal','completed_at_unix_ns','error_code','message'}:self.assertEqual(after[key],before[key])
        self.assertNotIn('dataset_mtime_ns',after);self.assertIsNone(self.owner._state_store.read_json(m.ACTIVE_JOURNAL_NAME))

    def test_new_active_owned_process_recovery(self):
        p,r=self.running_record(legacy=False);old=self.dead_owner();name=self.store_record(r);self.owner._state_store.write_json(m.ACTIVE_JOURNAL_NAME,self.active_for(r,old));self.owner._index_request_records()
        with self.owned_signals():self.owner._recover_stale_active();self.owner._recover_orphaned_records()
        self.assertIsNotNone(p.poll());self.assertEqual(self.owner._state_store.read_json(name)['error_code'],'owner_crash_outcome_unknown')

    def test_legacy_orphan_accepted_settles_no_process(self):
        r=record_for(legacy=True);name=self.store_record(r);self.owner._index_request_records()
        with mock.patch.object(m.os,'killpg',side_effect=AssertionError('no process should be signalled')):self.owner._recover_orphaned_records()
        self.assertEqual(self.owner._state_store.read_json(name)['error_code'],'owner_crash_recovered')

    def test_legacy_orphan_running_only_owned_process_terminated(self):
        p,r=self.running_record();name=self.store_record(r);self.owner._index_request_records()
        with self.owned_signals():self.owner._recover_orphaned_records()
        self.assertIsNotNone(p.poll());self.assertEqual(self.owner._state_store.read_json(name)['error_code'],'owner_crash_recovered')

    def test_active_recovery_mutation_visible_no_second_termination(self):
        p,r=self.running_record();old=self.dead_owner();self.store_record(r);self.owner._state_store.write_json(m.ACTIVE_JOURNAL_NAME,self.active_for(r,old));self.owner._index_request_records()
        with self.owned_signals(),mock.patch.object(self.owner,'_terminate_identity',wraps=self.owner._terminate_identity) as terminate:
            self.owner._recover_stale_active();self.owner._recover_orphaned_records()
        self.assertEqual(terminate.call_count,1)

    def test_original_live_owner_guard_preserved(self):
        p,r=self.running_record();self.store_record(r);self.owner._state_store.write_json(m.ACTIVE_JOURNAL_NAME,self.active_for(r,(self.owner._owner_pid,self.owner._owner_start_token)));self.owner._index_request_records()
        with mock.patch.object(m.os,'killpg',side_effect=AssertionError('must not signal')):
            with self.assertRaisesRegex(m.OfflineCliRecoveryError,'still alive'):self.owner._recover_stale_active()
        self.assertIsNone(p.poll())

    def test_authenticated_active_identity_mismatch_no_signal(self):
        p,r=self.running_record();old=self.dead_owner();self.store_record(r);active=self.active_for(r,old);active['operation_id']='8'*32;self.owner._state_store.write_json(m.ACTIVE_JOURNAL_NAME,active);self.owner._index_request_records()
        with mock.patch.object(m.os,'killpg',side_effect=AssertionError('must not signal')):
            with self.assertRaisesRegex(m.OfflineCliRecoveryError,'disagree'):self.owner._recover_stale_active()
        self.assertIsNone(p.poll())

    def test_terminal_record_leftover_journal_no_terminal_rewrite(self):
        p,info=self.spawn_waiter();old=self.dead_owner();ORIGINAL_KILLPG(p.pid,signal.SIGTERM);p.wait(timeout=3)
        r=failed(record_for(legacy=True));r.update(pid=p.pid,process_start_token=info.start_token,pgid=info.pgrp,sid=info.session,started_at_unix_ns=11000000000);name=self.store_record(r)
        self.owner._state_store.write_json(m.ACTIVE_JOURNAL_NAME,self.active_for(r,old));self.owner._index_request_records();before=(self.root/'state'/name).read_bytes()
        with self.owned_signals():self.owner._recover_stale_active();self.owner._recover_orphaned_records()
        self.assertEqual((self.root/'state'/name).read_bytes(),before)

    def test_unverifiable_child_probe_no_signal(self):
        p,r=self.running_record();name=self.store_record(r);self.owner._index_request_records()
        with mock.patch.object(m,'_probe_process',return_value=(m._ProbeKind.UNVERIFIABLE,None)),mock.patch.object(m,'_group_members',return_value=[]),mock.patch.object(m.os,'killpg',side_effect=AssertionError('must not signal')):
            with self.assertRaises(m.OfflineCliRecoveryError):self.owner._recover_orphaned_records()
        self.assertIsNone(p.poll());self.assertFalse(self.owner._state_store.read_json(name)['terminal'])

    def test_pid_reuse_negative_fixture_no_signal(self):
        p,r=self.running_record();records_info=m._probe_process(p.pid)[1];other=m._ProcessInfo(p.pid,'S',p.pid,p.pid,records_info.start_token+1)
        with mock.patch.object(m,'_probe_process',return_value=(m._ProbeKind.RUNNING,other)),mock.patch.object(m,'_group_members',return_value=[other]),mock.patch.object(m.os,'killpg',side_effect=AssertionError('must not signal')):
            with self.assertRaisesRegex(m.OfflineCliRecoveryError,'reused'):self.owner._verified_group_members(pid=p.pid,process_start_token=records_info.start_token,pgid=p.pid,sid=p.pid)
        self.assertIsNone(p.poll())

class LifecycleProcesses(RecoveryProcesses):
    def launch_owned_monitor(self,timeout=False):
        p,info=self.spawn_waiter()
        resolved_input=self.owner._input_root.resolve_input('input.jsonl')
        observed=resolved_input.observe_input();r=record_for(observation=observed)
        r.update(state='running',started_at_unix_ns=11000000000,pid=p.pid,process_start_token=info.start_token,pgid=info.pgrp,sid=info.session)
        resolved_output=self.owner._output_root.resolve_output(r['output_relative_path'])
        alias=self.owner._state_store.create_input_alias(resolved_input,r['operation_id'])
        stdout=m._BoundedCapture(p.stdout,1024,'test-owned-stdout');stderr=m._BoundedCapture(p.stderr,1024,'test-owned-stderr');stdout.start();stderr.start()
        op=m._Operation(request_id=r['request_id'],record=r,input_alias_name=alias,resolved_input=resolved_input,resolved_output=resolved_output,process=p,process_start_token=info.start_token,pgid=info.pgrp,sid=info.session,monotonic_deadline=time.monotonic()+(.02 if timeout else 5),stdout_capture=stdout,stderr_capture=stderr)
        self.owner._write_record_locked(r);self.owner._state_store.write_json(m.ACTIVE_JOURNAL_NAME,self.active_for(r,(self.owner._owner_pid,self.owner._owner_start_token)))
        self.owner._active=op;self.owner._binary_fd=-1;self.owner._binary_parent_fd=-1
        op.monitor=threading.Thread(target=self.owner._monitor_operation,args=(op,),daemon=True);op.monitor.start()
        return op

    def test_owned_live_cancel_preserves_record_schema_and_process_fences(self):
        with self.owned_signals():
            op=self.launch_owned_monitor();status=self.owner.cancel(op.request_id);op.monitor.join(timeout=3)
        self.assertEqual(status.state,m.OfflineCliState.CANCELLED)
        self.assertIsNotNone(op.process.returncode);self.assertFalse(op.monitor.is_alive());self.assertIsNone(status.output_size);self.assertNotIn('output_sha256',status.__dict__)
        self.assertEqual(self.owner._state_store.list_input_aliases(),())
        self.assertEqual(self.owner.get_status(op.request_id).state,m.OfflineCliState.CANCELLED)

    def test_owned_deadline_timeout_preserves_original_termination(self):
        with self.owned_signals():
            op=self.launch_owned_monitor(timeout=True);status=self.owner.wait(op.request_id,timeout=3);op.monitor.join(timeout=3)
        self.assertEqual(status.state,m.OfflineCliState.TIMED_OUT);self.assertIsNotNone(op.process.returncode);self.assertFalse(op.monitor.is_alive())
        self.assertIsNone(self.owner._state_store.read_json(m.ACTIVE_JOURNAL_NAME))

    def test_owned_shutdown_preserves_original_cleanup_and_authenticated_terminal(self):
        with self.owned_signals():
            op=self.launch_owned_monitor();self.owner.shutdown()
        self.assertFalse(op.monitor.is_alive());self.assertIsNotNone(op.process.returncode);self.assertEqual(op.record['state'],'shutdown')
        self.assertTrue(self.owner._resources_closed);self.assertIsNone(self.owner._active)

def load_tests(loader, tests, pattern):
    return unittest.TestSuite([loader.loadTestsFromTestCase(AdmissionAndFiles),loader.loadTestsFromTestCase(RecoveryProcesses),unittest.TestSuite(LifecycleProcesses(name) for name in sorted(LifecycleProcesses.__dict__) if name.startswith('test_owned_'))])

if __name__=='__main__':
    selection=sys.argv[1] if len(sys.argv)>1 else 'A'
    cls={'A':AdmissionAndFiles,'B':RecoveryProcesses,'C':LifecycleProcesses}[selection]
    suite=(unittest.TestSuite(cls(name) for name in sorted(cls.__dict__) if name.startswith('test_owned_')) if selection=='C' else unittest.defaultTestLoader.loadTestsFromTestCase(cls))
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    out=Path(os.environ.get('NEH_CLI_IDENTITY_RESULTS_DIR',str(ROOT/('qualification_'+selection))));out.mkdir(exist_ok=True)
    (out/'RESULT.json').write_text(json.dumps({'python':sys.version,'module':m.__file__,'selection':selection,'tests_run':result.testsRun,'failures':len(result.failures),'errors':len(result.errors),'passed':result.wasSuccessful(),'actual_hmac_constructor_calls':len(AUTH_CALLS),'scope':'Complete module actual methods; A files/auth/admission with recording launch; B own-subprocess recovery and labelled mock negative probes; C actual own-subprocess monitor cancel/timeout/shutdown with captures/alias; not full released-binary owner startup'},indent=2)+'\n')
    sys.exit(not result.wasSuccessful())
