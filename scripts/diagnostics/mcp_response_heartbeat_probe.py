import asyncio, contextvars, inspect, json, statistics, sys, threading, time, unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch, Mock
ROOT=Path(__file__).resolve().parents[2]; sys.path.insert(0,str(ROOT))
from gateways.mcp import protocol
from starlette.requests import Request
from starlette.responses import JSONResponse
from scripts.check_mcp_response_encoding import McpResponseEncodingTests
source=inspect.getsource(protocol.build_mcp_router)
old='''                response_payload = _jsonrpc_result(
                    request_id,
                    await run_in_threadpool(
                        lambda: build_mcp_tool_result(structured, max_total_bytes=result_budget)
                    ),
                )
                response = JSONResponse(response_payload)'''
new='''                response = await run_in_threadpool(
                    lambda: JSONResponse(_jsonrpc_result(request_id,
                        build_mcp_tool_result(structured, max_total_bytes=result_budget)))
                )'''
assert source.count(old)==1
ns={}; exec(source.replace(old,new),protocol.__dict__,ns); candidate=ns['build_mcp_router']; baseline=protocol.build_mcp_router
CTX=contextvars.ContextVar('probe_context',default='missing')
def req(method):
    data={'jsonrpc':'2.0','id':'fixture','method':method,'params':{'name':'offline_tool','arguments':{}} if method=='tools/call' else {}}
    raw=json.dumps(data).encode()
    async def receive(): return {'type':'http.request','body':raw,'more_body':False}
    return Request({'type':'http','http_version':'1.1','method':'POST','scheme':'http','path':'/mcp','query_string':b'','headers':[], 'server':('offline',80),'client':('127.0.0.1',123),'state':{'request_id':'offline-request'}},receive)
def endpoint(factory,result):
    rt=SimpleNamespace(invoke_tool=Mock(return_value=result),config=SimpleNamespace(gateway=SimpleNamespace(public_base_url='http://offline')))
    return next(r.endpoint for r in factory(rt,SimpleNamespace()).routes if 'POST' in r.methods)

async def sample(factory,value):
    renders=[]; heart=[]; init=[]; done=False; loop_thread=threading.get_ident()
    class TracedResponse(JSONResponse):
        def render(self,content):
            start=time.perf_counter(); out=super().render(content); end=time.perf_counter()
            if isinstance(content.get('result'),dict) and 'structuredContent' in content['result']:
                renders.append({'ms':(end-start)*1000,'on_loop':threading.get_ident()==loop_thread,'context':CTX.get()})
            return out
    call=endpoint(factory,value)
    async def heartbeat():
        previous=time.perf_counter()
        while not done:
            await asyncio.sleep(.001); now=time.perf_counter(); heart.append((now-previous)*1000); previous=now
    async def independent():
        while not done:
            target=time.perf_counter()+.002; await asyncio.sleep(.002); start=time.perf_counter(); response=await call(req('initialize')); end=time.perf_counter()
            assert response.status_code==200
            init.append({'handler_ms':(end-start)*1000,'scheduled_ms':(end-target)*1000})
    with patch.object(protocol,'_resolve_tool_invocation',return_value=('offline_tool',{})), patch.object(protocol,'JSONResponse',TracedResponse):
        h=asyncio.create_task(heartbeat()); i=asyncio.create_task(independent()); await asyncio.sleep(.005)
        start=time.perf_counter(); response=await call(req('tools/call')); elapsed=(time.perf_counter()-start)*1000
        await asyncio.sleep(.005); done=True; await asyncio.gather(h,i)
    return response,{'handler_ms':elapsed,'bytes':len(response.body),'max_heartbeat_gap_ms':max(heart),'max_initialize_handler_ms':max(x['handler_ms'] for x in init),'max_initialize_scheduled_ms':max(x['scheduled_ms'] for x in init),'renders':renders,'heartbeat_gaps_ms':heart,'initialize_samples':init}

async def cancellation(factory):
    entered=threading.Event(); release=threading.Event(); finished=threading.Event(); contexts=[]
    original=protocol.build_mcp_tool_result
    def blocked(*a,**kw):
        contexts.append(CTX.get()); entered.set(); release.wait(2)
        try: return original(*a,**kw)
        finally: finished.set()
    call=endpoint(factory,{'result':{'text':'small'}})
    with patch.object(protocol,'_resolve_tool_invocation',return_value=('offline_tool',{})),patch.object(protocol,'build_mcp_tool_result',side_effect=blocked):
        task=asyncio.create_task(call(req('tools/call')))
        while not entered.is_set(): await asyncio.sleep(.001)
        task.cancel(); await asyncio.sleep(.002); before=task.done(); release.set()
        try: await task
        except asyncio.CancelledError: pass
        while not finished.is_set(): await asyncio.sleep(.001)
    return {'cancelled':task.cancelled(),'done_before_release':before,'projection_finished':finished.is_set(),'contexts':contexts}

async def serialization_counts():
 rows=[]
 for size in [2048,1048576]:
  calls=[]; original=json.dumps; loop_thread=threading.get_ident()
  def record(value,*args,**kw):
   encoded=original(value,*args,**kw)
   if isinstance(value,dict) and value.get('jsonrpc')=='2.0': category='rpc_final' if value.get('result') is not None else 'rpc_empty_budget'
   elif isinstance(value,dict) and 'structuredContent' in value: category='tool_envelope_size_check'
   elif isinstance(value,dict) and 'result' in value: category='public_data_text'
   else: category='scalar_or_key_size_check'
   calls.append({'category':category,'utf8_bytes':len(encoded.encode('utf-8')),'on_loop':threading.get_ident()==loop_thread})
   return encoded
  value={'result':{'observation_context':{'camera_id':'front','metadata':{'notes':'x'*(size//2)}}}}
  call=endpoint(baseline,value); request=req('tools/call')
  with patch.object(protocol,'_resolve_tool_invocation',return_value=('offline_tool',{})),patch.object(json,'dumps',side_effect=record): response=await call(request)
  rows.append({'target_wire_bytes':size,'response_bytes':len(response.body),'calls':calls})
 return rows

async def main():
    CTX.set('context-preserved'); rows=[]
    # Text results are explicit capacity fixtures, never asserted production distributions.
    # Projection duplicates data into text and structuredContent, so ASCII payload is approximately half wire size.
    for target in [2048,1048576,4194304,8388608]:
        value={'result':{'observation_context':{'camera_id':'front','metadata':{'notes':'x'*(target//2)}}}}
        # Warm the worker and both handler arms before retained measurements.
        for factory in [baseline,candidate]: await sample(factory,{'result':{'text':'warm'}})
        expected=None
        for pair in range(5):
            for label,factory in ([('baseline',baseline),('candidate',candidate)] if pair%2==0 else [('candidate',candidate),('baseline',baseline)]):
                response,row=await sample(factory,value)
                signature=(response.status_code,response.raw_headers,response.body)
                if expected is None: expected=signature
                assert signature==expected
                assert row['renders'][0]['context']=='context-preserved'
                row.update(target_wire_bytes=target,arm=label,pair=pair,exact_response_equal=True); rows.append(row)
    cancels={label:await cancellation(factory) for label,factory in [('baseline',baseline),('candidate',candidate)]}; assert cancels['baseline']==cancels['candidate']
    output={'python':sys.version,'capacity_fixtures_only':True,'rows':rows,'cancellation':cancels,'serialization_counts':await serialization_counts(),'contract_tests_per_arm':9}
    Path(sys.argv[1]).write_text(json.dumps(output,indent=2))
    for size in [2048,1048576,4194304,8388608]:
        for arm in ['baseline','candidate']:
            subset=[r for r in rows if r['target_wire_bytes']==size and r['arm']==arm]
            print(size,arm,{key:statistics.median(r[key] for r in subset) for key in ['bytes','handler_ms','max_heartbeat_gap_ms','max_initialize_handler_ms','max_initialize_scheduled_ms']},flush=True)
    print('cancellation',cancels)

if __name__=='__main__':
    for label,factory in [('baseline',baseline),('candidate',candidate)]:
        with patch.object(protocol,'build_mcp_router',factory):
            result=unittest.TextTestRunner().run(unittest.defaultTestLoader.loadTestsFromTestCase(McpResponseEncodingTests)); assert result.wasSuccessful()
        print(label,'contracts',result.testsRun,flush=True)
    asyncio.run(main())
