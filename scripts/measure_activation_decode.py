"""Run bounded native decoder arms in separate rotated processes.

Usage: python scripts/measure_activation_decode.py TEST_BINARY OUTPUT_DIRECTORY
The binary must contain the explicitly ignored cfg(test) diagnostic.
"""
import json, os, re, subprocess, sys, time
from pathlib import Path
binary=Path(sys.argv[1]).resolve()
root=Path(sys.argv[2]).resolve()
root.mkdir(parents=True, exist_ok=True)
runs=[]
selector='pointcloud::activation_decode_diagnostic_tests::assigned_activation_decode_pair_diagnostic'
for group in range(3):
    counts=[1000,100000,1000000]
    counts=counts[group:]+counts[:group]
    for count in counts:
        arms=['baseline','retained'] if (group+counts.index(count))%2==0 else ['retained','baseline']
        for arm in arms:
            name='g%d-n%d-%s'%(group,count,arm)
            env=dict(os.environ,NEH_ACTIVATION_DIAG_POINTS=str(count),NEH_ACTIVATION_DIAG_ARM=arm)
            with (root/(name+'.log')).open('wb') as log:
                start=time.monotonic()
                process=subprocess.Popen([str(binary),selector,'--ignored','--exact','--nocapture','--test-threads=1'],stdout=log,stderr=subprocess.STDOUT,env=env)
                pid,status,usage=os.wait4(process.pid,0)
                code=os.WEXITSTATUS(status) if os.WIFEXITED(status) else -os.WTERMSIG(status)
                process.returncode=code
                elapsed=time.monotonic()-start
            result={'group':group,'points':count,'arm':arm,'log':name+'.log','exit_code':code,'process_wall_s':elapsed,'process_user_s':usage.ru_utime,'process_system_s':usage.ru_stime,'process_maxrss_kib':usage.ru_maxrss}
            content=(root/(name+'.log')).read_text()
            result['decoder_ms']=[float(x) for x in re.findall(r'ACTIVATION_DECODE_TIMING .*? ms=([\d.]+)',content)]
            result['memory_lines']=[line for line in content.splitlines() if 'ACTIVATION_DECODE_MEMORY ' in line]
            result['control_lines']=[line for line in content.splitlines() if 'ACTIVATION_DECODE_CONTROL ' in line]
            runs.append(result)
            (root/'results.json').write_text(json.dumps({'scope':'Synthetic assigned canonical cloud helper diagnostic; no production change. Process wall includes fixture, memory sampling, arbitrary scratch and seven timing repetitions. Exact allocation call counts unavailable.','binary':str(binary),'binary_size':binary.stat().st_size,'runs':runs},indent=2)+'\n')
            print(name,code,'process_wall_s',elapsed,'decoder_ms',result['decoder_ms'],flush=True)
            if code or len(result['decoder_ms'])!=7:
                sys.exit(1)
