#!/usr/bin/env python3
"""Actual dependency orchestrator, synthetic sources, real CMake; no SDK."""
import argparse,hashlib,importlib.util,json,os,pathlib,shutil,subprocess,sys
from contextlib import ExitStack
from unittest.mock import patch
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--evidence-dir',type=pathlib.Path,required=True)
parser.add_argument('--cmake',default='cmake',help='Existing CMake executable; never downloaded')
args=parser.parse_args()
REPO=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(REPO))
sys.dont_write_bytecode=True
cmake=shutil.which(args.cmake)
if not cmake: parser.error('CMake unavailable; pass --cmake for an existing installation')
CMAKE=pathlib.Path(cmake).resolve()
OUT=args.evidence_dir.absolute();OUT.mkdir(parents=True,exist_ok=False)
no_hash_guard=ExitStack()
for name in ('sha256','sha1','md5','blake2b','blake2s','new'):
 no_hash_guard.enter_context(patch.object(hashlib,name,side_effect=AssertionError('application hash forbidden')))
CACHE=OUT/'cache'; SOURCE=REPO/'scripts/prepare_slam2d_dependencies.py'
candidate=SOURCE.read_text()
start=candidate.index('def _build_directory(')
end=candidate.index('def build_dependencies(',start)
original=candidate[:start]+candidate[end:]
assert original.count('        build_root = _build_directory(cache, name, source_root)\n')==1
original=original.replace('        build_root = _build_directory(cache, name, source_root)\n','        build_root = cache / "build" / name\n')
(OUT/'baseline.py').write_text(original);(OUT/'candidate.py').write_text(candidate)
for package in ('onetbb','ceres'):
 for pin,value in [('a',1),('b',2)]:
  root=CACHE/'sources'/('slam2d-'+package+'-'+pin);root.mkdir(parents=True)
  lib='fixture_'+package
  fail='if(DEFINED ENV{NEH_FAIL_CERES_CONFIGURE})\nmessage(FATAL_ERROR "fixture configure failure")\nendif()\n' if package=='ceres' else ''
  (root/'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.16)\nproject('+lib+' CXX)\n'+fail+'add_library('+lib+' SHARED library.cpp)\ninstall(TARGETS '+lib+' LIBRARY DESTINATION lib)\n')
  (root/'library.cpp').write_text('#ifdef NEH_FAIL_BUILD\n#error fixture compile failure\n#endif\nextern "C" int fixture_value(){return '+str(value)+';}\n')
  (root/'LICENSE').write_text('Literal '+package+' '+pin+' notice\n')
boost=OUT/'fake-boost';boost.mkdir();(boost/'LICENSE').write_text('Literal fake Boost notice\n')
env=dict(os.environ)
for key in ('CMAKE_PREFIX_PATH','CMAKE_TOOLCHAIN_FILE','CXXFLAGS','CFLAGS','LDFLAGS','LD_LIBRARY_PATH','LIBRARY_PATH'):env.pop(key,None)
versions={'slam2d-onetbb':'a','slam2d-ceres':'a'};build_failure=[False];calls=[]
def run(argv,**kwargs):
 argv=[str(x) for x in argv];assert argv[0]=='cmake',argv
 argv[0]=str(CMAKE)
 if '-S' in argv and 'slam2d-ceres' in argv[argv.index('-S')+1]:argv+=['-DCMAKE_CXX_FLAGS='+('-DNEH_FAIL_BUILD' if build_failure[0] else '')]
 p=subprocess.run(argv,env=env,capture_output=True,text=True)
 calls.append({'argv':argv,'exit':p.returncode})
 (OUT/('%03d.log'%len(calls))).write_text(repr(argv)+'\n'+p.stdout+p.stderr)
 if p.returncode:raise subprocess.CalledProcessError(p.returncode,argv,p.stdout,p.stderr)
def module(name):
 spec=importlib.util.spec_from_file_location('_fixture_'+name,OUT/(name+'.py'));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
 def checkout(cache,source,runner,role):
  assert source[1]==versions[role]*40
  return CACHE/'sources'/(role+'-'+versions[role])
 m._checkout=checkout
 m._boost=lambda prefix,cache,compiler,jobs,runner:boost
 return m
baseline=module('baseline');trial=module('candidate')
# A new or uninitialized cache uses the admitted source name without allocating an identity.
new_cache=OUT/'fresh-cache'
source=CACHE/'sources/slam2d-onetbb-a'
assert trial._build_directory(new_cache,'oneTBB',source)==new_cache/'build'/source.name
(new_cache/'build/oneTBB').mkdir(parents=True)
assert trial._build_directory(new_cache,'oneTBB',source)==new_cache/'build'/source.name
assert list((new_cache/'build/oneTBB').iterdir())==[]
def objects():return {str(p.relative_to(CACHE)):p.stat().st_mtime_ns for p in CACHE.rglob('*.o')}
def value(path):return subprocess.check_output([sys.executable,'-B','-c','import ctypes,sys;print(ctypes.CDLL(sys.argv[1]).fixture_value())',str(path)],env=env,text=True).strip()
rows=[]
def stage(name,mod,expected,success=True):
 mod.SOURCES={key:(url,versions[mod.SOURCE_ROLES[key]]*40) for key,(url,revision) in mod.SOURCES.items()}
 generation=OUT/'generations'/name;before=objects();start=len(calls)
 try:
  result=mod.build_dependencies(generation,CACHE,jobs=1,run=run);assert success;assert result==generation
 except subprocess.CalledProcessError as error:
  assert not success;(OUT/(name+'-error.txt')).write_text(error.stderr)
 after=objects();changed=[p for p in after if before.get(p)!=after[p]]
 row={'stage':name,'changed_objects':changed,'success':success,'commands':calls[start:]}
 if success:
  assert json.loads((generation/'share/nuwax-slam2d/来源.json').read_text())['sources']['ceres'][1]==versions['slam2d-ceres']*40
  row['runtime_values']={n:value(generation/'lib'/('libfixture_'+n+'.so')) for n in ('onetbb','ceres')}
  assert (generation/'share/nuwax-slam2d/0-LICENSE').read_text().endswith(versions['slam2d-onetbb']+' notice\n')
  assert (generation/'share/nuwax-slam2d/2-LICENSE').read_text().endswith(versions['slam2d-ceres']+' notice\n')
 rows.append(row);(OUT/'results.json').write_text(json.dumps(rows,indent=2)+'\n')
 assert len(changed)==expected,row
 print(name,len(changed),'success',success,row.get('runtime_values'),flush=True)
 return row
stage('baseline-a',baseline,2)
legacy=objects()
stage('candidate-a-repeat',trial,0)
assert objects()==legacy
versions['slam2d-ceres']='b'
stage('baseline-pin-change-fails',baseline,0,False)
stage('candidate-b-first',trial,1)
stage('candidate-b-repeat',trial,0)
versions['slam2d-ceres']='a'
stage('candidate-rollback-a',trial,0)
assert all(objects()[k]==v for k,v in legacy.items())
versions['slam2d-ceres']='b'
env['NEH_FAIL_CERES_CONFIGURE']='1';stage('candidate-configure-failure',trial,0,False);env.pop('NEH_FAIL_CERES_CONFIGURE')
stage('candidate-configure-retry',trial,0)
build_failure[0]=True;stage('candidate-build-failure',trial,0,False);build_failure[0]=False
stage('candidate-build-retry',trial,1)
assert value(OUT/'generations/baseline-a/lib/libfixture_ceres.so')=='1'
assert value(OUT/'generations/candidate-b-first/lib/libfixture_ceres.so')=='2'
assert all(objects()[k]==v for k,v in legacy.items())
print('PASS: actual build_dependencies baseline/candidate; checkout and Boost stubbed, real CMake only')
