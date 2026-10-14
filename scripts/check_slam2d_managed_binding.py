#!/usr/bin/env python3
"""Real CMake package-binding trial with literal same-version ABI changes."""
import argparse,hashlib,json,os,pathlib,shutil,subprocess,sys
from contextlib import ExitStack
from unittest.mock import patch
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--evidence-dir',type=pathlib.Path,required=True)
parser.add_argument('--cmake',default='cmake',help='Existing CMake executable')
args=parser.parse_args()
REPO=pathlib.Path(__file__).resolve().parents[1]
cmake=shutil.which(args.cmake)
if not cmake:parser.error('CMake unavailable; pass an existing executable via --cmake')
CMAKE=pathlib.Path(cmake).resolve()
BASE=args.evidence_dir.absolute();BASE.mkdir(parents=True,exist_ok=False)
no_hash_guard=ExitStack()
for name in ('sha256','sha1','md5','blake2b','blake2s','new'):
 no_hash_guard.enter_context(patch.object(hashlib,name,side_effect=AssertionError('application hash forbidden')))
SOURCE=REPO/'services/nuwax-workspace/crates/nuwax-slam-toolbox-sys/native/CMakeLists.txt'
candidate=SOURCE.read_text()
old='find_package(TBB REQUIRED)\nfind_package(Ceres 2.2 REQUIRED)\n'
new='''if(NUWAX_SLAM2D_DEPENDENCY_PREFIX)
  # Normal variables shadow stale cached package directories only in managed installs.
  set(TBB_DIR TBB_DIR-NOTFOUND)
  set(Ceres_DIR Ceres_DIR-NOTFOUND)
  find_package(TBB REQUIRED CONFIG PATHS "${NUWAX_SLAM2D_DEPENDENCY_PREFIX}" NO_DEFAULT_PATH)
  find_package(Ceres 2.2 REQUIRED CONFIG PATHS "${NUWAX_SLAM2D_DEPENDENCY_PREFIX}" NO_DEFAULT_PATH)
else()
  find_package(TBB REQUIRED)
  find_package(Ceres 2.2 REQUIRED)
endif()
'''
assert candidate.count(new)==1
original=candidate.replace(new,old)
(BASE/'baseline-native.cmake').write_text(original)
(BASE/'candidate-native.cmake').write_text(candidate)
base_fragment=original.split('add_library(nuwax_slam_toolbox SHARED')[0]
candidate_fragment=candidate.split('add_library(nuwax_slam_toolbox SHARED')[0]
consumer_tail='''add_executable(consumer main.cpp helper.cpp)
target_link_libraries(consumer PRIVATE TBB::tbb Ceres::ceres Boost::serialization)
set_target_properties(consumer PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
install(TARGETS consumer RUNTIME DESTINATION bin)
get_target_property(ceres_location Ceres::ceres IMPORTED_LOCATION)
get_target_property(tbb_location TBB::tbb IMPORTED_LOCATION)
get_target_property(boost_location Boost::serialization IMPORTED_LOCATION)
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/bindings.txt" "Ceres=${Ceres_DIR}\\nTBB=${TBB_DIR}\\nBoost=${Boost_DIR}\\nceres_library=${ceres_location}\\ntbb_library=${tbb_location}\\nboost_library=${boost_location}\\n")
'''
for name,fragment in [('baseline',base_fragment),('managed',candidate_fragment)]:
 root=BASE/(name+'-consumer');root.mkdir(exist_ok=True)
 (root/'CMakeLists.txt').write_text(fragment+consumer_tail)
 (root/'main.cpp').write_text('#ifdef FAIL_BUILD\n#error literal compile failure\n#endif\n#include <cstdio>\n#include <ceres_fixture.hpp>\n#include <tbb_fixture.hpp>\n#include <boost_fixture.hpp>\nint helper(); int main(){std::printf("%d\\n",CERES_CALL()+TBB_CALL()+fixture_boost()+helper());}\n')
 (root/'helper.cpp').write_text('int helper(){return 0;}\n')
common=BASE/'system-packages'
for name,version,target in [('Eigen3','3.3.0','Eigen3::Eigen'),('glog','0.6.0','glog::glog'),('gflags','2.2.2','gflags')]:
 d=common/'lib/cmake'/name;d.mkdir(parents=True,exist_ok=True)
 (d/(name+'Config.cmake')).write_text('if(NOT TARGET '+target+')\nadd_library('+target+' INTERFACE IMPORTED)\nendif()\nset('+name+'_VERSION '+version+')\n')
 (d/(name+'ConfigVersion.cmake')).write_text('set(PACKAGE_VERSION '+version+')\nset(PACKAGE_VERSION_COMPATIBLE TRUE)\nset(PACKAGE_VERSION_EXACT TRUE)\n')
for pin,multiplier in [('a',1),('b',2),('c',3)]:
 root=BASE/('dependency-'+pin);root.mkdir(exist_ok=True)
 text='cmake_minimum_required(VERSION 3.16)\nproject(tiny_dependencies CXX)\n'
 for package,factor in [('ceres',10),('tbb',1)]:
  title='Ceres' if package=='ceres' else 'TBB';target='Ceres::ceres' if package=='ceres' else 'TBB::tbb'
  text+='add_library(fixture_'+package+' SHARED '+package+'.cpp)\ninstall(TARGETS fixture_'+package+' LIBRARY DESTINATION lib)\ninstall(FILES '+package+'_fixture.hpp DESTINATION include)\ninstall(FILES '+title+'Config.cmake '+title+'ConfigVersion.cmake DESTINATION lib/cmake/'+title+')\n'
  (root/(package+'.cpp')).write_text('extern "C" int fixture_'+package+'_'+pin+'(){return '+str(factor*multiplier)+';}\n')
  (root/(package+'_fixture.hpp')).write_text('extern "C" int fixture_'+package+'_'+pin+'();\n#define '+package.upper()+'_CALL fixture_'+package+'_'+pin+'\n')
  transitive='include(CMakeFindDependencyMacro)\nfind_dependency(Threads)\nfind_dependency(Eigen3 3.3 NO_MODULE)\nfind_dependency(glog CONFIG)\nfind_dependency(gflags CONFIG)\n' if package=='ceres' else ''
  config=transitive+'get_filename_component(prefix "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)\nadd_library('+target+' SHARED IMPORTED)\nset_target_properties('+target+' PROPERTIES IMPORTED_LOCATION "${prefix}/lib/libfixture_'+package+'.so" INTERFACE_INCLUDE_DIRECTORIES "${prefix}/include")\n'
  if package=='ceres':config+='set_property(TARGET Ceres::ceres PROPERTY INTERFACE_LINK_LIBRARIES "Threads::Threads;Eigen3::Eigen;glog::glog;gflags")\n'
  (root/(title+'Config.cmake')).write_text(config)
  (root/(title+'ConfigVersion.cmake')).write_text('set(PACKAGE_VERSION 2.2.0)\nset(PACKAGE_VERSION_COMPATIBLE TRUE)\nset(PACKAGE_VERSION_EXACT TRUE)\n')
 (root/'CMakeLists.txt').write_text(text)
boost=BASE/'boost-source';boost.mkdir(exist_ok=True)
(boost/'boost.cpp').write_text('extern "C" int fixture_boost(){return 7;}\n')
(boost/'boost_fixture.hpp').write_text('extern "C" int fixture_boost();\n')
(boost/'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.16)\nproject(unchanged_boost CXX)\nadd_library(fixture_boost SHARED boost.cpp)\ninstall(TARGETS fixture_boost LIBRARY DESTINATION lib)\ninstall(FILES boost_fixture.hpp DESTINATION include)\ninstall(FILES BoostConfig.cmake BoostConfigVersion.cmake DESTINATION lib/cmake/Boost)\n')
(boost/'BoostConfig.cmake').write_text('get_filename_component(prefix "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)\nadd_library(Boost::serialization SHARED IMPORTED)\nset_target_properties(Boost::serialization PROPERTIES IMPORTED_LOCATION "${prefix}/lib/libfixture_boost.so" INTERFACE_INCLUDE_DIRECTORIES "${prefix}/include")\nadd_library(Boost::filesystem INTERFACE IMPORTED)\nadd_library(Boost::thread INTERFACE IMPORTED)\nset(Boost_VERSION 1.83.0)\nset(Boost_serialization_FOUND TRUE)\nset(Boost_filesystem_FOUND TRUE)\nset(Boost_thread_FOUND TRUE)\n')
(boost/'BoostConfigVersion.cmake').write_text('set(PACKAGE_VERSION 1.83.0)\nset(PACKAGE_VERSION_COMPATIBLE TRUE)\nset(PACKAGE_VERSION_EXACT TRUE)\n')
OUT=BASE/'evidence';OUT.mkdir(exist_ok=False)
env=dict(os.environ)
for key in ('CMAKE_PREFIX_PATH','CMAKE_TOOLCHAIN_FILE','CXXFLAGS','CFLAGS','LDFLAGS','LD_LIBRARY_PATH','LIBRARY_PATH'):env.pop(key,None)
seq=[0];results=[]
def run(args,success=True):
 seq[0]+=1;p=subprocess.run([str(x) for x in args],env=env,capture_output=True,text=True)
 (OUT/('%03d.log'%seq[0])).write_text(repr([str(x) for x in args])+'\n'+p.stdout+p.stderr)
 assert (p.returncode==0)==success,(args,p.returncode,p.stdout,p.stderr)
 return p
for pin in ('a','b','c'):
 run([CMAKE,'-S',BASE/('dependency-'+pin),'-B',BASE/('dependency-build-'+pin),'-DCMAKE_BUILD_TYPE=Release'])
 run([CMAKE,'--build',BASE/('dependency-build-'+pin),'--parallel','1'])
run([CMAKE,'-S',boost,'-B',BASE/'boost-build','-DCMAKE_BUILD_TYPE=Release'])
run([CMAKE,'--build',BASE/'boost-build','--parallel','1'])
def seed(prefix,pin):
 run([CMAKE,'--install',BASE/('dependency-build-'+pin),'--prefix',prefix])
 run([CMAKE,'--install',BASE/'boost-build','--prefix',prefix])
def objs(build):return {str(p.relative_to(build)):p.stat().st_mtime_ns for p in build.rglob('*.o')} if build.exists() else {}
def stage(label,variant,prefix,managed,install,expected,run_success=True,configure_success=True,build_success=True,explicit=None,flags=''):
 build=BASE/(variant+'-build');before=objs(build)
 command=[CMAKE,'-S',BASE/(variant+'-consumer'),'-B',build,'-DCMAKE_BUILD_TYPE=Release','-DCMAKE_PREFIX_PATH='+str(prefix)+';'+str(common),'-DCMAKE_INSTALL_PREFIX='+str(install),'-DNUWAX_SLAM2D_DEPENDENCY_PREFIX='+str(managed),'-DCMAKE_CXX_FLAGS='+flags]
 if explicit:
  command+=['-DCeres_DIR='+str(explicit/'lib/cmake/Ceres'),'-DTBB_DIR='+str(explicit/'lib/cmake/TBB')]
 p=run(command,configure_success)
 row={'stage':label,'variant':variant,'configure_exit':p.returncode}
 if not configure_success:
  assert 'Ceres' in p.stderr
  row['configure_error']=p.stderr
 if configure_success:
  row['bindings']=(build/'bindings.txt').read_text()
  p=run([CMAKE,'--build',build,'--parallel','1'],build_success);row['build_exit']=p.returncode
  changed=[k for k,v in objs(build).items() if before.get(k)!=v];row['objects_changed']=changed
  if expected is not None:assert len(changed)==expected,row
  if build_success:
   run([CMAKE,'--install',build])
   p=run([install/'bin/consumer'],run_success);row['runtime_exit']=p.returncode;row['runtime_output']=p.stdout.strip();row['runtime_error']=p.stderr.strip()
 results.append(row);(OUT/'results.json').write_text(json.dumps(results,indent=2)+'\n')
 print(label,row.get('objects_changed'),row.get('runtime_output'),row['configure_exit'],flush=True)
 return row
ba=BASE/'baseline-generations/a';bb=BASE/'baseline-generations/b';seed(ba,'a');seed(bb,'b')
assert stage('baseline-a','baseline',ba,'',ba,2)['runtime_output']=='18'
row=stage('baseline-same-version-b','baseline',bb,'',bb,0,run_success=False)
assert str(ba/'lib/cmake/Ceres') in row['bindings'] and 'undefined symbol' in row['runtime_error']
a=BASE/'managed-generations/a';b=BASE/'managed-generations/b';c=BASE/'managed-generations/custom';seed(a,'a');seed(b,'b');seed(c,'c')
assert (a/'lib/libfixture_boost.so').read_bytes()==(b/'lib/libfixture_boost.so').read_bytes()
assert stage('managed-a','managed',a,a,a,2)['runtime_output']=='18'
a_binary=(a/'bin/consumer').read_bytes();a_time=(a/'bin/consumer').stat().st_mtime_ns
row=stage('managed-same-version-b','managed',b,b,b,2)
assert row['runtime_output']=='29' and str(b/'lib/cmake/Ceres') in row['bindings'] and str(b/'lib/cmake/TBB') in row['bindings']
assert str(a/'lib/cmake/Boost') in row['bindings']
assert stage('managed-b-repeat','managed',b,b,b,0)['runtime_output']=='29'
missing=BASE/'managed-generations/missing-ceres'
for rel in ('lib/libfixture_tbb.so','include/tbb_fixture.hpp'):
 target=missing/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(b/rel,target)
shutil.copytree(b/'lib/cmake/TBB',missing/'lib/cmake/TBB')
stage('managed-missing-ceres','managed',b,missing,missing,None,configure_success=False)
assert stage('managed-configure-retry','managed',b,b,b,0)['runtime_output']=='29'
stage('managed-compile-failure','managed',b,b,b,None,build_success=False,flags='-DFAIL_BUILD')
assert run([b/'bin/consumer']).stdout.strip()=='29'
assert stage('managed-build-retry','managed',b,b,b,None)['runtime_output']=='29'
rollback=BASE/'managed-generations/rollback-a';seed(rollback,'a')
assert stage('managed-rollback-a','managed',a,a,rollback,2)['runtime_output']=='18'
assert (a/'bin/consumer').read_bytes()==a_binary and (a/'bin/consumer').stat().st_mtime_ns==a_time
assert run([a/'bin/consumer']).stdout.strip()=='18'
assert stage('unmanaged-explicit-custom','managed',b,'',c,2,explicit=c)['runtime_output']=='40'
assert stage('unmanaged-custom-repeat','managed',b,'',c,0,explicit=c)['runtime_output']=='40'
assert stage('return-to-managed-b','managed',b,b,b,2)['runtime_output']=='29'
print('PASS: managed same-version ABI binding, absent package failure, retries, rollback, explicit unmanaged overrides, unchanged Boost copies')
