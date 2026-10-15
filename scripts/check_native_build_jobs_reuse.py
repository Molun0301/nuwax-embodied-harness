#!/usr/bin/env python3
"""Offline Cargo invalidation fixture; fake CMake, no native compilation."""
import argparse, json, os, pathlib, shutil, subprocess, sys
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--evidence-dir', type=pathlib.Path, required=True,
                    help='New persistent directory; existing evidence is never overwritten')
parser.add_argument('--cargo', default='cargo', help='Existing Cargo 1.88 executable')
args = parser.parse_args()
REPO = pathlib.Path(__file__).resolve().parents[1]
cargo = shutil.which(args.cargo)
if not cargo:
    parser.error('Cargo unavailable; pass --cargo pointing to an existing installation')
CARGO = pathlib.Path(cargo).resolve()
RUST = CARGO.parent
for program in (CARGO, RUST / 'rustc'):
    version = subprocess.check_output([str(program), '--version'], text=True).strip()
    if not version.startswith(program.name + ' 1.88.0 '):
        parser.error('This qualification requires existing Cargo/rustc 1.88.0: ' + version)
BASE = args.evidence_dir.absolute()
BASE.mkdir(parents=True, exist_ok=False)
FAKE = BASE / 'tools'
FAKE.mkdir(exist_ok=True)
(FAKE / 'cmake').write_text('''#!/usr/bin/env python3
import json, os, pathlib, sys
args=sys.argv[1:]
root=pathlib.Path.cwd()
phase='configure' if '-S' in args else 'build'
with pathlib.Path(os.environ['FIXTURE_COMMAND_LOG']).open('a') as f:
 f.write(json.dumps({'args':args,'phase':phase,'jobs_env':os.environ.get('CMAKE_BUILD_PARALLEL_LEVEL'),'makeflags':os.environ.get('MAKEFLAGS'),'cargo_makeflags':os.environ.get('CARGO_MAKEFLAGS'),'prefix':os.environ.get('CMAKE_PREFIX_PATH'),'toolchain':os.environ.get('CMAKE_TOOLCHAIN_FILE')})+'\\n')
if (root/'control'/('fail-'+phase)).exists():
 print('fixture injected '+phase+' failure',file=sys.stderr)
 sys.exit(29)
if phase=='configure':
 build=pathlib.Path(args[args.index('-B')+1]); build.mkdir(parents=True,exist_ok=True)
 (build/'gtsam-library.txt').write_text(str(build/'libfixture_gtsam.a')+'\\n')
 (build/'gtsam-openmp-libraries.txt').write_text('gomp=/fixture/libgomp.so\\n')
 (build/'CMakeCache.txt').write_text('CMAKE_GENERATOR:INTERNAL=Unix Makefiles\\n')
 (build/'pgo').write_text('fixture binary')
''')
(FAKE/'cmake').chmod(0o755)
SCRIPTS = {'slam': 'nuwax-slam-toolbox-sys', 'gtsam':'nuwax-gtsam', 'pgo':'nuwax-nav-pgo-bridge'}
for name, crate in SCRIPTS.items():
 original = (REPO / 'services/nuwax-workspace/crates' / crate / 'build.rs').read_text().replace('../../build_support/native_build.rs', 'native_build.rs')
 for version in ('baseline','candidate'):
  root=BASE/f'{name}-{version}'
  root.mkdir(exist_ok=True)
  for sub in ('src','native','control'): (root/sub).mkdir(exist_ok=True)
  (root/'Cargo.toml').write_text('[package]\nname = "jobs_fixture_'+name+'_'+version+'"\nversion = "0.1.0"\nedition = "2021"\n[workspace]\n')
  shutil.copyfile(REPO/'services/nuwax-workspace/build_support/native_build.rs', root/'native_build.rs')
  if name=='pgo':
   (root/'native/pgo').mkdir()
   (root/'shared-gtsam').mkdir()
   (root/'lcmtypes/nuwax_msgs').mkdir(parents=True)
   for message in ('NativePgoRequest', 'NativePgoResult', 'NativePgoStatus'):
    (root/'lcmtypes/nuwax_msgs'/(message+'.lcm')).touch()
  (root/'src/lib.rs').write_text('pub fn fixture() -> u32 { 1 }\n')
  (root/('native/pgo/input.cpp' if name=='pgo' else 'native/input.cpp')).write_text('// literal fixture; never compiled\n')
  needle='        "CMAKE_BUILD_PARALLEL_LEVEL",\n' if name in ('slam','pgo') else '    println!("cargo:rerun-if-env-changed=CMAKE_BUILD_PARALLEL_LEVEL");\n'
  anchor='        "CMAKE_TOOLCHAIN_FILE",\n' if name in ('slam','pgo') else '    println!("cargo:rerun-if-env-changed=CMAKE_TOOLCHAIN_FILE");\n'
  assert original.count(needle) in (0, 1)
  if needle in original:
   baseline=original
   candidate=original.replace(needle, '')
  else:
   candidate=original
   collapsed='    for key in ["CMAKE_PREFIX_PATH", "CMAKE_TOOLCHAIN_FILE"] {\n'
   if name=='slam' and collapsed in original:
    expanded='    for key in [\n        "CMAKE_PREFIX_PATH",\n        "CMAKE_TOOLCHAIN_FILE",\n'+needle+'    ] {\n'
    assert original.count(collapsed)==1
    baseline=original.replace(collapsed, expanded)
   else:
    assert original.count(anchor)==1
    baseline=original.replace(anchor, anchor+needle)
  text=baseline if version=='baseline' else candidate
  text=text.replace('../nuwax-gtsam/native', 'shared-gtsam').replace('../../lcmtypes/', 'lcmtypes/')
  (root/'build.rs').write_text(text)
  (root/'toolchain-a.cmake').write_text('# literal fixture; fake CMake does not read it\n')
  (root/'toolchain-b.cmake').write_text('# literal alternate toolchain\n')
results=[]
for name in SCRIPTS:
 for version in ('baseline','candidate'):
  root=BASE/f'{name}-{version}'
  command_log=root/'commands.jsonl'
  if command_log.exists(): raise RuntimeError('Use a new evidence root, do not overwrite execution logs')
  env=dict(os.environ)
  for key in tuple(env):
   if key.startswith(('CARGO_','RUST')) or key in ('CC','CXX','CMAKE_PREFIX_PATH','CMAKE_TOOLCHAIN_FILE','CMAKE_BUILD_PARALLEL_LEVEL'):
    env.pop(key,None)
  env.update(PATH=str(FAKE)+os.pathsep+str(RUST)+os.pathsep+env.get('PATH',''),RUSTC=str(RUST/'rustc'),RUSTDOC=str(RUST/'rustdoc'),CARGO_NET_OFFLINE='true',CARGO_TARGET_DIR=str(root/'target'),FIXTURE_COMMAND_LOG=str(command_log),CMAKE_BUILD_PARALLEL_LEVEL='2',CMAKE_PREFIX_PATH=str(root/'prefix-a'),CMAKE_TOOLCHAIN_FILE=str(root/'toolchain-a.cmake'))
  def run(label, expected, success=True):
   before=command_log.read_text().splitlines() if command_log.exists() else []
   command=[str(CARGO),'check','--offline','--jobs','1','--message-format=json']
   p=subprocess.run(command,cwd=root,env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
   (root/(label+'.stdout.jsonl')).write_text(p.stdout)
   (root/(label+'.stderr.log')).write_text(p.stderr)
   after=command_log.read_text().splitlines() if command_log.exists() else []
   calls=[json.loads(line) for line in after[len(before):]]
   row=dict(crate=name,version=version,stage=label,exit_code=p.returncode,commands=calls)
   results.append(row)
   (BASE/'results.json').write_text(json.dumps(results,indent=2)+'\n')
   assert (p.returncode==0)==success,(row,p.stderr)
   assert len(calls)==expected,row
   for call in calls:
    assert call['jobs_env']==(env['CMAKE_BUILD_PARALLEL_LEVEL'] if call['phase']=='configure' else None),row
    assert call['prefix']==env['CMAKE_PREFIX_PATH'],row
    assert call['toolchain']==env['CMAKE_TOOLCHAIN_FILE'],row
    if call['phase']=='build':
     assert '--parallel' not in call['args'],row
     assert call['makeflags']==call['cargo_makeflags'],row
    if name=='slam' and call['phase']=='configure':
     assert '-DNUWAX_SLAM2D_DEPENDENCY_PREFIX='+env.get('NUWAX_SLAM2D_DEPENDENCY_PREFIX','') in call['args'],row
   if expected==2:
    assert calls[0]['phase']=='configure' and calls[1]['phase']=='build',row
    assert '-DCMAKE_TOOLCHAIN_FILE='+env['CMAKE_TOOLCHAIN_FILE'] in calls[0]['args'],row
   print(name,version,label,'exit',p.returncode,'cmake_calls',len(calls),flush=True)
  def change_native(label):
   p=root/('native/pgo/input.cpp' if name=='pgo' else 'native/input.cpp'); p.write_text(p.read_text()+'// '+label+'\n')
  run('01-first',2)
  run('02-unchanged',0)
  env['CMAKE_BUILD_PARALLEL_LEVEL']='3'
  run('03-jobs-only',2 if version=='baseline' else 0)
  env['CMAKE_BUILD_PARALLEL_LEVEL']='4'; change_native('changed source')
  run('04-native-source',2); run('05-source-repeat',0)
  env['CMAKE_BUILD_PARALLEL_LEVEL']='5'; env['CMAKE_PREFIX_PATH']=str(root/'prefix-b')
  run('06-prefix',2); run('07-prefix-repeat',0)
  env['CMAKE_BUILD_PARALLEL_LEVEL']='6'; env['CMAKE_TOOLCHAIN_FILE']=str(root/'toolchain-b.cmake')
  run('08-toolchain',2); run('09-toolchain-repeat',0)
  env['CMAKE_BUILD_PARALLEL_LEVEL']='7'; change_native('configure failure')
  marker=root/'control/fail-configure'; marker.write_text('fail\n')
  run('10-configure-failure',1,False); marker.unlink()
  run('11-configure-retry',2)
  env['CMAKE_BUILD_PARALLEL_LEVEL']='8'; change_native('build failure')
  marker=root/'control/fail-build'; marker.write_text('fail\n')
  run('12-build-failure',2,False); marker.unlink()
  run('13-build-retry',2); run('14-final-repeat',0)
  if name=='slam':
   env['NUWAX_SLAM2D_DEPENDENCY_PREFIX']=str(root/'managed-generation')
   run('15-managed-prefix',2); run('16-managed-repeat',0)
   env.pop('NUWAX_SLAM2D_DEPENDENCY_PREFIX')
   run('17-unmanaged-transition',2); run('18-unmanaged-repeat',0)
print('PASS:',len(results),'Cargo invocations; fake CMake only; no SDK/object timing claims')
