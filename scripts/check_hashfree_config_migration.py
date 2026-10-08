"""Offline config/migration checks, targeting Python 3.8.

Run: python3 -B scripts/check_hashfree_config_migration.py
Optional history comparison: add --baseline /path/to/baseline_operations.py.
The baseline is an explicitly supplied trusted operations.py source capture, e.g.
from cb80236497088eb9d5358212dfe09b93cff53002. It is executed only when requested.
No services, network, real config, or artifact authenticity checks are exercised.
"""
import argparse
import importlib.util
import io
import json
import os
import stat
import sys
import unittest
from contextlib import ExitStack
from dataclasses import replace
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from scripts.deploy_installer import operations as current
from scripts.deploy_installer.model import DeploymentOptions

baseline = None
if __name__ == '__main__':
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--baseline', type=Path)
    options, remaining = parser.parse_known_args()
    sys.argv[1:] = remaining
    if options.baseline is not None:
        spec = importlib.util.spec_from_file_location(
            'scripts.deploy_installer._qualification_baseline', options.baseline)
        if spec is None or spec.loader is None:
            parser.error('--baseline must identify an importable Python source file')
        baseline = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(baseline)

MODULES = (current,) if baseline is None else (baseline, current)
LEGACY = '0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef.bak'
BLOCK = 1024 * 1024

class Contracts(unittest.TestCase):
    def setUp(self):
        self.tmp = TemporaryDirectory(prefix='neh-hashfree-config-')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def context(self, name='case', transaction='attempt-one'):
        options = replace(DeploymentOptions.for_tests(source_root=self.root / name),
                          transaction_id=transaction, no_systemd=True, no_verify=True)
        options.config_dir.mkdir(parents=True, exist_ok=True)
        return SimpleNamespace(options=options, notes={}, config_backups={}, previous_release=None,
                               release_switched=False,
                               runner=SimpleNamespace(run=lambda *a, **k: self.fail('external command forbidden')))

    def no_hash(self, module=current):
        stack = ExitStack()
        for name in ('sha256', 'sha1', 'md5', 'new', 'file_digest'):
            if hasattr(module.hashlib, name):
                stack.enter_context(patch.object(module.hashlib, name, side_effect=AssertionError('hash called: '+name)))
        stack.enter_context(patch.object(module, '_sha256', side_effect=AssertionError('_sha256 called')))
        return stack

    def recovery(self, module, context):
        module._record_recovery(context, context.options.release_dir, activated=False)
        loaded = self.context(context.options.source_root.name, context.options.transaction_id)
        self.assertTrue(module.load_persisted_recovery(loaded))
        return loaded

    def test_two_backups_manifest_metadata_repeat_restore_and_absent(self):
        ctx = self.context()
        paths = [ctx.options.config_dir / n for n in ('nuwax.env', 'go2-data-plane.env')]
        originals = [b'ONE=original\n', b'TWO=original\n']
        with self.no_hash():
            for i, (path, data) in enumerate(zip(paths, originals)):
                path.write_bytes(data)
                path.chmod(0o604 + i)
                current._backup_config(ctx, path)
            backup = current._transaction_backup_dir(ctx) / 'config'
            manifest = json.loads((backup / 'manifest.json').read_text())
            self.assertEqual(manifest['schema_version'], 1)
            self.assertEqual(set(manifest['files']), set(map(str, paths)))
            for path, data in zip(paths, originals):
                record = manifest['files'][str(path)]
                self.assertEqual(record['backup'], path.name + '.bak')
                self.assertTrue(record['existed'])
                self.assertEqual((backup / record['backup']).read_bytes(), data)
                s = path.stat()
                self.assertEqual(record['metadata'], dict(uid=s.st_uid, gid=s.st_gid, mode=stat.S_IMODE(s.st_mode)))
                self.assertEqual(stat.S_IMODE((backup / record['backup']).stat().st_mode), 0o600)
                path.write_bytes(b'CHANGED=yes\n')
                path.chmod(0o600)
                current._backup_config(ctx, path)
            self.assertEqual(json.loads((backup / 'manifest.json').read_text()), manifest)
            self.assertEqual(stat.S_IMODE(backup.parent.stat().st_mode), 0o700)
            self.assertEqual(stat.S_IMODE((backup/'manifest.json').stat().st_mode), 0o600)
            loaded = self.recovery(current, ctx)
            self.assertTrue(current.rollback(loaded))
            for path, data in zip(paths, originals):
                self.assertEqual(path.read_bytes(), data)
                s=path.stat()
                self.assertEqual(dict(uid=s.st_uid,gid=s.st_gid,mode=stat.S_IMODE(s.st_mode)), manifest['files'][str(path)]['metadata'])
            absent = self.context('absent')
            path = absent.options.config_dir/'nuwax.env'
            current._backup_config(absent,path)
            record=json.loads((current._transaction_backup_dir(absent)/'config/manifest.json').read_text())['files'][str(path)]
            self.assertEqual(record, dict(existed=False,backup=None,metadata=None))
            path.write_text('CREATED=yes\n')
            self.assertTrue(current.rollback(self.recovery(current,absent)))
            self.assertFalse(path.exists())

    def test_transaction_separation(self):
        a=self.context(transaction='one'); b=self.context(transaction='two')
        path=a.options.config_dir/'nuwax.env'
        with self.no_hash():
            path.write_bytes(b'A'); current._backup_config(a,path)
            path.write_bytes(b'B'); current._backup_config(b,path)
            for ctx,data in ((a,b'A'),(b,b'B')):
                self.assertEqual((current._transaction_backup_dir(ctx)/'config/nuwax.env.bak').read_bytes(),data)

    def test_literal_legacy_and_new_manifest_restore_both_implementations(self):
        for module in MODULES:
            for name in (LEGACY,'nuwax.env.bak'):
                with self.subTest(module=module.__name__,name=name):
                    ctx=self.context(module.__name__.split('.')[-1]+name)
                    path=ctx.options.config_dir/'nuwax.env'; path.write_bytes(b'NOW=changed\n')
                    root=module._prepare_backup_root(ctx)/'config'; root.mkdir()
                    (root/name).write_bytes(b'OLD=restored\n')
                    s=path.stat(); metadata=dict(uid=s.st_uid,gid=s.st_gid,mode=0o604)
                    (root/'manifest.json').write_text(json.dumps(dict(schema_version=1,files={str(path):dict(existed=True,backup=name,metadata=metadata)})))
                    with self.no_hash(module):
                        loaded=self.recovery(module,ctx)
                        self.assertEqual(loaded.config_backups[path],b'OLD=restored\n')
                        self.assertTrue(module.rollback(loaded))
                    self.assertEqual(path.read_bytes(),b'OLD=restored\n')
                    self.assertEqual(stat.S_IMODE(path.stat().st_mode),0o604)

    def test_content_cases_no_stat_cache(self):
        a=self.root/'a'; b=self.root/'b'
        with self.no_hash():
            for left,right in ((b'',b''),(b'',b'x'),(b'x',b''),(b'abc',b'abc'),(b'abc',b'ab'),(b'ab',b'abc')):
                a.write_bytes(left); b.write_bytes(right)
                self.assertEqual(current._same_file_contents(a,b),left==right)
            payload=b'a'*(BLOCK*2+13)
            a.write_bytes(payload); b.write_bytes(payload)
            self.assertTrue(current._same_file_contents(a,b))
            before=b.stat()
            os.utime(a,ns=(before.st_atime_ns,before.st_mtime_ns))
            for index in (0,BLOCK+17,len(payload)-1):
                changed=bytearray(payload); changed[index]=ord('z'); b.write_bytes(changed)
                os.utime(b,ns=(before.st_atime_ns,before.st_mtime_ns))
                self.assertEqual(b.stat().st_size,a.stat().st_size)
                self.assertEqual(b.stat().st_mtime_ns,a.stat().st_mtime_ns)
                self.assertFalse(current._same_file_contents(a,b))
                b.write_bytes(payload); os.utime(b,ns=(before.st_atime_ns,before.st_mtime_ns))
                self.assertTrue(current._same_file_contents(a,b))

    def test_bounded_paired_reads(self):
        calls=[]
        class Watched(io.BytesIO):
            def __init__(self,label): super().__init__(b'x'*(BLOCK*2+13)); self.label=label
            def read(self,size=-1):
                calls.append((self.label,size)); self_test.assertEqual(size,BLOCK)
                return super().read(size)
        self_test=self
        streams=[Watched('left'),Watched('right')]
        with self.no_hash(),patch.object(Path,'open',side_effect=streams):
            self.assertTrue(current._same_file_contents(self.root/'a',self.root/'b'))
        self.assertEqual(calls,[('left',BLOCK),('right',BLOCK)]*4)

    def test_io_errors_propagate(self):
        a=self.root/'a'; b=self.root/'b'; a.write_bytes(b'a'); b.write_bytes(b'a')
        with self.no_hash():
            with self.assertRaises(FileNotFoundError): current._same_file_contents(a,self.root/'absent')
            with patch.object(Path,'open',side_effect=PermissionError('denied')):
                with self.assertRaises(PermissionError): current._same_file_contents(a,b)
            class Broken(io.BytesIO):
                def read(self,size=-1): raise OSError('read failed')
            with patch.object(Path,'open',side_effect=[io.BytesIO(b'a'),Broken(b'a')]):
                with self.assertRaisesRegex(OSError,'read failed'): current._same_file_contents(a,b)
            with patch.object(current,'_same_file_contents',side_effect=OSError('compare failed')):
                with self.assertRaisesRegex(OSError,'compare failed'): current._copy_migration_source(a,b)
            self.assertEqual(b.read_bytes(),b'a')

    def migration_cases(self,module,root):
        root.mkdir(); source=root/'source'; destination=root/'destination'
        source.write_bytes(b'original'); source.chmod(0o604); os.utime(source,ns=(1000000000,2000000000))
        module._copy_migration_source(source,destination)
        self.assertEqual(destination.read_bytes(),b'original')
        self.assertEqual(stat.S_IMODE(destination.stat().st_mode),0o604)
        self.assertEqual(destination.stat().st_mtime_ns,source.stat().st_mtime_ns)
        module._copy_migration_source(source,destination)
        destination.write_bytes(b'conflict')
        with self.assertRaises(module.DeploymentError): module._copy_migration_source(source,destination)
        self.assertEqual(destination.read_bytes(),b'conflict')
        directory=root/'directory'; directory.mkdir()
        with self.assertRaises(module.DeploymentError): module._copy_migration_source(source,directory)
        module._copy_migration_source(root/'missing',root/'no-copy')
        self.assertFalse((root/'no-copy').exists())
        tree=root/'tree'; (tree/'nested').mkdir(parents=True); (tree/'empty').mkdir()
        (tree/'nested/data').write_bytes(b'tree'); target=root/'target'
        module._copy_migration_source(tree,target); module._copy_migration_source(tree,target)
        self.assertEqual((target/'nested/data').read_bytes(),b'tree'); self.assertTrue((target/'empty').is_dir())
        (target/'nested/data').write_bytes(b'changed')
        with self.assertRaises(module.DeploymentError): module._copy_migration_source(tree,target)
        self.assertEqual((target/'nested/data').read_bytes(),b'changed')
        for label,s,d in [('source-link',root/'source-link',root/'other'),('dest-link',source,root/'dest-link'),('parent-link',source,root/'parent-link/data')]:
            if label=='source-link': s.symlink_to(source)
            elif label=='dest-link': d.symlink_to(destination)
            else: (root/'parent-link').symlink_to(directory,target_is_directory=True)
            with self.subTest(label=label),self.assertRaises(module.DeploymentError): module._copy_migration_source(s,d)
        (tree/'nested/link').symlink_to(source)
        with self.assertRaises(module.DeploymentError): module._copy_migration_source(tree,root/'symlink-tree-target')
        # Deliberately no stronger claim about recursive destination nested symlinks.

    def test_migration_current_no_hash(self):
        with self.no_hash(): self.migration_cases(current,self.root/'current')

    @unittest.skipIf(baseline is None, 'historical comparison requires --baseline')
    def test_migration_baseline_same_outcomes(self):
        self.migration_cases(baseline,self.root/'baseline')

    @unittest.skipIf(baseline is None, 'historical comparison requires --baseline')
    def test_baseline_hash_probes_fail_expected(self):
        ctx=self.context(); path=ctx.options.config_dir/'nuwax.env'; path.write_bytes(b'x')
        dest=self.root/'dest'; dest.write_bytes(b'x')
        with self.no_hash(baseline):
            with self.assertRaisesRegex(AssertionError,'hash called'): baseline._backup_config(ctx,path)
            with self.assertRaisesRegex(AssertionError,'_sha256 called'): baseline._copy_migration_source(path,dest)
            tree=self.root/'tree'; tree.mkdir(); (tree/'data').write_bytes(b'x')
            out=self.root/'out'; out.mkdir(); (out/'data').write_bytes(b'x')
            with self.assertRaisesRegex(AssertionError,'_sha256 called'): baseline._copy_migration_source(tree,out)

    def test_existing_directory_config_treated_as_absent_unchanged(self):
        # Qualify the existing behavior, without introducing a new validation rule.
        for module in MODULES:
            ctx=self.context(module.__name__.split('.')[-1]+'directory-config')
            path=ctx.options.config_dir/'nuwax.env'; path.mkdir()
            module._backup_config(ctx,path)
            self.assertIsNone(ctx.config_backups[path])
            record=json.loads((module._transaction_backup_dir(ctx)/'config/manifest.json').read_text())['files'][str(path)]
            self.assertEqual(record,dict(existed=False,backup=None,metadata=None))
            self.assertTrue(path.is_dir())

    def test_migration_broken_symlinks_and_tree_destination_file(self):
        for module in MODULES:
            root=self.root/module.__name__.split('.')[-1]; root.mkdir()
            source=root/'source'; source.write_bytes(b'x')
            link=root/'broken'; link.symlink_to(root/'missing')
            with self.assertRaises(module.DeploymentError): module._copy_migration_source(link,root/'dest')
            with self.assertRaises(module.DeploymentError): module._copy_migration_source(source,link)
            tree=root/'tree'; tree.mkdir(); (tree/'data').write_bytes(b'x')
            with self.assertRaises(FileExistsError): module._copy_migration_source(tree,source)
            self.assertEqual(source.read_bytes(),b'x')

    def test_backup_symlink_guards_unchanged(self):
        for module in MODULES:
            for location in ('source','state','backups','transaction','config','file','manifest'):
                with self.subTest(module=module.__name__,location=location):
                    ctx=self.context(module.__name__.split('.')[-1]+location)
                    source=ctx.options.config_dir/'nuwax.env'; source.write_bytes(b'x')
                    root=module._transaction_backup_dir(ctx)
                    target=self.root/('target-'+module.__name__.split('.')[-1]+location)
                    if location in ('source','file','manifest'): target.write_bytes(b'untouched')
                    else: target.mkdir()
                    places={'source':source,'state':ctx.options.state_dir,'backups':ctx.options.state_dir/'backups','transaction':root,'config':root/'config','manifest':root/'config/manifest.json'}
                    if location=='file':
                        # Discover baseline name from its own persisted manifest, never calculate a new hash.
                        module._backup_config(ctx,source)
                        record=json.loads((root/'config/manifest.json').read_text())['files'][str(source)]
                        place=root/'config'/record['backup']; ctx.config_backups.clear()
                    else: place=places[location]
                    place.parent.mkdir(parents=True,exist_ok=True)
                    if place.exists(): place.unlink()
                    place.symlink_to(target,target_is_directory=target.is_dir())
                    with self.assertRaises(module.DeploymentError): module._backup_config(ctx,source)
                    if target.is_file(): self.assertEqual(target.read_bytes(),b'untouched')

if __name__=='__main__': unittest.main(verbosity=2)
