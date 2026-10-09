"""Same-read ownership, stable-tree races, and unchanged ordinary borrowed admission."""
import gc
import os
import pickle
import struct
import unittest
from dataclasses import replace
from contextlib import ExitStack
from pathlib import Path
from unittest.mock import patch

import test_assigned_workspace_activation as workspace_tests
from test_checkpoint_identity_v2 import response_fixture, fine_bytes, dense_bytes
from services.geometry import geometry_runtime_provider as provider_module
from services.geometry import mapping_backend_checkpoint_store as checkpoint
from services.geometry.checkpoint_payload import FrozenPayload
from services.mapping import asset_store
from skills.assigned_map_activation import activate_assigned_workspace


class RetainedActivationReadTests(unittest.TestCase):
    def setUp(self):
        exported = provider_module._checkpoint_export_bundle_from_proto(response_fixture(),
            fine_payload=fine_bytes(), dense_payload=dense_bytes())
        self.h = workspace_tests.AssignedWorkspaceActivationTests()
        with patch.object(workspace_tests, 'checkpoint_fixture', return_value=(
                exported.checkpoint, exported.occupancy_asset, exported.cloud_asset)):
            self.h.setUp()
        self.addCleanup(self.h.doCleanups)
        self.repository = self.h.service._asset_repository
        self.root = self.repository.current_directory(workspace_tests.MAP_NAME)

    def read(self):
        return self.h.service.get_map_asset_bundle(map_name=workspace_tests.MAP_NAME, _for_activation=True)

    def _assert_one_owned_validation(self, operation):
        counts = dict.fromkeys(('backend', 'canonical', 'fine', 'dense', 'freeze', 'load'), 0)
        with ExitStack() as stack:
            for name, label in (('_validate_backend_payload_header', 'backend'), ('_validate_canonical_members', 'canonical'),
                                ('validate_fine_archive', 'fine'), ('validate_dense_archive', 'dense')):
                original = getattr(checkpoint, name)
                def wrap(*args, _original=original, _label=label, **kwargs):
                    counts[_label] += 1
                    return _original(*args, **kwargs)
                stack.enter_context(patch.object(checkpoint, name, wrap))
            for cls, name, label in ((FrozenPayload, '__init__', 'freeze'),
                                     (checkpoint.MappingBackendCheckpointStore, 'load', 'load')):
                original = getattr(cls, name)
                def wrap(*args, _original=original, _label=label, **kwargs):
                    counts[_label] += 1
                    return _original(*args, **kwargs)
                stack.enter_context(patch.object(cls, name, wrap))
            stack.enter_context(patch.object(self.repository, '_validate_assigned_checkpoint_bodies',
                side_effect=AssertionError('borrowed body pass during activation')))
            workspace = operation()
        self.h.assert_consistent(workspace)
        self.assertEqual(counts, dict(backend=1, canonical=1, fine=1, dense=1, freeze=3, load=1))
        self.assertEqual(self.h.stub.installs, 1)
        return workspace

    def test_public_activation_one_owned_validation_no_borrowed_pass(self):
        self._assert_one_owned_validation(self.h.activate)

    def test_tree_closed_owners_retained_for_canonical_view_and_upload(self):
        closed=[]
        original=asset_store._StableDirectoryTree.close
        def close(tree):
            descriptors=[tree.parent_descriptor]+[x.descriptor for x in tree.files.values()]
            original(tree)
            closed.extend(descriptors)
        with patch.object(asset_store._StableDirectoryTree,'close',close):
            read=self.read()
        for fd in closed:
            if fd>=0:
                with self.assertRaises(OSError):os.fstat(fd)
        self.assertEqual(bytes(read.stored_checkpoint.checkpoint.canonical_cloud_xyz.view()),struct.pack('<fff',1,2,3))
        with patch.object(checkpoint.MappingBackendCheckpointStore,'load',side_effect=AssertionError('second load')):
            prepared=self.h.runtime.prepare_retained_assigned_map_activation(read,repository=self.repository,bundle_root=self.root)
            self.assertIs(prepared.stored_checkpoint,read.stored_checkpoint)
            result=activate_assigned_workspace(self.h.service,read.bundle,_activation_read=read,load_memory_history=True)
        self.h.assert_consistent(result)
        self.assertIn('Transfer:2',self.h.stub.calls)
        self.assertIn('Transfer:5',self.h.stub.calls)

    def test_ordinary_getter_still_borrowed_without_freeze_or_load(self):
        with patch.object(FrozenPayload,'__init__',side_effect=AssertionError('ordinary freeze')), \
                patch.object(checkpoint.MappingBackendCheckpointStore,'load',side_effect=AssertionError('ordinary load')):
            result=self.h.service.get_map_asset_bundle(map_name=workspace_tests.MAP_NAME)
        self.assertEqual(result.asset_identity,workspace_tests.ASSET_ID)

    def test_owner_root_bundle_mismatch_and_token_reconstruction_rejected(self):
        read=self.read()
        for repository,root in ((asset_store.MapAssetBundleRepository(str(self.h.root)),self.root),
                                (self.repository,self.root.parent)):
            with self.assertRaises(asset_store.MapAssetBundleValidationError):
                self.h.runtime.prepare_retained_assigned_map_activation(read,repository=repository,bundle_root=root)
        with self.assertRaises(TypeError):
            activate_assigned_workspace(self.h.service,read.bundle.model_copy(),_activation_read=read,load_memory_history=True)
        with self.assertRaises(TypeError):replace(read,root=self.root.parent)
        with self.assertRaises(TypeError):pickle.dumps(read)
        self.assertEqual(self.h.stub.installs,0)

    def _race(self,when,kind,operation=None):
        original=checkpoint.MappingBackendCheckpointStore.load;loaded=[]
        def mutate():
            if kind=='file':
                path=next((self.root/'mapping_backend/voxel/objects').glob('*-backend.bin'))
                body=bytearray(path.read_bytes());body[-8:]=struct.pack('<d',4.0)
                replacement=path.with_suffix('.replacement');replacement.write_bytes(body)
                replacement.chmod(path.stat().st_mode & 0o777);os.replace(replacement,path)
            else:
                previous=self.root.with_name('replaced-root');self.root.rename(previous);self.root.mkdir()
                for child in previous.iterdir():child.rename(self.root/child.name)
        def load(store,*args,**kwargs):
            if when=='before':mutate()
            value=original(store,*args,**kwargs);loaded.append(True)
            if when=='after':mutate()
            return value
        gc.collect();before=len(os.listdir('/proc/self/fd'))
        with patch.object(checkpoint.MappingBackendCheckpointStore,'load',load):
            with self.assertRaises(Exception):(operation or self.h.activate)()
        gc.collect();self.assertEqual(len(os.listdir('/proc/self/fd')),before)
        self.assertEqual(loaded,[True])
        self.assertEqual(self.h.stub.installs,1 if operation else 0)
        if operation:self.assertEqual(self.h.stub.calls,[])
        # A new exact read can still take the store lock: no descriptor/lock leaked.
        reopened=checkpoint.MappingBackendCheckpointStore(self.root).load('voxel',schema_version=2)
        self.assertEqual(reopened.checkpoint.checkpoint_id,workspace_tests.CHECKPOINT_ID)

    def test_member_replacement_between_metadata_and_load_rejected(self):self._race('before','file')
    def test_member_replacement_between_load_and_verify_rejected(self):self._race('after','file')
    def test_directory_replacement_between_metadata_and_load_rejected(self):self._race('before','directory')
    def test_directory_replacement_between_load_and_verify_rejected(self):self._race('after','directory')

    def test_load_failure_releases_tree_and_locks(self):
        gc.collect();before=len(os.listdir('/proc/self/fd'))
        with patch.object(checkpoint.MappingBackendCheckpointStore,'load',side_effect=OSError('injected read failure')):
            with self.assertRaises(Exception):self.h.activate()
        gc.collect();self.assertEqual(len(os.listdir('/proc/self/fd')),before)
        self.assertEqual(self.h.stub.installs,0)
        self.h.assert_consistent(self.h.activate())

    def test_original_direct_prepare_still_loads(self):
        bundle=self.h.service.get_map_asset_bundle(map_name=workspace_tests.MAP_NAME)
        original=checkpoint.MappingBackendCheckpointStore.load;calls=[]
        def load(*args,**kw):calls.append(1);return original(*args,**kw)
        with patch.object(checkpoint.MappingBackendCheckpointStore,'load',load):
            self.h.runtime.prepare_assigned_map_activation(bundle,bundle_root=self.root)
        self.assertEqual(calls,[1])

    def _catalog_alias_activation(self, kind):
        if kind == 'symlink':
            alias = self.h.root / 'catalog-alias'
            alias.symlink_to(self.h.root, target_is_directory=True)
        else:
            hop = self.h.root / 'path-hop'
            hop.mkdir()
            alias = hop / '..'
        repository = asset_store.MapAssetBundleRepository(str(alias))
        self.h.service._asset_repository = repository
        root = repository.current_directory(workspace_tests.MAP_NAME)
        # The unchanged ordinary read/direct preparation already accepts this alias.
        bundle = self.h.service.get_map_asset_bundle(map_name=workspace_tests.MAP_NAME)
        direct = self.h.runtime.prepare_assigned_map_activation(bundle, bundle_root=root)
        self.assertEqual(direct.bundle_root, self.root.resolve(strict=True))
        read = self.h.service.get_map_asset_bundle(map_name=workspace_tests.MAP_NAME, _for_activation=True)
        with patch.object(checkpoint.MappingBackendCheckpointStore, 'load', side_effect=AssertionError('second load')):
            prepared = self.h.runtime.prepare_retained_assigned_map_activation(read,
                repository=repository, bundle_root=root)
        self.assertIs(prepared.stored_checkpoint, read.stored_checkpoint)
        self.assertEqual(prepared.bundle_root, direct.bundle_root)
        self.h.assert_consistent(self.h.activate())

    def test_symlink_ancestor_catalog_preserves_direct_prepare_semantics(self):
        self._catalog_alias_activation('symlink')

    def test_parent_component_catalog_preserves_direct_prepare_semantics(self):
        self._catalog_alias_activation('parent')

    def _start_cold_restore(self):
        original = self.h.activate()
        self.h.runtime = self.h.new_runtime()
        self.h.store = workspace_tests.StateStore()
        self.h.mapping = workspace_tests.MappingService(
            provider_owner=workspace_tests.SimpleNamespace(), state_store=self.h.store)
        self.h.localization = workspace_tests.LocalizationService(
            provider_owner=workspace_tests.SimpleNamespace(), state_store=self.h.store)
        self.h.memory = workspace_tests.MemoryEndpoint()
        self.h.service = self.h.new_service()
        self.repository = self.h.service._asset_repository
        self.h.stub.calls.clear()
        return original.active_localization_session.activation_receipt_v2

    def test_cold_restore_one_owned_validation_after_tree_closes_without_dispatch(self):
        receipt = self._start_cold_restore()
        closed = []
        original_close = asset_store._StableDirectoryTree.close
        original_prepare = self.h.runtime.prepare_retained_assigned_map_activation
        prepared_reads = []
        def close(tree):
            descriptors = [tree.parent_descriptor] + [x.descriptor for x in tree.files.values()]
            original_close(tree)
            closed.extend(fd for fd in descriptors if fd >= 0)
        def prepare(read, **kwargs):
            self.assertTrue(closed)
            for fd in closed:
                with self.assertRaises(OSError):os.fstat(fd)
            prepared = original_prepare(read, **kwargs)
            self.assertIs(prepared.stored_checkpoint, read.stored_checkpoint)
            self.assertEqual(bytes(read.stored_checkpoint.checkpoint.canonical_cloud_xyz.view()),
                             struct.pack('<fff', 1, 2, 3))
            prepared_reads.append(read)
            return prepared
        with patch.object(asset_store._StableDirectoryTree, 'close', close), \
                patch.object(self.h.runtime, 'prepare_retained_assigned_map_activation', prepare):
            restored = self._assert_one_owned_validation(self.h.service.restore_active_workspace)
        self.assertEqual(len(prepared_reads), 1)
        self.assertEqual(restored.active_localization_session.activation_receipt_v2, receipt)
        self.assertEqual(self.h.stub.calls, ['GetMappingState', 'GetMappingState'])

    def test_cold_restore_cancellation_after_read_does_not_cache_body_for_retry(self):
        self._start_cold_restore()
        original = self.h.runtime.prepare_retained_assigned_map_activation
        reads = []
        def cancel(read, **kwargs):
            reads.append(read)
            raise workspace_tests.ActivationCancelled('after stable read')
        with patch.object(self.h.runtime, 'prepare_retained_assigned_map_activation', cancel):
            with self.assertRaises(workspace_tests.ActivationCancelled):
                self.h.service.restore_active_workspace()
        self.assertEqual(self.h.stub.calls, [])
        self.assertIsNone(self.h.runtime._map_activation_snapshot_read().bundle)
        path = next((self.root / 'mapping_backend/voxel/objects').glob('*-backend.bin'))
        body = path.read_bytes()
        corrupt = bytearray(body);corrupt[-8:] = struct.pack('<d', float('nan'))
        path.write_bytes(corrupt)
        with self.assertRaises(workspace_tests.GatewayError) as rejected:
            self.h.service.restore_active_workspace()
        self.assertEqual(rejected.exception.error_code, 'map_asset_bundle_invalid')
        self.assertIsInstance(rejected.exception.__cause__, asset_store.MapAssetBundleValidationError)
        self.assertEqual(self.h.stub.calls, [])
        path.write_bytes(body)
        def retry(read, **kwargs):
            self.assertIsNot(read, reads[0])
            self.assertIsNot(read.stored_checkpoint, reads[0].stored_checkpoint)
            return original(read, **kwargs)
        with patch.object(self.h.runtime, 'prepare_retained_assigned_map_activation', retry):
            self.h.assert_consistent(self.h.service.restore_active_workspace())
        self.assertEqual(self.h.stub.installs, 1)
        self.assertTrue(all(call == 'GetMappingState' for call in self.h.stub.calls))

    def test_cold_restore_load_cancellation_releases_tree_and_never_dispatches(self):
        self._start_cold_restore()
        gc.collect();before = len(os.listdir('/proc/self/fd'))
        with patch.object(checkpoint.MappingBackendCheckpointStore, 'load',
                          side_effect=workspace_tests.ActivationCancelled('during load')):
            with self.assertRaises(workspace_tests.ActivationCancelled):
                self.h.service.restore_active_workspace()
        gc.collect();self.assertEqual(len(os.listdir('/proc/self/fd')), before)
        self.assertEqual(self.h.stub.calls, [])
        self.h.assert_consistent(self.h.service.restore_active_workspace())
        self.assertEqual(self.h.stub.installs, 1)

    def test_cold_restore_member_replacement_before_load_rejected(self):
        self._start_cold_restore()
        self._race('before', 'file', self.h.service.restore_active_workspace)

    def test_cold_restore_member_replacement_after_load_rejected(self):
        self._start_cold_restore()
        self._race('after', 'file', self.h.service.restore_active_workspace)

    def test_cold_restore_directory_replacement_before_load_rejected(self):
        self._start_cold_restore()
        self._race('before', 'directory', self.h.service.restore_active_workspace)

    def test_cold_restore_directory_replacement_after_load_rejected(self):
        self._start_cold_restore()
        self._race('after', 'directory', self.h.service.restore_active_workspace)
