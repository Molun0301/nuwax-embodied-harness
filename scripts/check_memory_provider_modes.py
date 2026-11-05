"""当前记忆库词法和明确向量提供器的真实查询检查，不连接模型。"""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from contracts.memory import SemanticMemoryEntry
from scripts.diagnostics.current_fixtures import memory_service
from scripts.memory_test_vectors import ExplicitTextEmbedder

class ProviderModeChecks(unittest.TestCase):
    def test_lexical_mode_returns_current_record_without_fabricated_vectors(self):
        with memory_service() as service:
            record=SemanticMemoryEntry(memory_id="mem_fixture_20260101T000000Z_00000001",title="desk",summary="a desk",map_name="default")
            service.write(lambda store:store.put("memory",record))
            result=service.query_semantic_memory("desk",similarity_threshold=.4)
            self.assertEqual([item.record_id for item in result.matches],[record.memory_id])
            self.assertEqual(dict(service.snapshot().vectors),{})
    def test_explicit_real_provider_scores_without_fake_fallback(self):
        with memory_service(text_embedder=ExplicitTextEmbedder(dimension=16)) as service:
            record=SemanticMemoryEntry(memory_id="mem_fixture_20260101T000000Z_00000002",title="desk",summary="a desk",map_name="default")
            service.write(lambda store:store.put("memory",record))
            self.assertEqual(service.query_semantic_memory("desk").matches[0].record_id,record.memory_id)

if __name__=="__main__":unittest.main(verbosity=2)
