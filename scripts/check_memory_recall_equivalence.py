"""更新后的当前记忆内容与筛选立即生效，不返回旧源的词法命中。"""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from contracts.memory import SemanticMemoryEntry,MemoryPayloadFilter
from scripts.diagnostics.current_fixtures import memory_service

class RecallChecks(unittest.TestCase):
    def test_updated_record_does_not_keep_old_term_or_old_filter(self):
        with memory_service() as service:
            record=SemanticMemoryEntry(memory_id="mem_fixture_20260101T000000Z_00000001",title="needle",summary="",map_name="old-map")
            service.write(lambda store:store.put("memory",record))
            self.assertEqual(service.query_semantic_memory("needle",similarity_threshold=.8).matches[0].record_id,record.memory_id)
            current=record.model_copy(update={"title":"cabinet","map_name":"new-map"})
            service.write(lambda store:store.put("memory",current))
            self.assertEqual(service.query_semantic_memory("needle",similarity_threshold=.8).matches,[])
            self.assertEqual(service.query_semantic_memory("cabinet",payload_filter=MemoryPayloadFilter(map_name="old-map")).matches,[])
            self.assertEqual(service.query_semantic_memory("cabinet",payload_filter=MemoryPayloadFilter(map_name="new-map")).matches[0].record_id,record.memory_id)

if __name__=="__main__":unittest.main(verbosity=2)
