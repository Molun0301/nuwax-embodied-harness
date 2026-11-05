"""保留当前记忆算法的离线行为检查，不使用旧持久索引与身份协议。"""
from pathlib import Path
import sys
import unittest
from datetime import datetime,timezone
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from services.memory.service import MemoryService
import threading

class ServiceRegressionChecks(unittest.TestCase):
    def fixture(self):
        service=MemoryService.__new__(MemoryService)
        service._query_score_state=threading.local()
        return service

    def test_fuzzy_signatures_match_original_formula_and_preserve_token_identity(self):
            import random
            from difflib import SequenceMatcher
            service=self.fixture();randomizer=random.Random(3101)
            cases=[('desk',['Desk','table','desk 11','desk 99','']),
                   ('',['','desk']),('  !?  ',['','!?','desk']),
                   ('a b',['b a','a b']),('abc',['abcd','acbd','abcxxxxxxxxxx']),
                   ('  CAFÉ\x00桌子  ',['café桌子','ＣＡＦÉ 桌子','cafe\u0301桌子','CAFÉ\x00桌子']),
                   ('a b c d e',['a b c x x','a b c y z']),
                   ('tide',['diet']),('diet',['tide']),
                   ('前方红色桌子',['前方蓝色桌子123','前方绿色桌子789']),
                   ('a'*205+'b',['a'*205+'c','a'*205+'d','a'*199+'b','a'*200+'b']),
                   ('x y',['','  !?  ','x' * 201,'y' * 400])]
            for length in (199,200,201):
                left='b'+'a'*(length-1);right='a'*(length-1)+'b'
                cases.extend([(left,[right,'b'+'a'*(length-2)+'c']),
                              (right,[left,'c'+'a'*(length-1)])])
            alphabet='abcdefghi中女美98312 '
            for _ in range(800):
                query=''.join(randomizer.choices('abcde中女12 ',k=randomizer.randrange(1,60)))
                texts=[''.join(randomizer.choices(alphabet,k=randomizer.choice([5,20,100,199,200,201,400]))) for _ in range(3)]
                # Distinct actual strings sharing a signature exercise cache hits.
                texts += [text.translate(str.maketrans({'f':'g','h':'i','美':'女' if '女' not in query else '美','9':'8'})) for text in texts]
                cases.append((query,texts))
            def original(query,texts):
                if not query:return 0.0,'test'
                best=0.0
                for text in texts:
                    text=service._normalize_text(text)
                    if not text:continue
                    if query==text:return .99,'文本完全匹配。'
                    overlap=service._token_overlap_score(service._tokenize(query),service._tokenize(text))
                    fuzzy=SequenceMatcher(None,query,text).ratio()
                    best=max(best,fuzzy,overlap,.6*fuzzy+.4*overlap)
                return best,'test'
            def score(*args,**kwargs):
                for mode in ('disabled','score-only','query-local'):
                    state=SimpleNamespace()
                    if mode!='disabled':state.cache={}
                    if mode=='query-local':state.fuzzy_cache={}
                    with patch.object(service,'_query_score_state',state):
                        for number,(raw,texts) in enumerate(cases):
                            for name in ('cache','fuzzy_cache'):
                                if hasattr(state,name):getattr(state,name).clear()
                            query=service._normalize_text(raw)
                            with self.subTest(mode=mode,case=number):
                                # Compare the cold list before singleton calls warm either cache.
                                self.assertEqual(service._score_text_candidates(query,texts,fallback_reason='test'),original(query,texts))
                                for text in texts:
                                    self.assertEqual(service._score_text_candidates(query,[text],fallback_reason='test'),original(query,[text]))
                                self.assertEqual(service._score_text_candidates(query,texts,fallback_reason='test'),original(query,texts))
                                self.assertEqual(service._score_text_candidates(query,list(reversed(texts)),fallback_reason='test'),original(query,list(reversed(texts))))
                        for name in ('cache','fuzzy_cache'):
                            self.assertLessEqual(len(getattr(state,name,{})),2048)
            score()

if __name__=="__main__":unittest.main(verbosity=2)
