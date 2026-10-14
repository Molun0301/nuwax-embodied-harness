#!/usr/bin/env python3
"""Compare historical DISTINCT with production source seeks and limit-20 SQL pages.

Synthetic query-side benchmark only: excludes log refresh/parsing and raw-ID checks.
Run with the project's Python dependencies installed. No external files are changed.
"""
from __future__ import annotations

import argparse
import json
import sqlite3
import statistics
import time
from types import SimpleNamespace

import check_log_index_hashfree as harness


def measure(db, action, repeats):
    # Count separately so Python progress callbacks do not distort timing samples.
    calls = [0]
    def progress():
        calls[0] += 100
        return 0
    db.set_progress_handler(progress, 100)
    result = action()
    db.set_progress_handler(None, 0)
    samples = []
    for _ in range(repeats):
        started = time.perf_counter()
        assert action() == result
        samples.append((time.perf_counter() - started) * 1000)
    return result, {'median_ms': round(statistics.median(samples), 4),
                    'sqlite_ops_approx': calls[0]}


def run(count, source_count, repeats, heavy_first=0):
    db = sqlite3.connect(':memory:')
    db.executescript('''CREATE TABLE entries(scope_id INTEGER, source TEXT, time_key TEXT,
        identity TEXT, level TEXT);
        CREATE INDEX entry_source ON entries(scope_id,source,time_key,identity);
        CREATE INDEX entry_order ON entries(scope_id,time_key DESC,identity DESC);
        CREATE INDEX entry_level ON entries(scope_id,level,time_key DESC,identity DESC);''')
    db.executemany('INSERT INTO entries VALUES(?,?,?,?,?)',
        ((1, '' if i < heavy_first else 'source-' + str(i % source_count), str(i).zfill(8), str(i),
          'warning' if i % 997 == 0 else 'info') for i in range(count)))
    owner = SimpleNamespace(connection=db, scope_id=1)
    for cursor_name, cursor in [('newest', None), ('deep_1pct', (str(count // 100).zfill(8), '')),
                                ('empty', ('00000000', ''))]:
        matcher = harness.protocol._LogFilter('', '', '', '', '', '', '')
        matcher.cursor = cursor
        sql = 'SELECT DISTINCT source FROM entries WHERE scope_id=?'
        args = [1]
        if cursor:
            sql += ' AND (time_key,identity)<(?,?)'
            args.extend(cursor)
        for rare in (False, True):
            page_sql = 'SELECT time_key,identity FROM entries WHERE scope_id=?'
            page_args = [1]
            if cursor:
                page_sql += ' AND (time_key,identity)<(?,?)'; page_args.extend(cursor)
            if rare:
                page_sql += ' AND level=?'; page_args.append('warning')
            page_sql += ' ORDER BY time_key DESC,identity DESC LIMIT 21'
            def baseline():
                return sorted(row[0] for row in db.execute(sql, args)), list(db.execute(page_sql, page_args))
            def proposed():
                sources = harness.PersistentLogIndex._sources_before(owner, matcher, harness.index_module._WorkBudget())
                return sorted(sources), list(db.execute(page_sql, page_args))
            old_result, old = measure(db, baseline, repeats)
            new_result, new = measure(db, proposed, repeats)
            assert old_result == new_result
            print(json.dumps({'rows': count, 'sources': source_count, 'cursor': cursor_name,
                              'rare_level': rare, 'limit': 20, 'heavy_first': heavy_first, 'baseline': old, 'current': new,
                              'distinct_plan': [r[3] for r in db.execute('EXPLAIN QUERY PLAN ' + sql, args)],
                              'seek_plan': [r[3] for r in db.execute(
                                  'EXPLAIN QUERY PLAN SELECT source,time_key,identity FROM entries '
                                  'WHERE scope_id=? AND source>? ORDER BY source,time_key,identity LIMIT 2',
                                  [1, 'source-0'])]}), flush=True)
    db.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rows', type=int, nargs='+', default=[10000, 100000, 1000000])
    parser.add_argument('--sources', type=int, nargs='+', default=[3, 10])
    parser.add_argument('--repeats', type=int, default=5)
    parser.add_argument('--heavy-first', type=int, default=0, help='Initial records assigned to one source sorting first')
    options = parser.parse_args()
    print(json.dumps({'python': __import__('sys').version, 'sqlite': sqlite3.sqlite_version}), flush=True)
    for count in options.rows:
        for source_count in options.sources:
            run(count, source_count, options.repeats, options.heavy_first)
