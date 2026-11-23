/* 新控制台入口：统一状态 → 主实时连接 → 任务表 → 数据源 → 外壳。
   主连接本批只订阅外壳用到的主题；舞台、面板模块挂载时用 stream.want() 增加主题。 */
import { createStore, initialState } from './store.js';
import * as api from './api.js';
import { createStream } from './stream.js';
import { createTasks } from './tasks.js';
import { startFeeds } from './feeds.js';
import { createShell } from './shell.js';

const store = createStore(initialState());
const stream = createStream({ topics: ['robot.state', 'geometry.state', 'tasks.state', 'system.metrics'],
  params: { map_binary_version: '4' }, queued: ['tasks.state'], primary: true, store });
const tasks = createTasks({ store, stream, api });
const feeds = startFeeds({ store, stream, api });
createShell({ store, stream, api, tasks, feeds });
stream.connect();
void tasks.load();
