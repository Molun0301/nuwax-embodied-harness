#!/usr/bin/env node
// Exercise the production status renderer with a minimal DOM, without dependencies or network.
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const root = path.resolve(__dirname, '..');
const html = fs.readFileSync(path.join(root, 'scripts/device_manager/static/index.html'), 'utf8');
const nodes = new Map(Array.from(html.matchAll(/\bid="([^"]+)"/g), match => [match[1], {textContent: ''}]));
const context = vm.createContext({document: {
  readyState: 'loading', addEventListener() {}, getElementById: id => nodes.get(id)
}});
let source = fs.readFileSync(path.join(root, 'scripts/device_manager/static/app.js'), 'utf8');
const end = source.lastIndexOf('})();');
source = source.slice(0, end) + '\nglobalThis.applyStatus = applyStatus;\n' + source.slice(end);
vm.runInContext(source, context);

for (const installed of [false, true, false]) {
  const installDir = '/mnt/机器人 data/<literal>& "directory"';
  context.applyStatus({installation: {installed, install_dir: installDir}});
  assert.equal(nodes.get('setupInstallDir').textContent, installDir);
  assert.equal(nodes.get('installDir').textContent, installDir);
  assert.equal(nodes.get('setupInstallDir').innerHTML, undefined, 'Paths must use textContent');
}
context.applyStatus({installation: {installed: false}});
assert.equal(nodes.get('setupInstallDir').textContent, '未知', 'Missing data must not invent a default path');
assert.equal(nodes.get('installDir').textContent, '未知', 'Refresh must not retain an old location');
assert(html.includes('--install-dir "目标目录"'));
assert(html.includes('--cli'));
console.log('Install-location UI: first install, installed, repeat and missing status checks passed');
