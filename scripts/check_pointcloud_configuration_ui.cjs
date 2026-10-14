// Runs the real settings row renderer with a minimal DOM; no browser/network needed.
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const path = require('node:path');
class Element {
  constructor(tag) { this.tagName = tag; this.children = []; this.dataset = {}; this._text = ''; this.classList = { add() {}, toggle() {} }; }
  append(...items) { this.children.push(...items); }
  appendChild(item) { this.children.push(item); return item; }
  addEventListener() {}
  set textContent(value) { this._text = String(value); }
  get textContent() { return this._text + this.children.map(c => c.textContent || '').join(' '); }
}
const file = path.join(__dirname, '../gateways/http/static/js/console-settings.js');
let source = fs.readFileSync(file, 'utf8');
const end = source.lastIndexOf('})();');
assert(end > 0);
source = source.slice(0, end) + '\nglobalThis.testBuildRow = buildConfigRow;\n' + source.slice(end);
const context = { window: {}, document: { createElement: tag => new Element(tag), getElementById: () => null }, console };
vm.createContext(context);
vm.runInContext(source, context, { filename: file });
const base = { key: 'NUWAX_GEOMETRY_POINT_CLOUD_MAX_POINTS', zh_name: '兼容配置', type: 'int', value: '1200',
  default_value: '1200', effective_value_known: false, pending_restart: false };
const inactive = context.testBuildRow({ ...base, effective_value_scope: 'inactive', apply_scope: '仅兼容保存，当前内置执行链不使用' });
assert(inactive.textContent.includes('兼容保留，当前流水线不使用'));
assert(!inactive.textContent.includes('当前进程值未采集'));
assert(!inactive.textContent.includes('待重启生效'));
const external = context.testBuildRow({ ...base, effective_value_scope: 'external_process' });
assert(external.textContent.includes('外部进程实际采用值未核实'));
const currentUnknown = context.testBuildRow({ ...base, effective_value_scope: 'current_process' });
assert(currentUnknown.textContent.includes('当前进程值未采集'));
const currentPending = context.testBuildRow({ ...base, effective_value_scope: 'current_process', effective_value_known: true,
  pending_restart: true, effective_value: '900' });
assert(currentPending.textContent.includes('已保存，待重启生效'));
assert(currentPending.textContent.includes('当前生效：900'));
console.log('4 real-renderer status cases passed');
