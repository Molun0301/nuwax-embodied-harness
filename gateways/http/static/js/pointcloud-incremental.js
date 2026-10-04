// 分页点云传输与原子接纳；不拥有显示资源，不物化累计 XYZ。
export const POINTCLOUD_PAGE_POINTS = 1024;
export const POINTCLOUD_MAX_POINTS = Math.floor(32 * 1024 * 1024 / 12);
const MAGIC = [0x4e, 0x55, 0x57, 0x41, 0x58, 0x50, 0x43, 0];
const MAX_U64 = 18446744073709551615n;
const IDENTITY_FIELDS = ['source', 'source_epoch', 'stream_generation', 'layout_generation', 'frame_id'];
const TRANSACTION_FIELDS = [...IDENTITY_FIELDS, 'revision_ordinal', 'base_revision_ordinal', 'transfer_id',
  'revision', 'source_timestamp_ns', 'point_count', 'chunk_size_points', 'reset', 'packet_count',
  'changed_chunk_count', 'mapping_source_epoch', 'source_incremental'];

function integer(value, name) {
  if (typeof value !== 'string' || !/^(0|[1-9]\d*)$/.test(value) || BigInt(value) > MAX_U64) throw new Error(`非法点云整数 ${name}`);
  return value;
}
function count(value, maximum, name) {
  if (!Number.isSafeInteger(value) || value < 0 || value > maximum) throw new Error(`非法点云数量 ${name}`);
  return value;
}
export function pointCloudBase(header) {
  return Object.fromEntries([...IDENTITY_FIELDS, 'revision_ordinal'].map(key => [key, header[key]]));
}
function sameIdentity(left, right) { return IDENTITY_FIELDS.every(key => left?.[key] === right?.[key]); }

export function parsePointCloudPagePacket(buffer) {
  if (!(buffer instanceof ArrayBuffer) || buffer.byteLength < 13 || buffer.byteLength > 128 * 1024) throw new Error('分页点云消息大小非法');
  const bytes = new Uint8Array(buffer);
  if (!MAGIC.every((value, index) => bytes[index] === value) || bytes[8] !== 2) throw new Error('不支持的分页点云协议');
  const headerLength = new DataView(buffer).getUint32(9, true);
  const payloadOffset = 13 + headerLength;
  if (!headerLength || headerLength > 64 * 1024 || payloadOffset > bytes.length || payloadOffset % 4) throw new Error('分页点云头长度非法');
  const header = JSON.parse(new TextDecoder('utf8', {fatal: true}).decode(bytes.subarray(13, payloadOffset)));
  if (!header || header.topic !== 'pointcloud_map.pages' || header.format !== 'float32-xyz-le') throw new Error('分页点云格式非法');
  for (const key of ['source', 'source_epoch', 'frame_id', 'revision', 'transfer_id']) {
    if (typeof header[key] !== 'string' || !header[key] || header[key].length > 4096) throw new Error(`分页点云身份缺失 ${key}`);
  }
  if (!['live', 'saved'].includes(header.source) || typeof header.mapping_source_epoch !== 'string'
    || typeof header.reset !== 'boolean' || typeof header.source_incremental !== 'boolean') throw new Error('分页点云来源非法');
  for (const key of ['stream_generation', 'layout_generation', 'revision_ordinal', 'base_revision_ordinal', 'source_timestamp_ns']) integer(header[key], key);
  count(header.point_count, POINTCLOUD_MAX_POINTS, 'point_count');
  if (header.chunk_size_points !== POINTCLOUD_PAGE_POINTS) throw new Error('不支持的点云页面大小');
  const totalPages = Math.ceil(header.point_count / POINTCLOUD_PAGE_POINTS);
  count(header.changed_chunk_count, totalPages, 'changed_chunk_count');
  count(header.packet_count, Math.ceil(POINTCLOUD_MAX_POINTS / POINTCLOUD_PAGE_POINTS / 4), 'packet_count');
  if (header.packet_count !== Math.max(1, Math.ceil(header.changed_chunk_count / 4))) throw new Error('分页点云消息总数不一致');
  count(header.packet_index, header.packet_count - 1, 'packet_index');
  if (!Array.isArray(header.chunks) || header.chunks.length !== Math.min(4, header.changed_chunk_count - header.packet_index * 4)) throw new Error('分页点云页面目录不一致');
  if (header.source === 'saved' && !header.reset) throw new Error('保存资产必须是显式完整快照');
  if (header.reset && (header.base_revision_ordinal !== '0' || header.changed_chunk_count !== totalPages)) throw new Error('点云重建覆盖不完整');
  if (!header.reset && BigInt(header.revision_ordinal) <= BigInt(header.base_revision_ordinal)) throw new Error('点云修订未推进');
  count(header.payload_length, 4 * POINTCLOUD_PAGE_POINTS * 12, 'payload_length');
  if (payloadOffset + header.payload_length !== buffer.byteLength) throw new Error('分页点云总长度不一致');
  let offset = payloadOffset;
  const seen = new Set();
  const chunks = header.chunks.map(item => {
    count(item.chunk_index, totalPages - 1, 'chunk_index');
    if (seen.has(item.chunk_index)) throw new Error('重复点云页面');
    seen.add(item.chunk_index);
    const expected = Math.min(POINTCLOUD_PAGE_POINTS, header.point_count - item.chunk_index * POINTCLOUD_PAGE_POINTS);
    if (item.point_count !== expected) throw new Error('点云尾页长度不一致');
    integer(item.revision_ordinal, 'chunk revision');
    if (BigInt(item.revision_ordinal) > BigInt(header.revision_ordinal)
      || (!header.reset && BigInt(item.revision_ordinal) <= BigInt(header.base_revision_ordinal))) throw new Error('点云页面修订不一致');
    if (offset + item.point_count * 12 > buffer.byteLength) throw new Error('点云页面截断');
    const xyz = new Float32Array(buffer, offset, item.point_count * 3);
    for (const coordinate of xyz) if (!Number.isFinite(coordinate)) throw new Error('点云坐标必须有限');
    offset += xyz.byteLength;
    return {...item, xyz};
  });
  if (offset !== buffer.byteLength) throw new Error('点云页面目录未覆盖载荷');
  return {header, chunks};
}

export class PointCloudPageAssembler {
  constructor() { this.base = null; this.pointCount = 0; this.pending = null; this.mapEpoch = ''; this.connectionFresh = true; }
  beginConnection() { this.pending = null; this.connectionFresh = true; }
  discardPending() { this.pending = null; }
  invalidate() { this.pending = null; this.base = null; this.pointCount = 0; }
  setMapEpoch(epoch) {
    const next = typeof epoch === 'string' ? epoch : '';
    if (!next || next === this.mapEpoch) return false;
    const previous = this.mapEpoch;
    this.mapEpoch = next;
    if (this.base && ((this.acceptedMapEpoch && this.acceptedMapEpoch !== next)
      || (!this.acceptedMapEpoch && previous && previous !== next))) { this.invalidate(); return true; }
    if (this.pending?.header.mapping_source_epoch && this.pending.header.mapping_source_epoch !== next) this.pending = null;
    return false;
  }
  push(buffer) {
    const packet = parsePointCloudPagePacket(buffer);
    const header = packet.header;
    if (this.mapEpoch && header.mapping_source_epoch && this.mapEpoch !== header.mapping_source_epoch) {
      this.pending = null;
      throw new Error('点云所属地图已失效');
    }
    if (header.packet_index === 0) {
      if (this.pending) throw new Error('上一点云事务未结束');
      if (!header.reset && (!sameIdentity(this.base, header) || this.base.revision_ordinal !== header.base_revision_ordinal)) throw new Error('点云基线不匹配');
      // 同一连接内旧 reset 不能撤销已接受的较新来源代次或布局。
      if (this.base && !this.connectionFresh && (BigInt(header.stream_generation) < BigInt(this.base.stream_generation)
        || (header.source_epoch === this.base.source_epoch && header.source === this.base.source
          && BigInt(header.layout_generation) < BigInt(this.base.layout_generation))
        || (sameIdentity(this.base, header) && BigInt(header.revision_ordinal) < BigInt(this.base.revision_ordinal)))) throw new Error('点云重建修订已过期');
      this.pending = {header, chunks: [], seen: new Set(), next: 0, bytes: 0};
    }
    const pending = this.pending;
    if (!pending || header.packet_index !== pending.next || !TRANSACTION_FIELDS.every(key => header[key] === pending.header[key])) throw new Error('点云事务片段不连续');
    for (const chunk of packet.chunks) {
      if (pending.seen.has(chunk.chunk_index)) throw new Error('点云事务重复页面');
      pending.seen.add(chunk.chunk_index); pending.chunks.push(chunk); pending.bytes += chunk.xyz.byteLength;
    }
    if (pending.bytes > 32 * 1024 * 1024) throw new Error('点云事务超过显示容量');
    pending.next++;
    if (pending.next < header.packet_count) return null;
    if (pending.seen.size !== header.changed_chunk_count) throw new Error('点云事务页面缺失');
    if (!header.reset && header.point_count > this.pointCount) {
      for (let page = Math.floor(this.pointCount / POINTCLOUD_PAGE_POINTS); page < Math.ceil(header.point_count / POINTCLOUD_PAGE_POINTS); page++) {
        if (!pending.seen.has(page)) throw new Error('点云增长页面缺失');
      }
    }
    this.pending = null;
    return {header: pending.header, chunks: pending.chunks};
  }
  accept(batch) {
    this.connectionFresh = false;
    this.base = pointCloudBase(batch.header);
    this.pointCount = batch.header.point_count;
    this.acceptedMapEpoch = batch.header.mapping_source_epoch;
  }
}

export function mappingEpochFromMap(snapshot) {
  const metadata = snapshot?.metadata || {};
  const session = metadata.mapping_session_id;
  const generation = metadata.session_generation;
  const exactGeneration = (typeof generation === 'number' && Number.isSafeInteger(generation) && generation >= 0)
    || (typeof generation === 'string' && /^(0|[1-9]\d*)$/.test(generation) && BigInt(generation) <= MAX_U64);
  let epoch = typeof session === 'string' && session && exactGeneration ? `${session}:${generation}` : snapshot?.source_epoch;
  for (let depth = 0; typeof epoch === 'string' && depth < 4; depth++) {
    let value; try { value = JSON.parse(epoch); } catch (_) { break; }
    if (!Array.isArray(value) || value[0] !== 'map-view-v2' || typeof value[1] !== 'string') break;
    epoch = value[1];
  }
  return typeof epoch === 'string' ? epoch : '';
}

export class IncrementalPointCloudConnection {
  constructor({onBatch, onState = () => {}, onError = () => {}, onInvalidate = () => {}}) {
    this.onBatch = onBatch; this.onState = onState; this.onError = onError; this.onInvalidate = onInvalidate;
    this.assembler = new PointCloudPageAssembler(); this.socket = null; this.timer = null;
    this.closed = false; this.epoch = 0; this.retryMs = 1000; this.awaitingReset = false;
  }
  connect() {
    if (this.closed || this.socket) return;
    const url = new URL('/streams/pointcloud/ws', window.location.href);
    url.protocol = url.protocol === 'https:' ? 'wss:' : 'ws:';
    const socket = new WebSocket(url); const epoch = ++this.epoch;
    this.socket = socket; socket.binaryType = 'arraybuffer';
    const current = () => !this.closed && this.socket === socket && this.epoch === epoch;
    socket.addEventListener('open', () => {
      if (!current()) return;
      this.retryMs = 1000; this.awaitingReset = false; this.assembler.beginConnection();
      socket.send(JSON.stringify({type: 'pointcloud.resume', base: this.assembler.base, mapping_source_epoch: this.assembler.mapEpoch}));
    });
    socket.addEventListener('message', event => {
      if (!current()) return;
      try {
        if (typeof event.data === 'string') {
          const message = JSON.parse(event.data);
          if (message.type === 'pointcloud.ready' && message.version === 2) this.onState('ready');
          return;
        }
        if (this.awaitingReset) {
          const packet = parsePointCloudPagePacket(event.data);
          if (!packet.header.reset || packet.header.packet_index !== 0) return;
          this.awaitingReset = false;
        }
        const batch = this.assembler.push(event.data);
        if (!batch) return;
        this.onBatch(batch);
        this.assembler.accept(batch);
        socket.send(JSON.stringify({type: 'pointcloud.ack', transfer_id: batch.header.transfer_id, base: this.assembler.base}));
      } catch (error) {
        this.onError(error);
        this.resync();
      }
    });
    socket.addEventListener('close', () => {
      if (!current()) return;
      this.socket = null; this.assembler.discardPending(); this.onState('disconnected');
      this.timer = window.setTimeout(() => { this.timer = null; this.connect(); }, this.retryMs);
      this.retryMs = Math.min(15000, this.retryMs * 2);
    });
    socket.addEventListener('error', () => { if (current()) this.onState('error'); });
  }
  resync() {
    this.assembler.discardPending();
    if (this.awaitingReset) return;
    this.awaitingReset = true;
    if (this.socket?.readyState === WebSocket.OPEN) this.socket.send(JSON.stringify({type: 'pointcloud.resync', mapping_source_epoch: this.assembler.mapEpoch}));
  }
  setMapEpoch(epoch) {
    const previous = this.assembler.mapEpoch;
    if (this.assembler.setMapEpoch(epoch)) this.onInvalidate();
    if (previous === this.assembler.mapEpoch) return;
    this.assembler.discardPending();
    this.awaitingReset = true;
    if (this.socket?.readyState === WebSocket.OPEN) this.socket.send(JSON.stringify({type: 'pointcloud.map', mapping_source_epoch: this.assembler.mapEpoch}));
  }
  dispose() {
    this.closed = true; this.epoch++;
    if (this.timer !== null) window.clearTimeout(this.timer);
    this.timer = null; this.assembler.invalidate();
    const socket = this.socket; this.socket = null; socket?.close();
  }
}
