// 分页点云传输与原子接纳；不拥有显示资源，不物化累计 XYZ。
export const POINTCLOUD_PAGE_POINTS = 1024;
export const POINTCLOUD_MAX_POINTS = Math.floor(32 * 1024 * 1024 / 12);
const MAGIC = [0x4e, 0x55, 0x57, 0x41, 0x58, 0x50, 0x43, 0];
const IDENTITY_FIELDS = ['source', 'source_epoch', 'stream_generation', 'layout_generation', 'frame_id'];

export function pointCloudBase(header) {
  return Object.fromEntries([...IDENTITY_FIELDS, 'revision_ordinal'].map(key => [key, header[key]]));
}

export function parsePointCloudPagePacket(buffer) {
  if (!(buffer instanceof ArrayBuffer) || buffer.byteLength < 13 || buffer.byteLength > 128 * 1024) throw new Error('分页点云消息大小非法');
  const bytes = new Uint8Array(buffer);
  if (!MAGIC.every((value, index) => bytes[index] === value) || bytes[8] !== 2) throw new Error('不支持的分页点云协议');
  const headerLength = new DataView(buffer).getUint32(9, true);
  const payloadOffset = 13 + headerLength;
  if (!headerLength || headerLength > 64 * 1024 || payloadOffset > bytes.length || payloadOffset % 4) throw new Error('分页点云头长度非法');
  const header = JSON.parse(new TextDecoder('utf8', {fatal: true}).decode(bytes.subarray(13, payloadOffset)));
  if (payloadOffset + header.payload_length !== buffer.byteLength) throw new Error('分页点云总长度不一致');
  let offset = payloadOffset;
  const chunks = header.chunks.map(item => {
    const payloadBytes = item.point_count * 12;
    if (payloadBytes < 0 || offset + payloadBytes > buffer.byteLength) throw new Error('点云页面截断');
    const xyz = new Float32Array(buffer, offset, item.point_count * 3);
    offset += payloadBytes;
    return {...item, xyz};
  });
  if (offset !== buffer.byteLength) throw new Error('点云页面目录未覆盖载荷');
  return {header, chunks};
}

export class PointCloudPageAssembler {
  constructor() { this.base = null; this.pointCount = 0; this.pending = null; this.mapEpoch = ''; }
  beginConnection() { this.pending = null; }
  discardPending() { this.pending = null; }
  invalidate() { this.pending = null; this.base = null; this.pointCount = 0; }
  setMapEpoch(epoch) {
    if (epoch === this.mapEpoch) return false;
    this.mapEpoch = epoch;
    this.invalidate();
    return true;
  }
  push(buffer) {
    const packet = parsePointCloudPagePacket(buffer);
    const header = packet.header;
    if (header.packet_index === 0) {
      this.pending = null;
      if (!header.reset && (!this.base
          || BigInt(this.base.revision_ordinal) !== BigInt(header.base_revision_ordinal))) {
        throw new Error('点云基线不匹配');
      }
      this.pending = {header, chunks: [], next: 0};
    }
    const pending = this.pending;
    pending.chunks.push(...packet.chunks);
    pending.next++;
    if (pending.next < pending.header.packet_count) return null;
    this.pending = null;
    return {header: pending.header, chunks: pending.chunks};
  }
  accept(batch) {
    this.base = pointCloudBase(batch.header);
    this.pointCount = batch.header.point_count;
  }
}

export function mappingEpochFromMap(snapshot) {
  const metadata = snapshot?.metadata || {};
  if (metadata.identity_schema_version === 2) {
    // 地图选择请求沿现有服务端接口编码；接收页面时不重建或比对资产身份。
    return `assigned:${metadata.asset_identity.asset_id}:${metadata.activation_receipt_v2.request.checkpoint_id}:${metadata.map_activation_generation}`;
  }
  let epoch = metadata.mapping_session_id
    ? `${metadata.mapping_session_id}:${metadata.session_generation}` : snapshot?.source_epoch;
  for (let depth = 0; typeof epoch === 'string' && depth < 4; depth++) {
    let value; try { value = JSON.parse(epoch); } catch (_) { break; }
    if (!Array.isArray(value) || value[0] !== 'map-view-v2') break;
    epoch = value[1];
  }
  return epoch || '';
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
    this.onState('resync');
    if (this.socket?.readyState === WebSocket.OPEN) this.socket.send(JSON.stringify({type: 'pointcloud.resync', mapping_source_epoch: this.assembler.mapEpoch}));
  }
  setMapEpoch(epoch) {
    if (!this.assembler.setMapEpoch(epoch)) return;
    this.onInvalidate();
    // 新选择使用新连接，旧连接已排队的页面由连接代次直接撤销。
    this.epoch++;
    if (this.timer !== null) window.clearTimeout(this.timer);
    this.timer = null;
    const socket = this.socket; this.socket = null; socket?.close();
    this.awaitingReset = false;
    if (epoch) this.connect();
  }
  dispose() {
    this.closed = true; this.epoch++;
    if (this.timer !== null) window.clearTimeout(this.timer);
    this.timer = null; this.assembler.invalidate();
    const socket = this.socket; this.socket = null; socket?.close();
  }
}
