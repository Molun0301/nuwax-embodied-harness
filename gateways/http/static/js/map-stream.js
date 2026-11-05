/* 地图与位姿数据流的浏览器编解码。 */
(function () {
"use strict";
      function decodeMapBinaryFrame(buffer) {
        const bytes = new Uint8Array(buffer);
        const magic = [0x4e, 0x55, 0x57, 0x41, 0x58, 0x4d, 0x41, 0x50];
        if (bytes.byteLength < 13 || !magic.every((value, index) => bytes[index] === value) || bytes[8] !== 4) return null;
        const headerLength = new DataView(buffer, 9, 4).getUint32(0, true);
        if (13 + headerLength > bytes.byteLength) return null;
        const message = JSON.parse(new TextDecoder().decode(bytes.subarray(13, 13 + headerLength)));
        const frame = message.frame;
        const grid = frame.payload.grid;
        const cells = grid.width * grid.height;
        if (13 + headerLength + cells + frame.payload.raw_data_length > bytes.byteLength) return null;
        grid.data = new Int8Array(buffer, 13 + headerLength, cells);
        frame.payload.raw_data = new Int8Array(buffer, 13 + headerLength + cells, frame.payload.raw_data_length);
        delete frame.payload.raw_data_length;
        return frame;
      }
      function streamBytes(value) {
        if (value instanceof Int8Array) return value;
        const raw = atob(value.data);
        return Int8Array.from(raw, (item) => item.charCodeAt(0));
      }
      function mapForCanvas(update) {
        if (!update) return {};
        const grid = update.grid;
        return {map_name: grid.map_name, occupancy_grid: {...grid, map_id: grid.map_name,
          resolution_m: grid.resolution, data: streamBytes(grid.data),
          origin: {position: {x: grid.origin_x, y: grid.origin_y, z: 0}}}};
      }
      function poseForCanvas(pose) {
        return pose ? {position: {x: pose.x, y: pose.y, z: pose.z},
          orientation: {x: pose.qx, y: pose.qy, z: pose.qz, w: pose.qw},
          frame_id: pose.frame_id, timestamp: new Date(pose.timestamp_ns / 1e6).toISOString()} : null;
      }

window.nuwaxMapStream = {decodeMapBinaryFrame, mapForCanvas, poseForCanvas};
})();
