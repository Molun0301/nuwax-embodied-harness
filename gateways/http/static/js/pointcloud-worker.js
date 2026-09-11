import { PointCloudProcessor } from './pointcloud-processing.js?v=20260911-background-preprocess';

// 本线程只拥有一次显示预处理，不访问设备、业务状态或图形资源。
self.onmessage = ({ data }) => {
  const { id, config, options, positions } = data;
  try {
    const startedAt = performance.now();
    const processor = new PointCloudProcessor(config);
    processor._allPositions = positions;
    processor._pointStyle = options.style;
    processor._lod = options.lod;
    processor._isLiveCloudMapSnapshot = options.live;
    processor._robotPose = { z: options.robotZ };
    const result = processor._preparePointCloud();
    result.computeMs = performance.now() - startedAt;
    // 同一缓冲只能转移一次；即使显示位置直接复用输入也不回传源数据副本。
    const buffers = [...new Set([result.positions.buffer, result.colors.buffer])];
    self.postMessage({ id, result }, buffers);
  } catch (error) {
    self.postMessage({ id, error: String(error?.message || error) });
  }
};
