//! 完整原生测量只发布一次；保留子进程算法要求的扫描序号。
use crate::lio_ingress::{
    encode_native_imu_payload, encode_pointcloud_request_payload_with_sequence,
    validate_native_raw_pointcloud_fields,
};
use crate::native_lio_owner::NativePrivateChannels;
use crate::proto::PublishPointCloudRequest;
use nuwax_codec::lcm::input_frame;
use nuwax_protocol::RawLcmFrameBuilder;
use nuwax_transport::LcmTransport;
use tonic::Status;

#[derive(Default)]
pub(crate) struct NativeInputDelivery {
    sequence: i32,
    last_imu_timestamp_ns: Option<u64>,
    failure: Option<Status>,
}
impl NativeInputDelivery {
    pub(crate) fn has_failed(&self) -> bool {
        self.failure.is_some()
    }
    pub(crate) async fn publish(
        &mut self,
        transport: &LcmTransport,
        channels: &NativePrivateChannels,
        request: &PublishPointCloudRequest,
    ) -> Result<(), Status> {
        if let Some(error) = &self.failure {
            return Err(error.clone());
        }
        validate_native_raw_pointcloud_fields(request)
            .map_err(|e| Status::invalid_argument(e.to_string()))?;
        if request.imu_samples.is_empty() {
            return Err(Status::invalid_argument("原生输入需要真实惯性窗口"));
        }
        let imu = request
            .imu_samples
            .iter()
            .map(encode_native_imu_payload)
            .collect::<Result<Vec<_>, _>>()
            .map_err(|e| Status::invalid_argument(e.to_string()))?;
        let mut latest = self.last_imu_timestamp_ns;
        for sample in &request.imu_samples {
            if let Some(previous) = latest {
                // 与原生前端 scan_imu_buffer.hpp 的既有物理缺测上限一致。
                if sample.timestamp_ns > previous && sample.timestamp_ns - previous > 500_000_000 {
                    let error = Status::failed_precondition(format!(
                        "惯性源时间断档：{}纳秒",
                        sample.timestamp_ns - previous
                    ));
                    self.failure = Some(error.clone());
                    return Err(error);
                }
            }
            latest = Some(latest.map_or(sample.timestamp_ns, |last| last.max(sample.timestamp_ns)));
        }
        self.sequence = self
            .sequence
            .checked_add(1)
            .ok_or_else(|| Status::out_of_range("原生扫描序号耗尽"))?;
        let cloud = encode_pointcloud_request_payload_with_sequence(request, self.sequence)
            .map_err(|e| Status::invalid_argument(e.to_string()))?;
        let frames = if let Some(binding) = &channels.input_frame {
            let payload = input_frame::encode_body(&cloud, &imu)
                .map_err(|e| Status::invalid_argument(e.to_string()))?;
            vec![
                RawLcmFrameBuilder::new(binding.0.clone(), binding.1.clone())
                    .payload(payload)
                    .build(),
            ]
        } else {
            let mut frames = imu
                .into_iter()
                .map(|payload| {
                    RawLcmFrameBuilder::new(
                        channels.input_imu.0.clone(),
                        channels.input_imu.1.clone(),
                    )
                    .payload(payload)
                    .build()
                })
                .collect::<Vec<_>>();
            frames.push(
                RawLcmFrameBuilder::new(
                    channels.input_cloud.0.clone(),
                    channels.input_cloud.1.clone(),
                )
                .payload(cloud)
                .build(),
            );
            frames
        };
        transport
            .publish_batch(frames)
            .await
            .map_err(|e| Status::unavailable(e.to_string()))?;
        self.last_imu_timestamp_ns = latest;
        Ok(())
    }
}

#[cfg(test)]
#[path = "native_input_delivery_tests.rs"]
mod tests;
