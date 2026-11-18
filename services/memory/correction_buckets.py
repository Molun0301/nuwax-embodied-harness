"""按来源时刻分开保存实例的累积几何。"""
import math

import numpy as np

from contracts.spatial_memory import InstanceCorrectionBucket
from services.memory.instance_evidence import accumulate_support_voxels, fuse_position, observation_support_points


# 五秒内相邻观测的回环增量差别在厘米级。
BUCKET_NS = 5_000_000_000


def bucket_key(pose):
    stamp = pose.source_timestamp_ns or 0
    return pose.session_id, stamp // BUCKET_NS * BUCKET_NS


def voxel_centers(support):
    return (np.asarray(support.voxel_keys).reshape(-1, 3) + 0.5) * support.voxel_size_m


def accumulate(support, points, frame, config):
    return accumulate_support_voxels(support, points, frame_id=frame,
        voxel_size_m=config.support_voxel_size_m, max_voxels=config.support_max_voxels)


def instance_buckets(instance):
    if instance.correction_buckets:
        return instance.correction_buckets
    if instance.support_evidence is not None:
        return [InstanceCorrectionBucket(session_id=0, start_ns=0, mean_offset_ns=0,
            pose=instance.pose, support=instance.support_evidence)]
    return []


def _combine(bucket, incoming, offset, weight, points, config):
    prior_weight = bucket.support.position_weight if bucket is not None else 0.0
    support = accumulate(bucket.support if bucket is not None else None, points, incoming.frame_id, config)
    pose = incoming if bucket is None else bucket.pose
    mean = offset if bucket is None else bucket.mean_offset_ns
    if weight:
        pose, total, sigma = fuse_position(pose, prior_weight, incoming, math.sqrt(1 / weight))
        mean += (offset - mean) * weight / total
        support = support.model_copy(update={"position_weight": total, "position_sigma_m": sigma})
    return pose, mean, support


def _remove_position(bucket, pose, weight):
    remaining = max(0.0, bucket.support.position_weight - weight)
    if remaining <= 1e-9:
        return bucket.model_copy(update={"support": bucket.support.model_copy(
            update={"position_weight": 0.0, "position_sigma_m": None})})
    fraction = weight / remaining
    position = bucket.pose.position.model_copy(update={axis: getattr(bucket.pose.position, axis) +
        fraction * (getattr(bucket.pose.position, axis) - getattr(pose.position, axis)) for axis in "xyz"})
    mean = bucket.mean_offset_ns + fraction * (bucket.mean_offset_ns -
        ((pose.source_timestamp_ns or 0) - bucket.start_ns))
    return bucket.model_copy(update={"pose": bucket.pose.model_copy(update={"position": position}),
        "mean_offset_ns": mean, "support": bucket.support.model_copy(
            update={"position_weight": remaining, "position_sigma_m": math.sqrt(1 / remaining)})})


def add_observation(instance, observation, sigma, config, *, position=True, removed=None,
                    reset_position=False, reset_voxels=False):
    incoming = observation.pose
    buckets = {(bucket.session_id, bucket.start_ns): bucket
               for bucket in instance_buckets(instance)} if instance is not None else {}
    if reset_position:
        buckets = {key: bucket.model_copy(update={"support": bucket.support.model_copy(update={
            "position_weight": 0.0, "position_sigma_m": None})})
            for key, bucket in buckets.items() if not reset_voxels}
    elif removed is not None:
        pose, prior_weight = removed
        key = bucket_key(pose)
        if key in buckets:
            buckets[key] = _remove_position(buckets[key], pose, prior_weight)
    key = bucket_key(incoming)
    points = observation_support_points(observation.attributes, frame_id=incoming.frame_id)
    pose, mean, support = _combine(buckets.get(key), incoming, (incoming.source_timestamp_ns or 0) - key[1],
        1 / sigma ** 2 if position else 0.0, points if points is not None else (), config)
    buckets[key] = InstanceCorrectionBucket(session_id=key[0], start_ns=key[1],
        mean_offset_ns=mean, pose=pose, support=support)
    return list(buckets.values())


def merge_buckets(records, config, *, position_record=None):
    buckets = {}
    for record in records:
        for bucket in instance_buckets(record):
            key = bucket.session_id, bucket.start_ns
            weight = bucket.support.position_weight if position_record is None or record is position_record else 0.0
            pose, mean, support = _combine(buckets.get(key), bucket.pose, bucket.mean_offset_ns,
                weight, voxel_centers(bucket.support), config)
            buckets[key] = bucket.model_copy(update={"pose": pose, "mean_offset_ns": mean, "support": support})
    return list(buckets.values())


def reduce_buckets(buckets, pose, evidence, config):
    weight = 0.0
    sigma = evidence.position_sigma_m
    points = []
    for bucket in sorted(buckets, key=lambda bucket: (bucket.start_ns, bucket.session_id)):
        if bucket.support.position_weight:
            pose, weight, sigma = fuse_position(pose, weight, bucket.pose,
                math.sqrt(1 / bucket.support.position_weight))
        points.append(voxel_centers(bucket.support))
    support = accumulate(None, np.concatenate(points), evidence.frame_id, config)
    return pose.position, evidence.model_copy(update={"voxel_keys": support.voxel_keys,
        "position_weight": weight, "position_sigma_m": sigma})
