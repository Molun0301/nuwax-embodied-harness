"""将同一会话的记忆坐标推进到重放后的地图坐标。"""
from dataclasses import replace
from itertools import product

import numpy as np

from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.geometry import Pose, Quaternion, Vector3
from contracts.spatial_memory import InstanceSupportEvidence, SceneObservationSample
from services.memory.correction_buckets import accumulate, bucket_key, reduce_buckets, voxel_centers
from services.memory.instance_evidence import support_extent


SESSION_KEY = "geometry_session_id"
SAMPLE_SOURCE = "geometry_sample_source"
_AXES = "xyz"


def _xyz(value):
    if isinstance(value, dict):
        return [value[axis] for axis in _AXES]
    if isinstance(value, (tuple, list, np.ndarray)):
        return value
    return [getattr(value, axis) for axis in _AXES]


def _point(value):
    return dict(zip(_AXES, value.tolist()))


def _like(value, position):
    return _point(position) if isinstance(value, dict) else position.tolist()


class Rigid:
    def __init__(self, rotation, translation):
        self.rotation = rotation
        self.translation = np.asarray(translation)
        vector = np.asarray(_xyz(rotation))
        identity = np.eye(3)
        self.matrix = identity + 2 * rotation.w * np.cross(vector, identity).T + (
            2 * (np.outer(vector, vector) - vector.dot(vector) * identity))

    def point(self, value):
        return np.asarray(_xyz(value)) @ self.matrix.T + self.translation

    def direction(self, value):
        return self.matrix @ np.asarray(_xyz(value))

    def orientation(self, value):
        left, right = np.asarray(_xyz(self.rotation)), np.asarray(_xyz(value))
        xyz = self.rotation.w * right + value.w * left + np.cross(left, right)
        return Quaternion(**_point(xyz), w=self.rotation.w * value.w - left.dot(right))

    def pose(self, pose, seq):
        return pose.model_copy(update={"position": Vector3(**_point(self.point(pose.position))),
            "orientation": self.orientation(pose.orientation), "correction_seq": seq})

    def bounds(self, lower, upper):
        corners = self.point(list(product(*zip(_xyz(lower), _xyz(upper)))))
        return _point(corners.min(axis=0)), _point(corners.max(axis=0))

    def extent(self, extent):
        return {**extent, "center_offset_m": _point(self.direction(extent["center_offset_m"])),
            "orientation": self.orientation(Quaternion.model_validate(extent["orientation"])).model_dump()}


class MapCorrection:
    def __init__(self, session_id, seq, frame, message, config):
        self.session_id = session_id
        self.seq = seq
        self.frame = frame
        self.config = config
        self.keyframes = tuple((key.timestamp_ns, np.asarray(_xyz(key.old_position)), Rigid(
            Quaternion(x=key.rotation.x, y=key.rotation.y, z=key.rotation.z, w=key.rotation.w),
            _xyz(key.translation))) for key in message.keyframes)

    def is_map(self, frame):
        return frame_ids_semantically_equal(frame, self.frame)

    def at_time(self, stamp):
        return min(self.keyframes, key=lambda key: abs(key[0] - stamp))[2]

    def at_pose(self, pose):
        if pose.source_timestamp_ns is not None:
            return self.at_time(pose.source_timestamp_ns)
        position = np.asarray(_xyz(pose.position))
        return min(self.keyframes, key=lambda key: ((key[1] - position) ** 2).sum())[2]

    def pose(self, pose, *, derived=None):
        if pose is None or not self.is_map(pose.frame_id) or pose.session_id not in (0, self.session_id):
            return pose
        return (derived or self.at_pose(pose)).pose(pose, self.seq)

    def attributes(self, attributes, pose):
        corrected = dict(attributes)
        if attributes.get("observer_body_pose"):
            observer = Pose.model_validate(attributes["observer_body_pose"])
            corrected["observer_body_pose"] = self.pose(observer).model_dump(mode="json")
        source = attributes.get(SAMPLE_SOURCE)
        if source is not None and source["session_id"] != self.session_id:
            return corrected
        motion = self.at_time(source["source_timestamp_ns"]) if source is not None else self.at_pose(pose)
        for name in ("map_points", "support_points_map", "point_samples_map", "map_point", "point_sample_map"):
            value = attributes.get(name)
            if isinstance(value, dict) and not self.is_map(value.get("frame_id") or pose.frame_id):
                continue
            if isinstance(value, dict) and "points" in value:
                corrected[name] = {**value, "points": [_like(point, motion.point(point)) for point in value["points"]]}
            elif isinstance(value, dict) and "x" in value:
                corrected[name] = {**value, **_point(motion.point(value))}
            elif isinstance(value, (tuple, list)) and value:
                corrected[name] = (motion.point(value).tolist() if np.isscalar(value[0])
                                   else [_like(point, motion.point(point)) for point in value])
        if attributes.get("centroid_map") is not None:
            value = attributes["centroid_map"]
            corrected["centroid_map"] = _like(value, motion.point(value))
        prefix = "centroid_map_"
        if prefix + "x" in attributes and prefix + "y" in attributes:
            moved = motion.point([attributes.get(prefix + axis, 0.0) for axis in _AXES])
            corrected.update({prefix + axis: value for axis, value in zip(_AXES, moved.tolist())})
        if "min_map_x" in attributes and "max_map_x" in attributes:
            lower, upper = motion.bounds(*([attributes[prefix + axis] for axis in _AXES]
                                           for prefix in ("min_map_", "max_map_")))
            corrected.update({"min_map_" + axis: lower[axis] for axis in _AXES})
            corrected.update({"max_map_" + axis: upper[axis] for axis in _AXES})
            corrected.update({"extent_" + axis + "_m": upper[axis] - lower[axis] for axis in _AXES})
        for name in ("bbox_3d_map", "object_3d_bbox", "support_cluster_summary"):
            value = attributes.get(name)
            if isinstance(value, dict) and "min" in value and "max" in value and self.is_map(value.get("frame_id") or pose.frame_id):
                lower, upper = motion.bounds(value["min"], value["max"])
                updates = {"min": _like(value["min"], np.asarray(_xyz(lower))),
                    "max": _like(value["max"], np.asarray(_xyz(upper)))}
                if "center" in value:
                    updates["center"] = _like(value["center"], motion.point(value["center"]))
                if "extent_m" in value:
                    updates["extent_m"] = {axis: upper[axis] - lower[axis] for axis in _AXES}
                corrected[name] = {**value, **updates}
        value = attributes.get("object_extent")
        if value and value.get("orientation") and self.is_map(value["frame_id"]):
            corrected["object_extent"] = motion.extent(value)
        if attributes.get("support_voxels_map"):
            support = InstanceSupportEvidence.model_validate(attributes["support_voxels_map"])
            if self.is_map(support.frame_id):
                corrected["support_voxels_map"] = self.voxels(support, motion).model_dump(mode="json")
        return corrected

    def voxels(self, support, motion):
        result = accumulate(None, motion.point(voxel_centers(support)), support.frame_id, self.config)
        return support.model_copy(update={"voxel_keys": result.voxel_keys})

    def projected(self, item):
        pose = self.pose(item.pose)
        if pose is item.pose:
            return item
        return replace(item, pose=pose, attributes=self.attributes(item.attributes, item.pose),
            observer_body_pose=self.pose(item.observer_body_pose))

    def instance(self, record):
        if not self.is_map(record.pose.frame_id) or not record.correction_buckets:
            return record, None
        motions = {}
        buckets = []
        for bucket in record.correction_buckets:
            if bucket.session_id == self.session_id and self.is_map(bucket.pose.frame_id):
                motion = self.at_time(bucket.start_ns + round(bucket.mean_offset_ns))
                motions[bucket.session_id, bucket.start_ns] = motion
                bucket = bucket.model_copy(update={"pose": motion.pose(bucket.pose, self.seq),
                    "support": self.voxels(bucket.support, motion)})
            buckets.append(bucket)
        position, evidence = reduce_buckets(buckets, record.pose, record.support_evidence, self.config)
        old = record.pose
        rotation = motions.get(bucket_key(old))
        orientation = rotation.rotation if rotation is not None else Quaternion()
        rotated_center = rotation.direction(old.position) if rotation is not None else np.asarray(_xyz(old.position))
        derived = Rigid(orientation, np.asarray(_xyz(position)) - rotated_center)
        pose = derived.pose(old, self.seq)
        window = []
        for sample in record.observation_window:
            metadata = dict(sample.metadata)
            if metadata.get("frame_geometry"):
                metadata["frame_geometry"] = [self.frame_geometry(item) for item in metadata["frame_geometry"]]
            window.append(sample.model_copy(update={"pose": self.pose(sample.pose), "metadata": metadata}))
        metadata = self.attributes(record.metadata, old)
        if "observation_window" in metadata:
            metadata["observation_window"] = [sample.model_dump(mode="json") for sample in window]
        extent = support_extent(evidence, pose=pose)
        if extent is not None:
            metadata["object_extent"] = extent
        elif record.metadata.get("object_extent"):
            metadata["object_extent"] = derived.extent(record.metadata["object_extent"])
        return record.model_copy(update={"pose": pose, "support_evidence": evidence, "correction_buckets": buckets,
            "attributes": self.attributes(record.attributes, old), "metadata": metadata,
            "inspection_poses": [self.pose(item, derived=derived) for item in record.inspection_poses],
            "observation_window": window}), derived

    def frame_geometry(self, value):
        if value is None:
            return value
        pose = Pose.model_validate(value["pose"])
        corrected = self.pose(pose)
        if corrected is pose:
            return value
        return {**value, "pose": corrected.model_dump(mode="json"),
            "attributes": self.attributes(value["attributes"], pose)}

    def scene_sample(self, sample, derived):
        motion = derived.get(sample.metadata.get("source_instance_id"))
        if sample.pose is None or (motion is None and sample.pose.session_id != self.session_id):
            return sample
        return sample.model_copy(update={"pose": self.pose(sample.pose, derived=motion)})

    def scene(self, record, derived):
        updates = {"observation_samples": [self.scene_sample(sample, derived) for sample in record.observation_samples]}
        adopted = record.metadata.get("adopted_label_evidence")
        if adopted:
            updates["metadata"] = {**record.metadata, "adopted_label_evidence": {**adopted, "observation_samples": [
                self.scene_sample(SceneObservationSample.model_validate(sample), derived).model_dump(mode="json")
                for sample in adopted["observation_samples"]]}}
        boundary = record.boundary
        if record.metadata.get("source") != "room_segmentation" and self.is_map(record.frame_id):
            for field in ("entry_poses", "inspection_poses"):
                updates[field] = [self.pose(pose) for pose in getattr(record, field)]
            if boundary is not None and boundary.centroid is not None and self.is_map(boundary.frame_id):
                motion = self.at_pose(boundary.centroid)
                changes = {"centroid": self.pose(boundary.centroid),
                    "polygon_points": [Vector3(**_point(motion.point(point))) for point in boundary.polygon_points]}
                if boundary.min_corner is not None and boundary.max_corner is not None:
                    lower, upper = motion.bounds(boundary.min_corner, boundary.max_corner)
                    changes.update(min_corner=Vector3(**lower), max_corner=Vector3(**upper))
                updates["boundary"] = boundary.model_copy(update=changes)
        return record.model_copy(update=updates)

    def apply(self, store):
        derived = {}
        for record in store.records("instance"):
            if record.metadata.get(SESSION_KEY) == self.session_id:
                corrected, motion = self.instance(record)
                if motion is not None:
                    for key in (record.instance_id, *record.merged_instance_ids):
                        derived[key] = motion
                    store.put_live_instance(corrected, persist=store.has("instance", record.instance_id))
        for kind in ("location", "memory", "observation", "trajectory", "scene_space"):
            for record in store.records(kind):
                if record.metadata.get(SESSION_KEY) != self.session_id:
                    continue
                if kind == "scene_space":
                    store.put(kind, self.scene(record, derived))
                    continue
                pose = self.pose(record.pose)
                changes = {"pose": pose}
                if kind == "trajectory":
                    changes["target_pose"] = self.pose(record.target_pose, derived=derived.get(record.source_record_id))
                if kind == "observation" and pose is not record.pose:
                    changes["metadata"] = self.attributes(record.metadata, record.pose)
                    old, new = record.pose.position, pose.position
                    changes["summary"] = record.summary.replace(f"在地图位置 {old.x:.2f}, {old.y:.2f}",
                        f"在地图位置 {new.x:.2f}, {new.y:.2f}", 1)
                store.put(kind, record.model_copy(update=changes))
