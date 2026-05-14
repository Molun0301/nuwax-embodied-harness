from __future__ import annotations

from collections import deque
import time

from contracts.base import utc_now
from contracts.events import RuntimeEventCategory
from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.geometry import Pose
from contracts.runtime_views import LocalizationSession, LocalizationSessionStatus, LocalizationSnapshot, RelocalizationCandidate
from core import EventBus, StateNamespace, StateStore, StreamBus
from gateways.errors import GatewayError
from providers import LocalizationProvider
from typing import Deque, Dict, Optional, Tuple

from .state_machine import (
    LocalizationStateMachine,
    LocalizationDiagnostics,
    create_localization_state_machine,
)


class LocalizationService:
    """统一定位与 TF 快照服务。

    平台定位运行时核心，管理 LocalizationSession 生命周期：
    - activate_map -> ensure_active_session -> relocalize -> READY -> navigation available
    - 支持 STALE（跟踪丢失可恢复）和 FAILED（定位失败）状态
    - 数据源来自平台几何运行时，不直接依赖 Go2 数据面
    """

    def __init__(
        self,
        *,
        provider_owner,
        state_store: StateStore,
        event_bus: Optional[EventBus] = None,
        stream_bus: Optional[StreamBus] = None,
        history_limit: int = 100,
        state_machine: Optional[LocalizationStateMachine] = None,
    ) -> None:
        self._provider_owner = provider_owner
        self._state_store = state_store
        self._event_bus = event_bus
        self._stream_bus = stream_bus
        self._history: Deque[LocalizationSnapshot] = deque(maxlen=max(1, history_limit))
        self._session_history: Deque[LocalizationSession] = deque(maxlen=max(1, history_limit))
        self._latest_snapshot: Optional[LocalizationSnapshot] = None
        self._active_session: Optional[LocalizationSession] = None
        # 平台定位状态机
        self._state_machine: LocalizationStateMachine = state_machine or create_localization_state_machine()
        self._last_diagnostics: Optional[LocalizationDiagnostics] = None
        self._localization_update_subscription_id: Optional[object] = None
        self._odom_publisher_failure_message: Optional[str] = None
        self._subscribe_provider_localization_updates()
        self._subscribe_odom_publisher_failures()

    def refresh(self) -> LocalizationSnapshot:
        """从本地定位提供器刷新定位快照。"""

        provider = self._get_localization_provider()
        current_pose, frame_tree = self._capture_provider_data(provider)
        return self.ingest_external_snapshot(
            source_name=provider.provider_name,
            current_pose=current_pose,
            frame_tree=frame_tree,
            metadata={"source_kind": "provider", "provider_version": provider.provider_version},
        )

    def _subscribe_provider_localization_updates(self) -> None:
        subscription = getattr(self._provider_owner, "subscribe_localization_updates", None)
        if callable(subscription):
            self._localization_update_subscription_id = subscription(self._handle_provider_localization_update)
            if self._localization_update_subscription_id is not None:
                return
        providers = getattr(self._provider_owner, "providers", None)
        subscription = getattr(providers, "subscribe_localization", None)
        if callable(subscription):
            self._localization_update_subscription_id = subscription(self._handle_provider_localization_update)

    def _subscribe_odom_publisher_failures(self) -> None:
        """订阅 odom publisher 故障事件，故障即把状态机置 stale。"""

        if self._event_bus is None:
            return
        try:
            self._event_bus.subscribe(
                self._on_odom_publisher_event,
                event_types=("geometry.odom_publisher_failed",),
            )
        except Exception:
            return

    def _on_odom_publisher_event(self, event) -> None:
        try:
            event_type = str(getattr(event, "event_type", "") or "")
            if event_type != "geometry.odom_publisher_failed":
                return
            self._odom_publisher_failure_message = str(getattr(event, "message", "") or "odom publisher failed")
            # 故障即丢弃位姿可用性，让状态机进入 stale
            self._state_machine.pose_received(pose_available=False, tracking_quality=0.0)
            self._state_machine.check_timeouts()
        except Exception:
            pass

    def has_provider_stream_subscription(self) -> bool:
        """返回定位服务是否已接入 provider 原生流。"""

        return self._localization_update_subscription_id is not None

    def _handle_provider_localization_update(
        self,
        current_pose,
        frame_tree=None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> Optional[LocalizationSnapshot]:
        update_metadata = dict(metadata or {})
        source_name = str(
            update_metadata.get("source_name")
            or update_metadata.get("provider_name")
            or "platform_localization_provider"
        )
        try:
            return self.ingest_external_snapshot(
                source_name=source_name,
                current_pose=current_pose,
                frame_tree=frame_tree,
                metadata=update_metadata,
            )
        except Exception:
            return None

    def ensure_active_session(
        self,
        *,
        map_name: str,
        map_version_id: str,
        frame_id: Optional[str] = None,
        metadata: Optional[Dict[str, object]] = None,
        geometry_asset: Optional[Dict[str, object]] = None,
    ) -> LocalizationSession:
        """确保当前存在绑定到指定地图版本的平台定位会话。"""

        resolved_geometry_asset = self._clone_metadata(geometry_asset) if geometry_asset is not None else None
        asset_failure = self._resolve_geometry_asset_failure(resolved_geometry_asset)
        existing = self._active_session
        if (
            existing is not None
            and existing.map_name == str(map_name or "").strip()
            and existing.map_version_id == str(map_version_id or "").strip()
            and (
                existing.frame_id == (str(frame_id or "").strip() or None)
                or (
                    existing.frame_id is not None
                    and str(frame_id or "").strip()
                    and frame_ids_semantically_equal(existing.frame_id, str(frame_id or "").strip())
                )
            )
        ):
            if resolved_geometry_asset is None or existing.geometry_asset == resolved_geometry_asset:
                return existing
            updated = existing.model_copy(
                update={
                    "geometry_asset": resolved_geometry_asset,
                    "status": LocalizationSessionStatus.FAILED if asset_failure else existing.status,
                    "last_error": asset_failure or existing.last_error,
                    "metadata": {
                        **dict(existing.metadata),
                        "geometry_asset_summary": resolved_geometry_asset,
                        **dict(metadata or {}),
                    },
                },
                deep=True,
            )
            self._set_active_session(updated)
            return updated
        session = LocalizationSession(
            session_id=self._build_session_id(map_name=map_name, map_version_id=map_version_id),
            map_name=str(map_name or "").strip(),
            map_version_id=str(map_version_id or "").strip(),
            frame_id=str(frame_id or "").strip() or None,
            status=LocalizationSessionStatus.FAILED if asset_failure else LocalizationSessionStatus.LOCALIZING,
            source_name="platform_localization_runtime",
            pose_available=False,
            geometry_asset=resolved_geometry_asset,
            last_error=asset_failure,
            metadata={
                **dict(metadata or {}),
                "geometry_asset_summary": resolved_geometry_asset,
            },
        )
        self._latest_snapshot = None
        self._set_active_session(session)
        return session

    def refresh_active_session(
        self,
        *,
        map_name: str,
        map_version_id: str,
        frame_id: Optional[str] = None,
        metadata: Optional[Dict[str, object]] = None,
        force_relocalize: bool = False,
        geometry_asset: Optional[Dict[str, object]] = None,
    ) -> LocalizationSession:
        """刷新当前平台定位会话，使用平台状态机管理定位生命周期。

        主链：
        1. 从平台几何运行时获取位姿和诊断数据（不直接依赖 Go2 数据面）
        2. 更新平台定位状态机
        3. 根据状态机状态更新 LocalizationSession
        4. 生成平台级 last_error 语义

        Args:
            map_name: 绑定地图名称
            map_version_id: 绑定地图版本
            frame_id: 目标坐标系
            metadata: 附加元数据
            force_relocalize: 是否强制触发重定位
        """

        session = self.ensure_active_session(
            map_name=map_name,
            map_version_id=map_version_id,
            frame_id=frame_id,
            metadata=metadata,
            geometry_asset=geometry_asset,
        )

        provider = self._get_localization_provider()
        provider_name = provider.provider_name

        # 从平台几何运行时获取位姿数据（数据源可能是 RTAB-Map 或 Go2 SDK，但通过平台 provider 层抽象）
        current_pose, frame_tree = self._capture_provider_data(provider)

        # 构建定位诊断信息
        diagnostics = self._build_localization_diagnostics(
            provider=provider,
            current_pose=current_pose,
            session_frame_id=session.frame_id,
        )
        self._last_diagnostics = diagnostics

        # 新会话首次刷新或显式重定位时，都应进入 LOCALIZING 周期。
        if force_relocalize or self._state_machine.current_state == "idle":
            self._state_machine.start()

        # 更新状态机并检查超时
        self._state_machine.check_timeouts()

        if current_pose is not None:
            # 位姿可用，根据跟踪质量更新状态机
            pose_available = True
            tracking_quality = diagnostics.tracking_quality
            backend_state = diagnostics.backend_state

            # 检查坐标系匹配
            if session.frame_id and not frame_ids_semantically_equal(current_pose.frame_id, session.frame_id):
                pose_available = False
                diagnostics.last_error = "坐标系不匹配：%s vs %s" % (current_pose.frame_id, session.frame_id)
                tracking_quality = 0.0

            self._state_machine.pose_received(
                pose_available=pose_available,
                tracking_quality=tracking_quality,
                backend_state=backend_state,
            )
        else:
            # 无位姿
            self._state_machine.pose_received(
                pose_available=False,
                tracking_quality=0.0,
                backend_state=diagnostics.backend_state,
            )

        # 再次检查超时
        self._state_machine.check_timeouts()

        # 获取最终状态机状态
        machine_state = self._state_machine.current_state

        # 映射到 LocalizationSessionStatus
        session_status = self._map_machine_state_to_session_status(machine_state)
        asset_failure = self._resolve_geometry_asset_failure(session.geometry_asset)
        if asset_failure:
            session_status = LocalizationSessionStatus.FAILED

        relocalization_candidates = self._build_relocalization_candidates(
            diagnostics=diagnostics,
            current_pose=current_pose,
            geometry_asset=session.geometry_asset,
            session_status=session_status,
        )
        last_relocalization = relocalization_candidates[-1] if relocalization_candidates else None

        # 构建 last_error
        last_error = asset_failure or self._build_session_last_error(
            diagnostics=diagnostics,
            session_status=session_status,
        )

        # 构建快照
        usable_pose = current_pose if session_status == LocalizationSessionStatus.READY else None
        snapshot = self.ingest_external_snapshot(
            source_name="platform_localization:%s" % session.map_name,
            current_pose=usable_pose,
            frame_tree=frame_tree if usable_pose is not None else None,
            metadata={
                "source_kind": "platform_localization_session",
                "map_name": session.map_name,
                "platform_map_version_id": session.map_version_id,
                "localization_session_id": session.session_id,
                "provider_name": provider_name,
                "machine_state": machine_state,
                "diagnostics": diagnostics.model_dump(mode="json"),
                "relocalization_candidates": [item.model_dump(mode="json") for item in relocalization_candidates],
                "last_relocalization": last_relocalization.model_dump(mode="json") if last_relocalization is not None else None,
                "geometry_asset_summary": session.geometry_asset,
                "force_relocalize": force_relocalize,
                **dict(metadata or {}),
            },
        )

        # 更新会话
        updated = session.model_copy(
            update={
                "status": session_status,
                "source_name": provider_name,
                "pose_available": session_status == LocalizationSessionStatus.READY,
                "relocalization_candidates": relocalization_candidates,
                "last_relocalization": last_relocalization,
                "last_localized_at": utc_now() if session_status == LocalizationSessionStatus.READY else session.last_localized_at,
                "last_error": last_error,
                "metadata": {
                    **dict(session.metadata),
                    "machine_state": machine_state,
                    "asset_failure": asset_failure,
                    **dict(snapshot.metadata),
                },
            },
            deep=True,
        )
        self._set_active_session(updated)
        return updated

    def relocalize_active_session(
        self,
        *,
        map_name: str,
        map_version_id: str,
        frame_id: Optional[str] = None,
        metadata: Optional[Dict[str, object]] = None,
        geometry_asset: Optional[Dict[str, object]] = None,
    ) -> LocalizationSession:
        """对当前激活会话执行平台级重定位。

        正式主链：
        1. 重置状态机到 LOCALIZING 状态
        2. 根据后端模式真实驱动 SLAM/AMCL 重定位（slam_toolbox/clear_changes、AMCL /initialpose）
        3. 刷新会话并返回最新状态

        Returns:
            更新后的 LocalizationSession
        """

        # 触发状态机重定位（清 stale_since 计时、attempts++）
        self._state_machine.relocalize()

        # 真接入：根据当前 session/asset 解析后端模式，向 SLAM/AMCL 发出真实重定位指令
        backend_dispatch = self._dispatch_backend_relocalize(
            geometry_asset=geometry_asset,
            metadata=metadata,
        )

        merged_metadata: Dict[str, object] = {
            "relocalize": True,
            "backend_relocalize": backend_dispatch,
            **dict(metadata or {}),
        }
        return self.refresh_active_session(
            map_name=map_name,
            map_version_id=map_version_id,
            frame_id=frame_id,
            metadata=merged_metadata,
            force_relocalize=False,  # 状态机已处理重定位
            geometry_asset=geometry_asset,
        )

    def _dispatch_backend_relocalize(
        self,
        *,
        geometry_asset: Optional[Dict[str, object]],
        metadata: Optional[Dict[str, object]],
    ) -> Dict[str, object]:
        """根据后端模式向真实 SLAM/AMCL 发出重定位指令。"""

        backend_mode = self._resolve_backend_mode(
            geometry_asset=geometry_asset,
            metadata=metadata,
        )
        # 取最近 provider pose 作为 /initialpose hint；缺失时让后端用上次 known pose
        hint_pose: Optional[Pose] = None
        try:
            provider = self._find_localization_provider()
            if provider is not None:
                hint_pose = provider.get_current_pose()
        except Exception:
            hint_pose = None

        result: Dict[str, object] = {
            "backend_mode": backend_mode,
            "hint_pose_available": hint_pose is not None,
            "actions": [],
            "errors": [],
        }
        if backend_mode == "amcl_localization_only":
            self._publish_initialpose(hint_pose, result)
        elif backend_mode == "slam_toolbox":
            # slam_toolbox 在线建图模式下，发 clear_changes 让其重新接受新 keyframe；
            # 同时 /initialpose 让 AMCL（如同时启用）重新撒粒子。
            self._call_slam_toolbox_clear_changes(result)
            self._publish_initialpose(hint_pose, result)
        else:
            result["errors"].append(
                "未知后端模式 backend_mode=%s，跳过真接入；建议在 mapping 端写入 backend_mode 元数据。"
                % backend_mode
            )
        return result

    @staticmethod
    def _resolve_backend_mode(
        *,
        geometry_asset: Optional[Dict[str, object]],
        metadata: Optional[Dict[str, object]],
    ) -> str:
        """从 metadata / geometry_asset 解析后端模式。"""

        for source in (metadata or {}, geometry_asset or {}):
            value = str(source.get("backend_mode") or "").strip().lower()
            if value:
                return value
        # 兜底：external_slam_nav2 默认 slam_toolbox（在线 SLAM）。
        stack = str((geometry_asset or {}).get("geometry_stack") or "").strip().lower()
        if stack == "external_slam_nav2":
            return "slam_toolbox"
        return "unknown"

    def _publish_initialpose(self, hint_pose: Optional[Pose], result: Dict[str, object]) -> None:
        """向 AMCL `/initialpose` 发布初始位姿。"""

        if hint_pose is None:
            result["errors"].append("无可用 hint pose，跳过 /initialpose 发布。")
            return
        try:
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from services.geometry.ros2_context import acquire_ros2_context, release_ros2_context

            rclpy = acquire_ros2_context()
            node = rclpy.create_node("nuwax_localization_initialpose_publisher")
            try:
                qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                )
                pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos)
                msg = PoseWithCovarianceStamped()
                # AMCL 期望 frame=map
                msg.header.frame_id = "map"
                msg.pose.pose.position.x = float(hint_pose.position.x)
                msg.pose.pose.position.y = float(hint_pose.position.y)
                msg.pose.pose.position.z = float(hint_pose.position.z)
                msg.pose.pose.orientation.x = float(hint_pose.orientation.x)
                msg.pose.pose.orientation.y = float(hint_pose.orientation.y)
                msg.pose.pose.orientation.z = float(hint_pose.orientation.z)
                msg.pose.pose.orientation.w = float(hint_pose.orientation.w)
                # 单位对角协方差作为 hint（不强约束）
                msg.pose.covariance = [0.25 if i in (0, 7, 35) else 0.0 for i in range(36)]
                pub.publish(msg)
                result["actions"].append({"target": "/initialpose", "status": "published"})
            finally:
                node.destroy_node()
                release_ros2_context()
        except Exception as exc:
            result["errors"].append("/initialpose 发布失败：%s" % exc)

    def _call_slam_toolbox_clear_changes(self, result: Dict[str, object]) -> None:
        """调用 slam_toolbox `/slam_toolbox/clear_changes` service 清除累积修正。"""

        try:
            from slam_toolbox.srv import Clear  # type: ignore
            from services.geometry.ros2_context import acquire_ros2_context, release_ros2_context

            rclpy = acquire_ros2_context()
            node = rclpy.create_node("nuwax_localization_slam_toolbox_client")
            try:
                client = node.create_client(Clear, "/slam_toolbox/clear_changes")
                if not client.wait_for_service(timeout_sec=2.0):
                    result["errors"].append("/slam_toolbox/clear_changes service 未就绪。")
                    return
                future = client.call_async(Clear.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
                if future.done() and future.result() is not None:
                    result["actions"].append({"target": "/slam_toolbox/clear_changes", "status": "called"})
                else:
                    result["errors"].append("/slam_toolbox/clear_changes 调用未返回结果。")
            finally:
                node.destroy_node()
                release_ros2_context()
        except Exception as exc:
            result["errors"].append("/slam_toolbox/clear_changes 调用失败：%s" % exc)

    def get_localization_diagnostics(self) -> Optional[LocalizationDiagnostics]:
        """返回当前平台定位诊断信息。"""

        return self._last_diagnostics

    def _build_relocalization_candidates(
        self,
        *,
        diagnostics: LocalizationDiagnostics,
        current_pose: Optional[object],
        geometry_asset: Optional[Dict[str, object]],
        session_status: LocalizationSessionStatus,
    ) -> Tuple[RelocalizationCandidate, ...]:
        metadata = dict(diagnostics.metadata or {})
        backend_metadata = metadata.get("backend_state") if isinstance(metadata.get("backend_state"), dict) else {}
        raw_candidates = (
            backend_metadata.get("relocalization_candidates")
            or metadata.get("relocalization_candidates")
            or backend_metadata.get("icp_candidates")
            or metadata.get("icp_candidates")
            or []
        )
        candidates = []
        if isinstance(raw_candidates, list):
            for item in raw_candidates:
                if isinstance(item, dict):
                    candidates.append(self._candidate_from_metadata(item, geometry_asset=geometry_asset))
        if candidates:
            return tuple(candidates)
        backend = str(metadata.get("backend_type") or diagnostics.backend_name or "unknown")
        quality_metadata = backend_metadata.get("quality") if isinstance(backend_metadata.get("quality"), dict) else {}
        failure_reason = diagnostics.last_error if session_status != LocalizationSessionStatus.READY else None
        return (
            RelocalizationCandidate(
                backend=backend,
                pose=current_pose if isinstance(current_pose, Pose) and session_status == LocalizationSessionStatus.READY else None,
                fitness=self._optional_float(
                    backend_metadata.get("fitness")
                    or backend_metadata.get("icp_fitness")
                    or quality_metadata.get("fitness")
                ),
                inlier_ratio=self._optional_float(
                    backend_metadata.get("inlier_ratio")
                    or backend_metadata.get("inlier")
                    or backend_metadata.get("icp_inlier_ratio")
                    or quality_metadata.get("inlier_ratio")
                ),
                covariance=self._resolve_candidate_covariance(backend_metadata),
                asset_refs=self._resolve_candidate_asset_refs(geometry_asset),
                failure_reason=failure_reason,
                metadata={
                    "status": session_status.value,
                    "backend_state": diagnostics.backend_state,
                    "quality_unknown": True,
                    "diagnostics": diagnostics.model_dump(mode="json"),
                },
            ),
        )

    def _candidate_from_metadata(
        self,
        item: Dict[str, object],
        *,
        geometry_asset: Optional[Dict[str, object]],
    ) -> RelocalizationCandidate:
        return RelocalizationCandidate(
            backend=str(item.get("backend") or item.get("backend_type") or item.get("source") or "unknown"),
            pose=item.get("pose") if isinstance(item.get("pose"), Pose) else None,
            fitness=self._optional_float(item.get("fitness") or item.get("icp_fitness")),
            inlier_ratio=self._optional_float(item.get("inlier_ratio") or item.get("inlier")),
            covariance=self._resolve_candidate_covariance(item),
            asset_refs=self._resolve_candidate_asset_refs(geometry_asset, item.get("asset_refs")),
            failure_reason=str(item.get("failure_reason") or item.get("last_error") or "").strip() or None,
            metadata=self._clone_metadata(item),
        )

    def _resolve_geometry_asset_failure(self, geometry_asset: Optional[Dict[str, object]]) -> Optional[str]:
        if not geometry_asset:
            return None
        quality_metrics = geometry_asset.get("quality_metrics")
        if not isinstance(quality_metrics, dict):
            return None
        is_external_stack = str(geometry_asset.get("geometry_stack") or "").strip().lower() == "external_slam_nav2"
        database = quality_metrics.get("database")
        database_missing = isinstance(database, dict) and database.get("present") is False
        database_required = bool(database.get("required", not is_external_stack)) if isinstance(database, dict) else True
        optional_database_missing = bool(database_missing and not database_required)
        if quality_metrics.get("status") == "failed":
            for item in quality_metrics.get("diagnostics") or []:
                if isinstance(item, dict) and item.get("level") == "error":
                    code = str(item.get("code") or "").strip()
                    if optional_database_missing and code in {"external_geometry_state_missing", "external_geometry_state_not_materialized"}:
                        continue
                    return str(item.get("message") or item.get("code") or "几何资产不可用。")
            if optional_database_missing:
                return None
            return "几何资产不可用。"
        if database_missing and database_required:
            return "地图版本缺少 RTAB-Map database 资产。"
        return None

    def _resolve_candidate_asset_refs(
        self,
        geometry_asset: Optional[Dict[str, object]],
        explicit_refs: Optional[object] = None,
    ) -> list[Dict[str, object]]:
        if isinstance(explicit_refs, list):
            return [self._clone_metadata(item) for item in explicit_refs if isinstance(item, dict)]
        if not geometry_asset:
            return []
        assets = geometry_asset.get("assets")
        if isinstance(assets, list):
            return [self._clone_metadata(item) for item in assets if isinstance(item, dict)]
        is_external_stack = str(geometry_asset.get("geometry_stack") or "").strip().lower() == "external_slam_nav2"
        # external_slam_nav2 真实后端是 slam_toolbox。
        state_asset_type = "slam_toolbox_posegraph" if is_external_stack else "rtabmap_database"
        refs = []
        # 优先读 backend_state_path（新命名），缺失时 fallback 到老 rtabmap_db_path（兼容老 version.json）。
        backend_path = (
            str(geometry_asset.get("backend_state_path") or "").strip()
            or str(geometry_asset.get("rtabmap_db_path") or "").strip()
        )
        if backend_path:
            refs.append({"asset_type": state_asset_type, "path": backend_path, "frame_id": geometry_asset.get("frame_id")})
        pcd_path = str(geometry_asset.get("pcd_path") or "").strip()
        if pcd_path:
            refs.append({"asset_type": "pcd", "path": pcd_path, "frame_id": geometry_asset.get("frame_id")})
        return refs

    def _resolve_candidate_covariance(self, metadata: Dict[str, object]) -> Optional[Dict[str, object]]:
        covariance = metadata.get("covariance") or metadata.get("pose_covariance")
        if isinstance(covariance, dict):
            return self._clone_metadata(covariance)
        if covariance is not None:
            return {"value": self._clone_metadata(covariance)}
        return {"status": "unknown"}

    @staticmethod
    def _optional_float(value: object) -> Optional[float]:
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _build_localization_diagnostics(
        self,
        provider: LocalizationProvider,
        current_pose: Optional[object],
        session_frame_id: Optional[str],
    ) -> LocalizationDiagnostics:
        """构建平台定位诊断信息。"""

        from adapters.localization.geometry_state_adapter import (
            BackendLocalizationState,
            compute_tracking_quality,
        )

        backend_name = provider.provider_name

        # 尝试获取后端的原始状态
        backend_state = self._capture_backend_state(provider)

        # 坐标系匹配检查
        frame_id_match = True
        if current_pose is not None and session_frame_id is not None:
            frame_id_match = frame_ids_semantically_equal(current_pose.frame_id, session_frame_id)

        # 构建后端状态对象
        state = BackendLocalizationState(
            backend_name=backend_name,
            backend_type=self._resolve_backend_type(provider),
            pose_available=current_pose is not None,
            tracking_state=backend_state.get("tracking_state", "unknown"),
            map_loaded=backend_state.get("map_loaded", False),
            frame_id_match=frame_id_match,
            quality_score=backend_state.get("quality_score", 0.0),
            loop_closure_detected=backend_state.get("loop_closure_detected", False),
            relocalization_in_progress=backend_state.get("relocalization_in_progress", False),
            last_error=backend_state.get("last_error"),
            metadata=backend_state,
        )

        # 计算跟踪质量
        tracking_quality = compute_tracking_quality(state)

        return LocalizationDiagnostics(
            backend_name=backend_name,
            pose_available=current_pose is not None,
            tracking_quality=tracking_quality,
            time_since_last_pose_sec=0.0,  # 由状态机计算
            backend_state=state.tracking_state,
            tracking_lost_count=0,  # 由状态机管理
            relocalization_attempts=self._state_machine._relocalization_attempts if hasattr(self._state_machine, '_relocalization_attempts') else 0,
            last_error=state.last_error,
            metadata={
                "backend_type": state.backend_type,
                "loop_closure": state.loop_closure_detected,
                "relocalizing": state.relocalization_in_progress,
                "backend_state": self._clone_metadata(backend_state),
            },
        )

    def _capture_backend_state(
        self,
        provider: LocalizationProvider,
    ) -> Dict[str, object]:
        """捕获后端定位状态（RTAB-Map 或 Go2 SDK）。"""

        # 尝试获取后端状态方法
        get_state = getattr(provider, "get_localization_state", None)
        if callable(get_state):
            try:
                state = get_state()
                if isinstance(state, dict):
                    return state
            except Exception:
                pass

        # 尝试获取原始诊断信息
        get_diagnostics = getattr(provider, "get_localization_diagnostics", None)
        if callable(get_diagnostics):
            try:
                diag = get_diagnostics()
                if hasattr(diag, "model_dump"):
                    return diag.model_dump(mode="json")
                if isinstance(diag, dict):
                    return diag
            except Exception:
                pass

        # 返回默认状态
        return {
            "tracking_state": "unknown",
            "map_loaded": False,
            "quality_score": 0.0,
            "last_error": None,
        }

    def _resolve_backend_type(self, provider: LocalizationProvider) -> str:
        """解析后端类型。

        历史命名 `rtabmap` 已退场；当前真实在线 SLAM 后端是 `slam_toolbox`。
        provider_name 同时匹配 `slam_toolbox` 与历史 `rtabmap` 字面值，统一归一为
        `slam_toolbox`。
        """

        backend_type = getattr(provider, "backend_type", None)
        if backend_type:
            normalized = str(backend_type).strip().lower()
            # 兼容存量 backend_type="rtabmap" 写入，统一对外语义为 slam_toolbox。
            if normalized == "rtabmap":
                return "slam_toolbox"
            return str(backend_type)

        provider_name = str(getattr(provider, "provider_name", "") or "").lower()
        if "slam_toolbox" in provider_name or "rtabmap" in provider_name:
            return "slam_toolbox"
        if "go2" in provider_name:
            return "go2_sdk"
        if "nav2" in provider_name:
            return "nav2"

        return "unknown"

    def _map_machine_state_to_session_status(
        self,
        machine_state: str,
    ) -> LocalizationSessionStatus:
        """将状态机状态映射到 LocalizationSessionStatus。"""

        status_map = {
            "idle": LocalizationSessionStatus.IDLE,
            "localizing": LocalizationSessionStatus.LOCALIZING,
            "ready": LocalizationSessionStatus.READY,
            "stale": LocalizationSessionStatus.STALE,
            "failed": LocalizationSessionStatus.FAILED,
        }
        return status_map.get(machine_state, LocalizationSessionStatus.LOCALIZING)

    def _build_session_last_error(
        self,
        diagnostics: LocalizationDiagnostics,
        session_status: LocalizationSessionStatus,
    ) -> Optional[str]:
        """构建会话级别 last_error 语义。"""

        if diagnostics.last_error:
            return diagnostics.last_error

        if session_status == LocalizationSessionStatus.STALE:
            return "跟踪丢失，尝试恢复中..."

        if session_status == LocalizationSessionStatus.FAILED:
            return "定位失败，需要重新激活地图或重定位。"

        if session_status == LocalizationSessionStatus.LOCALIZING:
            return "定位中..."

        return None

    def refresh_current_session(
        self,
        *,
        metadata: Optional[Dict[str, object]] = None,
    ) -> Optional[LocalizationSession]:
        """刷新当前已存在的平台定位会话。"""

        session = self._active_session
        if session is None:
            return None
        return self.refresh_active_session(
            map_name=session.map_name,
            map_version_id=session.map_version_id,
            frame_id=session.frame_id,
            metadata=metadata,
            geometry_asset=session.geometry_asset,
        )

    def get_active_session(self) -> Optional[LocalizationSession]:
        """返回当前激活的平台定位会话。"""

        return self._active_session

    def clear_active_session(self, *, reason: str = "") -> None:
        """清空当前激活的平台定位会话。"""

        self._active_session = None
        self._state_machine.reset()
        self._last_diagnostics = None
        self._state_store.write(
            StateNamespace.LOCALIZATION,
            "active_session",
            None,
            source="localization_service",
            metadata={"kind": "localization_session", "reason": str(reason or "").strip() or None},
        )

    def ingest_external_snapshot(
        self,
        *,
        source_name: str,
        current_pose=None,
        frame_tree=None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> LocalizationSnapshot:
        """写入来自适配器或外部系统的定位快照。"""

        snapshot = LocalizationSnapshot(
            source_name=source_name,
            current_pose=current_pose,
            frame_tree=frame_tree,
            metadata=dict(metadata or {}),
        )
        self._latest_snapshot = snapshot
        self._history.append(snapshot)
        self._state_store.write(
            StateNamespace.LOCALIZATION,
            source_name,
            snapshot,
            source=source_name,
            metadata={"kind": "latest_localization"},
        )

        if self._event_bus is not None:
            self._event_bus.publish(
                self._event_bus.build_event(
                    "localization.snapshot_ready",
                    category=RuntimeEventCategory.ROBOT,
                    source="localization_service",
                    message="定位快照已更新。",
                    payload={"source_name": source_name},
                )
            )
        if self._stream_bus is not None:
            if current_pose is not None:
                self._stream_bus.publish(
                    "localization.pose",
                    source="localization_service",
                    payload=current_pose,
                    frame_id=getattr(current_pose, "frame_id", None),
                    status="available",
                    metadata={"source_name": source_name},
                )
            if frame_tree is not None:
                self._stream_bus.publish(
                    "localization.frame_tree",
                    source="localization_service",
                    payload=frame_tree,
                    status="available",
                    metadata={"source_name": source_name},
                )
            self._stream_bus.publish(
                "localization.trajectory",
                source="localization_service",
                payload={"trajectory": [item.current_pose for item in self._history if item.current_pose is not None]},
                frame_id=getattr(current_pose, "frame_id", None),
                status="available" if current_pose is not None else "missing",
                metadata={"source_name": source_name, "history_size": len(self._history)},
            )
        return snapshot

    def get_latest_snapshot(self) -> Optional[LocalizationSnapshot]:
        """返回最新定位快照。"""

        return self._latest_snapshot

    def clear_latest_snapshot(self) -> None:
        """清空当前最新定位快照。"""

        self._latest_snapshot = None

    def list_session_history(self, *, limit: Optional[int] = None) -> Tuple[LocalizationSession, ...]:
        """返回定位会话历史。"""

        items = list(self._session_history)
        if limit is not None:
            items = items[-max(0, limit) :]
        return tuple(items)

    def list_history(self, *, limit: Optional[int] = None) -> Tuple[LocalizationSnapshot, ...]:
        """返回定位历史。"""

        items = list(self._history)
        if limit is not None:
            items = items[-max(0, limit) :]
        return tuple(items)

    def is_available(self) -> bool:
        """判断定位服务当前是否可用。"""

        if self._latest_snapshot is not None:
            return True
        provider = self._find_localization_provider()
        if provider is None or not provider.is_available():
            return False
        availability_checker = getattr(provider, "is_localization_available", None)
        if callable(availability_checker):
            return bool(availability_checker())
        return True

    def _get_localization_provider(self) -> LocalizationProvider:
        provider = self._find_localization_provider()
        if provider is None:
            raise GatewayError("当前平台几何运行时未提供定位提供器。")
        if not provider.is_available():
            raise GatewayError("当前定位提供器暂不可用。")
        return provider

    def _capture_provider_data(self, provider: LocalizationProvider):
        return provider.get_current_pose(), provider.get_frame_tree()

    @classmethod
    def _clone_metadata(cls, value):
        if hasattr(value, "model_dump"):
            return value.model_dump(mode="json")
        if isinstance(value, dict):
            return {key: cls._clone_metadata(inner) for key, inner in value.items()}
        if isinstance(value, list):
            return [cls._clone_metadata(item) for item in value]
        if isinstance(value, tuple):
            return [cls._clone_metadata(item) for item in value]
        return value

    def _set_active_session(self, session: LocalizationSession) -> LocalizationSession:
        self._active_session = session
        self._session_history.append(session)
        self._state_store.write(
            StateNamespace.LOCALIZATION,
            "active_session",
            session,
            source="localization_service",
            metadata={
                "kind": "localization_session",
                "map_name": session.map_name,
                "map_version_id": session.map_version_id,
                "status": session.status.value,
            },
        )
        if self._event_bus is not None:
            self._event_bus.publish(
                self._event_bus.build_event(
                    "localization.session_updated",
                    category=RuntimeEventCategory.ROBOT,
                    source="localization_service",
                    message="平台定位会话已更新。",
                    payload={
                        "map_name": session.map_name,
                        "map_version_id": session.map_version_id,
                        "status": session.status.value,
                    },
                )
            )
        return session

    def _build_session_id(self, *, map_name: str, map_version_id: str) -> str:
        timestamp = int(utc_now().timestamp() * 1000)
        return "locsess_%s_%s_%d" % (
            str(map_name or "").strip(),
            str(map_version_id or "").strip(),
            timestamp,
        )

    def _find_localization_provider(self) -> Optional[LocalizationProvider]:
        providers = getattr(self._provider_owner, "providers", None)
        if providers is None or not isinstance(providers, LocalizationProvider):
            return None
        return providers
