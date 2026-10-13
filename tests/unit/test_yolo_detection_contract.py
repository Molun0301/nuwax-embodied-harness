"""检测契约、模型调用与预热的离线回归；不加载真实模型。"""
from types import SimpleNamespace
import threading

import cv2
import numpy as np
import pytest

from contracts.image import ImageEncoding, ImageFrame
from core.errors import GatewayError
from services.perception.base import DetectionBundle
from services.perception.detectors import YoloWorldDetectorBackend, _WorkerResult, _clamp_score, normalize_detection_2d
from services.perception.image_utils import decode_image_frame_to_bgr
from services.perception.prewarm import prewarm_yolo_world, start_yolo_world_worker


def _frame(*, metadata=None, width=100, height=100):
    return ImageFrame(camera_id="front", frame_id="camera/front", width_px=width, height_px=height,
                      encoding=ImageEncoding.BGR8, data=bytes(width * height * 3), metadata=metadata or {})


def _result(*, names=None, boxes=None, scores=None, classes=None):
    return SimpleNamespace(
        boxes=SimpleNamespace(xyxy=boxes if boxes is not None else [[1, 2, 10, 20]],
                              conf=scores if scores is not None else [.9],
                              cls=classes if classes is not None else [0]),
        names=names, masks=None,
    )


def _prewarm_config():
    return SimpleNamespace(perception=SimpleNamespace(yolo_world=SimpleNamespace(
        enabled=True, backend_name="yolo_world", model_name="model.engine", image_size=8,
    )))


def test_partial_box_clipping_preserves_actual_intersection():
    detection = normalize_detection_2d({"label": "chair", "score": .8,
        "bbox": {"x_px": -10, "y_px": -5, "width_px": 20, "height_px": 15}}, _frame())
    assert detection.bbox.x_px == detection.bbox.y_px == 0
    assert detection.bbox.width_px == detection.bbox.height_px == 10
    subpixel = normalize_detection_2d({"label": "chair", "bbox": {"x_px": 1, "y_px": 1, "width_px": .3, "height_px": .4}}, _frame())
    assert subpixel.bbox.width_px == pytest.approx(.3)
    assert subpixel.bbox.height_px == pytest.approx(.4)


@pytest.mark.parametrize("bbox", [
    {"x_px": 120, "y_px": 10, "width_px": 20, "height_px": 10},
    {"x_px": float("nan"), "y_px": 10, "width_px": 20, "height_px": 10},
    {"x_px": 10, "y_px": 10, "width_px": -1, "height_px": 10},
])
def test_invalid_boxes_do_not_turn_into_border_detections(bbox):
    with pytest.raises(GatewayError) as raised:
        normalize_detection_2d({"label": "chair", "bbox": bbox}, _frame())
    assert raised.value.error_code == "invalid_detection_bbox"
    assert _clamp_score(float("nan")) == 0
    assert _clamp_score(float("inf")) == 0


@pytest.mark.parametrize("names,expected_label", [(["Chair"], "chair"), ({"0": "Chair"}, "chair"), (None, "class_0")])
def test_yolo_normalization_uses_returned_names_without_loading_model(monkeypatch, names, expected_label):
    backend = YoloWorldDetectorBackend(use_worker_process=False)
    monkeypatch.setattr(backend, "_get_model", lambda: pytest.fail("归一化不应加载模型"))
    result = _result(names=names,
        boxes=[[-10, -5, 20, 15], [120, 10, 130, 20], [10, 10, 5, 20], [float("nan"), 1, 20, 20], [1, 1, 5, 5], [1, 1, 5, 5], ["bad", 1, 5, 5]],
        scores=[.9, .9, .9, .9, float("nan"), .9, .9], classes=[0, 0, 0, 0, 0, "bad", 0])
    detections = backend._normalize_yolo_result(result, _frame())
    assert len(detections) == 1
    detection = detections[0]
    assert detection.label == expected_label
    assert (detection.bbox.x_px, detection.bbox.y_px, detection.bbox.width_px, detection.bbox.height_px) == (0, 0, 20, 15)
    assert detection.attributes["relative_area"] == .03
    assert detection.attributes["center_x_ratio"] == .1
    assert detection.attributes["center_y_ratio"] == .075


def test_worker_result_preserves_list_class_table():
    result = _WorkerResult(np.asarray([[1, 2, 10, 20]]), np.asarray([.9]), np.asarray([0]), ["chair"], None)
    assert result.names == ["chair"]


def test_mask_quality_converts_each_tensor_once():
    class TensorValues:
        def __init__(self, values):
            self.values, self.calls = values, 0
        def tolist(self):
            self.calls += 1
            return list(self.values)
    values = [TensorValues([.7] * 200), TensorValues([.8] * 199), TensorValues([.9] * 200)]
    backend = YoloWorldDetectorBackend(use_worker_process=False)
    result = backend._extract_mask_quality(SimpleNamespace(predicted_iou=values[0], stability_score=values[1], scores=values[2]))
    assert [value.calls for value in values] == [1, 1, 1]
    assert result[0] == {"predicted_iou": .7, "stability_score": .8, "segmentation_score": .9}
    assert result[-1] == {"predicted_iou": .7, "segmentation_score": .9}


def test_yolo_mask_normalization_owns_output_without_copying_new_detection(monkeypatch):
    from contracts.perception import Detection2D

    backend = YoloWorldDetectorBackend(use_worker_process=False)
    polygon = np.asarray([[1., 2.], [10., 2.], [10., 20.], [1., 20.]])
    result = _result(names={0: "Chair"}, boxes=[[1, 2, 10, 20]] * 2,
                     scores=[.9, .8], classes=[0, 0])
    result.masks = SimpleNamespace(xy=[polygon, polygon])
    monkeypatch.setattr(Detection2D, "model_copy",
                        lambda *args, **kwargs: pytest.fail("新检测应在构造时附带分割，不再复制多边形"))

    first, second = backend._normalize_yolo_result(result, _frame())
    assert first.label == "chair"
    assert first.segmentation.polygon_px == ((1., 2.), (10., 2.), (10., 20.), (1., 20.))
    assert first.segmentation.area_px == 162.
    assert first.segmentation.quality.score == .9
    assert first.attributes["mask_center_px"] is first.attributes["centroid_uv"]
    first.attributes["mask_polygon_px"][0][0] = 99.
    assert polygon[0, 0] == 1.
    assert first.attributes["segmentation_polygon_px"][0][0] == 1.
    assert first.segmentation.polygon_px[0][0] == 1.
    assert second.attributes["mask_polygon_px"][0][0] == 1.
    polygon[0, 0] = 50.
    assert second.segmentation.polygon_px[0][0] == 1.
    assert second.attributes["mask_polygon_px"][0][0] == 1.


def test_compressed_image_shape_must_match_contract():
    image = np.zeros((4, 6, 3), dtype=np.uint8)
    ok, encoded = cv2.imencode(".png", image)
    assert ok
    frame = ImageFrame(camera_id="front", frame_id="camera/front", width_px=6, height_px=4,
                       encoding=ImageEncoding.PNG, data=encoded.tobytes())
    np.testing.assert_array_equal(decode_image_frame_to_bgr(frame), image)
    with pytest.raises(GatewayError) as raised:
        decode_image_frame_to_bgr(frame.model_copy(update={"width_px": 12}))
    assert raised.value.error_code == "image_dimensions_mismatch"


@pytest.mark.parametrize("weights,worker_enabled", [("model.pt", True), ("model.engine", False)])
def test_prewarm_does_not_create_unused_worker(monkeypatch, weights, worker_enabled):
    backend = YoloWorldDetectorBackend(weights=weights, use_worker_process=worker_enabled)
    monkeypatch.setattr("services.perception.inference_worker.YoloInferenceWorker", lambda *args, **kwargs: pytest.fail("该路径不应创建工作进程"))
    assert backend.start_inference_worker_nonblocking() is False
    status = start_yolo_world_worker(SimpleNamespace(get_backend=lambda name: backend), _prewarm_config())
    assert status["status"] == "starting"
    assert status["inference_execution"] == "in_process"


def test_dynamic_classes_are_applied_once_per_change_and_only_first_result_is_consumed():
    class Model:
        names = {0: "chair"}
        def __init__(self): self.class_calls = []
        def set_classes(self, classes):
            self.class_calls.append(classes)
            self.names = dict(enumerate(classes))
        def predict(self, image, **kwargs):
            yield _result(names=dict(self.names))
            raise AssertionError("单帧检测不应消费多余结果")
    model = Model()
    backend = YoloWorldDetectorBackend(weights="model.pt", class_names=("chair",), use_worker_process=False)
    backend._model = model
    for classes in [("chair",), ("chair",), ("bottle",)]:
        result = backend.detect(_frame(metadata={"detection_classes": classes}))
        assert result.metadata["availability"] == "available"
        assert result.metadata["class_prompt_applied"] is True
        assert result.detections_2d[0].label == classes[0]
    assert model.class_calls == [["chair"], ["bottle"]]


def test_fixed_engine_reports_actual_classes_and_rejects_unsupported_requests():
    class Model:
        names = ["chair", "table"]
        def set_classes(self, classes): pytest.fail("固定引擎不能设置动态词表")
        def predict(self, image, **kwargs): return [_result(names=self.names)]
    backend = YoloWorldDetectorBackend(weights="model.engine", use_worker_process=False)
    backend._model = Model()
    bundle = backend.detect(_frame(metadata={"detection_classes": ["dog"]}))
    assert bundle.detections_2d == ()
    assert bundle.metadata["reason"] == "unsupported_engine_classes"
    assert bundle.metadata["requested_classes"] == ["dog"]
    assert bundle.metadata["open_vocabulary_classes"] == ["chair", "table"]
    assert bundle.metadata["supports_dynamic_vocabulary"] is False
    assert bundle.metadata["class_prompt_applied"] is False
    assert bundle.metadata["class_prompt_limitation"] == "fixed_engine_vocabulary"
    assert bundle.metadata["inference_execution"] == "in_process"
    assert bundle.metadata["unsupported_requested_classes"] == ["dog"]


@pytest.mark.parametrize("metadata,expected", [
    ({"stage_status": "not_ready", "availability": "unavailable", "reason": "model_failed", "error": "missing weights"}, "deferred"),
    ({"stage_status": "completed", "availability": "available"}, "ready"),
])
def test_prewarm_obeys_detector_readiness_including_empty_success(metadata, expected):
    backend = SimpleNamespace(detect=lambda frame: DetectionBundle(metadata=metadata))
    result = prewarm_yolo_world(SimpleNamespace(get_backend=lambda name: backend), _prewarm_config())
    assert result["status"] == expected
    if expected == "deferred":
        assert result["reason"] == "model_failed"
        assert result["error"] == "missing weights"




def test_prewarm_and_detection_share_model_vocabulary_lock():
    entered, release, second_predict = threading.Event(), threading.Event(), threading.Event()
    outputs = {}
    class Model:
        names = {}
        def set_classes(self, classes): self.names = dict(enumerate(classes))
        def predict(self, image, **kwargs):
            if self.names[0] == "chair":
                entered.set()
                assert release.wait(1)
            else:
                second_predict.set()
            return [_result(names=dict(self.names))]
    backend = YoloWorldDetectorBackend(weights="model.pt", use_worker_process=False)
    backend._model = Model()
    first = threading.Thread(target=lambda: outputs.setdefault("first", backend.detect(_frame(metadata={"detection_classes": ["chair"]}))))
    second = threading.Thread(target=lambda: outputs.setdefault("second", backend.detect(_frame(metadata={"detection_classes": ["bottle"]}))))
    first.start()
    assert entered.wait(1)
    second.start()
    try:
        assert not second_predict.wait(.02)
    finally:
        release.set()
    first.join(1)
    second.join(1)
    assert not first.is_alive() and not second.is_alive()
    assert outputs["first"].detections_2d[0].label == "chair"
    assert outputs["second"].detections_2d[0].label == "bottle"




@pytest.mark.parametrize('score', [1.01, 100.0, -0.01, float('inf'), float('nan')])
def test_invalid_model_scores_never_become_full_confidence(score):
    backend = YoloWorldDetectorBackend(use_worker_process=False)
    result = _result(names={0: 'table'}, boxes=[[1, 2, 10, 20]] * 2,
                     scores=[score, .673], classes=[0, 0])
    detections = backend._normalize_yolo_result(result, _frame())
    assert [item.score for item in detections] == [.673]
    assert backend._last_output_validation['invalid_score_count'] == 1
    assert _clamp_score(score) == 0


def test_model_output_respects_configured_confidence_without_rounding():
    backend = YoloWorldDetectorBackend(use_worker_process=False, confidence_threshold=.4)
    result = _result(names={0: 'table'}, boxes=[[1, 2, 10, 20]] * 3,
                     scores=[.399, .9971, 1.0], classes=[0, 0, 0])
    assert [item.score for item in backend._normalize_yolo_result(result, _frame())] == [.9971, 1.0]


def test_fixed_engine_subset_and_duplicate_door_window_follow_production_pipeline():
    from services.perception.detectors import DetectorPipeline
    from services.perception.postprocess import ClassAgnosticNmsPostProcessor
    class Model:
        names = ['door', 'window', 'table']
        def predict(self, image, **kwargs):
            return [_result(names=self.names, boxes=[[10,10,90,90], [11,10,90,90], [1,1,8,8]],
                            scores=[.83,.72,.91], classes=[0,1,2])]
    backend = YoloWorldDetectorBackend(weights='model.engine', use_worker_process=False)
    backend._model = Model()
    pipeline = DetectorPipeline((backend,), post_processor=ClassAgnosticNmsPostProcessor())
    bundle, _ = pipeline.detect(_frame(metadata={'detection_classes':['door','window']}))
    assert [d.label for d in bundle.detections_2d] == ['door']
    assert bundle.detections_2d[0].score == .83
    assert 'window' in bundle.detections_2d[0].attributes['alternative_labels']
    assert bundle.metadata['unrequested_detection_count'] == 1
    assert bundle.metadata['postprocess']['suppressed_count'] == 1


def test_dynamic_vocabulary_stays_on_inference_device_after_first_prediction(monkeypatch):
    """复现CPU词元缓存随首次预测迁移参数后，第二次换词设备不一致。"""
    import sys
    from types import ModuleType
    class Model:
        def __init__(self, weights):
            self.device = 'cpu'
            self.text_token_device = None
            self.names = {}
        def to(self, device):
            self.device = device
            return self
        def set_classes(self, classes):
            if self.text_token_device is None:
                self.text_token_device = self.device
            if self.text_token_device != self.device:
                raise RuntimeError('文本词元与模型不在同一设备')
            self.names = dict(enumerate(classes))
        def predict(self, image, **kwargs):
            # 模拟Ultralytics首次预测自动迁移参数，但不会更新文本分词器的设备字段。
            self.device = 'cuda:0'
            return [_result(names=self.names)]
    module = ModuleType('ultralytics')
    module.YOLO = module.YOLOWorld = Model
    utils = ModuleType('ultralytics.utils.torch_utils')
    utils.select_device = lambda device, verbose=False: 'cuda:0'
    monkeypatch.setitem(sys.modules, 'ultralytics', module)
    monkeypatch.setitem(sys.modules, 'ultralytics.utils.torch_utils', utils)
    monkeypatch.setattr('services.perception.detectors._ensure_torchvision_nms_fallback', lambda: None)
    backend = YoloWorldDetectorBackend(weights='model.pt', device='0', use_worker_process=False)
    for label in ['chair', 'refrigerator', 'dining table']:
        bundle = backend.detect(_frame(metadata={'open_vocabulary_classes': [label]}))
        assert bundle.metadata['availability'] == 'available'
        assert bundle.metadata['class_prompt_applied'] is True
        assert bundle.detections_2d[0].label == label


def test_empty_configuration_uses_and_restores_model_default_classes():
    """未配置类别时不强塞object；临时找物后恢复模型原本类别。"""
    class Model:
        def __init__(self):
            self.names = {0: 'chair', 1: 'refrigerator'}
            self.calls = []
        def set_classes(self, classes):
            self.calls.append(list(classes))
            self.names = dict(enumerate(classes))
        def predict(self, image, **kwargs):
            return [_result(names=dict(self.names))]
    model = Model()
    backend = YoloWorldDetectorBackend(weights='model.pt', class_names=(), use_worker_process=False)
    backend._model = model
    first = backend.detect(_frame())
    assert model.calls == []
    assert first.metadata['class_prompt_applied'] is False
    assert first.metadata['open_vocabulary_class_source'] == 'model_default_classes'
    assert first.metadata['open_vocabulary_classes'] == ['chair', 'refrigerator']
    requested = backend.detect(_frame(metadata={'open_vocabulary_classes': ['bottle']}))
    assert requested.detections_2d[0].label == 'bottle'
    restored = backend.detect(_frame())
    assert restored.detections_2d[0].label == 'chair'
    assert restored.metadata['class_prompt_applied'] is False
    assert model.calls == [['bottle'], ['chair', 'refrigerator']]


@pytest.mark.parametrize('result_available', [False, True])
def test_fixed_engine_keeps_owned_worker_path_without_in_process_fallback(monkeypatch, result_available):
    calls = []
    value = ([[1, 2, 10, 20]], [.9], [0], ['chair'], None) if result_available else None
    owner = SimpleNamespace(predict=lambda image: calls.append('predict') or value,
        failure_reason='controlled recovery', last_inference_ms=1.0,
        status=lambda: {'state': 'ready' if result_available else 'recovering'})
    backend = YoloWorldDetectorBackend(weights='model.engine', use_worker_process=True)
    backend._inference_worker = owner
    monkeypatch.setattr(backend, '_get_inference_worker', lambda: owner)
    monkeypatch.setattr(backend, '_get_model', lambda: pytest.fail('工作进程路径不得切回调用线程加载模型'))
    bundle = backend.detect(_frame())
    assert calls == ['predict']
    assert backend._inference_worker is owner
    assert bundle.metadata['worker_fallback_used'] is False
    if result_available:
        assert bundle.metadata['inference_execution'] == 'worker_process'
        assert bundle.detections_2d[0].label == 'chair'
    else:
        assert bundle.metadata['availability'] == 'unavailable'
        assert bundle.detections_2d == ()


def test_detector_close_retains_worker_owner_until_retry_completes():
    results = iter([False, True])
    owner = SimpleNamespace(close=lambda timeout: next(results))
    backend = YoloWorldDetectorBackend(weights='model.engine')
    backend._inference_worker = owner
    assert backend.close(timeout=0) is False
    assert backend._inference_worker is owner
    assert backend.start_inference_worker_nonblocking() is False
    assert backend.close(timeout=1) is True
