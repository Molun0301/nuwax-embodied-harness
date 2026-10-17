"""建图效果对比：导入已在地图坐标系中的XYZT点云和同源轨迹。"""
from __future__ import annotations

import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import re
import shutil
import struct
import tempfile
from urllib.parse import unquote, urlsplit

from tool_data import cloud_statistics, load_poses, write_json

ROOT = Path(__file__).resolve().parents[2]
STATIC = ROOT / "gateways/http/static"
DEFAULT_WORKSPACE = ROOT / "runtime_data/lio_compare"
MANIFEST = "版本清单.json"
NOTE = "首末水平距离和高度跨度是轨迹描述；没有外部真值，不能表示绝对精度。仅在实际返回同一位置时讨论闭合误差。"


def read_manifest(workspace: Path) -> dict:
    path = workspace / MANIFEST
    if not path.exists():
        return {"schema_version": 1, "reference": None, "selected": None, "variants": []}
    meta = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(meta.get("variants"), list):
        raise ValueError("版本清单缺少variants数组")
    return meta


def identifier(value: str) -> str:
    if not re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9_-]{0,79}", value):
        raise ValueError("版本标识需为1至80个英文字母、数字、下划线或短横线")
    return value


def local_asset(folder: Path, name: str) -> Path:
    path = (folder / name).resolve(strict=True)
    if not path.is_relative_to(folder.resolve()) or not path.is_file():
        raise ValueError("数据文件必须位于指定数据目录内")
    return path


def add_variant(workspace: Path, ident: str, title: str, cloud: Path, poses: Path,
                source_label: str, reference=False, source_base_ns=None, change="", metadata=None):
    identifier(ident)
    meta = read_manifest(workspace)
    if any(v["id"] == ident for v in meta["variants"]):
        raise ValueError(f"版本{ident}已存在，请使用新的标识保留对比记录")
    static_map = bool((metadata or {}).get("static_map"))
    rows = load_poses(poses, allow_empty=static_map)
    statistics = cloud_statistics(cloud)
    base = (rows[0]["timestamp_ns"] if rows else 0) if source_base_ns is None else source_base_ns
    if type(base) is not int or base < 0 or (rows and base > rows[0]["timestamp_ns"]):
        raise ValueError("source_base_ns必须是早于或等于首个位姿的非负整数纳秒值")
    start, end = (rows[0]["p"], rows[-1]["p"]) if rows else (None, None)
    delta = [b - a for a, b in zip(start, end)] if rows else None
    duration = max(statistics["time_range_seconds"][1], (rows[-1]["timestamp_ns"] - base) / 1e9 if rows else 0)
    variant = {**(metadata or {}), **statistics, "id": ident, "title": title,
               "point_file": ident + ".xyzt", "trajectory_file": ident + "-轨迹.json",
               "frame_count": len(rows), "source_base_ns": base, "start": start, "end": end,
               "duration_seconds": max(1, math.ceil(duration)), "closure_xy_m": math.hypot(*delta[:2]) if rows else None,
               "height_difference_m": delta[2] if rows else None,
               "height_span_m": max(p["p"][2] for p in rows) - min(p["p"][2] for p in rows) if rows else None,
               "path_length_m": sum(math.dist(a["p"], b["p"]) for a, b in zip(rows, rows[1:])) if rows else None,
               "hide_markers": static_map or not rows,
               "primary_metric_label": "首末水平距离", "primary_metric_value": f"{math.hypot(*delta[:2]):.3f} 米" if rows else "无配套轨迹",
               "dataset_label": source_label, "metric_note": NOTE,
               "change": change or "导入已生成的地图坐标点云与轨迹",
               "location": (metadata or {}).get("location", str(cloud.resolve())), "assessment": "已导入，需结合相同输入、轨迹和地图形状判断。",
               "status": "待对比", "cloud_source_note": "保留导入文件的坐标、点数与时间",
               "source": (metadata or {}).get("source", {"cloud": str(cloud.resolve()), "poses": str(poses.resolve()), "label": source_label})}
    workspace.mkdir(parents=True, exist_ok=True)
    if metadata and metadata.get("legacy_metadata"):
        history = metadata["legacy_metadata"]
        variant["change"] = history.get("change") or "历史结果导入"
        variant["assessment"] = "历史评价（本次未重新验收）：" + history.get("assessment", "未记录")
    if metadata and metadata.get("replay_observations"):
        observations = metadata["replay_observations"]
        variant["assessment"] = f"点云转换完成；原重放记录错误{observations['errors_count']}条，退出码{observations['returncode']}。请核对原重放完整性。"
    for name in (variant["point_file"], variant["trajectory_file"]):
        if (workspace / name).exists():
            raise ValueError(f"目标文件已存在：{name}")
    shutil.copyfile(cloud, workspace / variant["point_file"])
    write_json(workspace / variant["trajectory_file"], rows)
    meta["variants"].append(variant)
    if reference or meta["reference"] is None:
        meta["reference"] = ident
    meta["selected"] = ident
    write_json(workspace / MANIFEST, meta)
    return variant


def import_legacy(source: Path, workspace: Path, chosen: list[str]):
    """复制数据与说明，历史评价留作来源；使用统一的当前指标口径。"""
    if (workspace / MANIFEST).exists():
        raise ValueError("历史导入需要新的工作目录，避免覆盖已有对比")
    old = read_manifest(source)
    variants = {v["id"]: v for v in old["variants"]}
    selected = set(chosen or variants)
    pending = list(selected)
    while pending:
        ident = pending.pop()
        if ident not in variants:
            raise ValueError(f"历史版本不存在：{ident}")
        reference = variants[ident].get("reference_id") or old.get("reference")
        if reference and reference not in selected:
            selected.add(reference)
            pending.append(reference)
    if not selected or not selected.issubset(variants):
        raise ValueError("历史清单为空或缺少引用的参考版本")
    # 在临时目录完整校验后发布，错误输入不留下半份导入。
    workspace.parent.mkdir(parents=True, exist_ok=True)
    if workspace.exists() and any(workspace.iterdir()):
        raise ValueError("历史导入的目标目录必须为空")
    with tempfile.TemporaryDirectory(prefix=".lio-import-", dir=workspace.parent) as temporary:
        staged = Path(temporary)
        for ident, variant in variants.items():
            if ident not in selected:
                continue
            add_variant(staged, ident, variant.get("title", ident),
                        local_asset(source, variant["point_file"]), local_asset(source, variant["trajectory_file"]),
                        variant.get("dataset_label", "历史导入，来源信息见清单"),
                        source_base_ns=variant.get("source_base_ns"), metadata={
                            "reference_id": variant.get("reference_id"), "legacy_metadata": variant,
                            "static_map": variant.get("static_map", False), "hide_markers": variant.get("hide_markers", False)})
        meta = read_manifest(staged)
        meta["reference"] = old.get("reference") if old.get("reference") in selected else next(iter(meta["variants"]))["id"]
        meta["selected"] = old.get("selected") if old.get("selected") in selected else meta["variants"][-1]["id"]
        write_json(staged / MANIFEST, meta)
        workspace.mkdir(exist_ok=True)
        for asset in staged.iterdir():
            shutil.move(str(asset), workspace / asset.name)
    return {"imported": len(selected), "workspace": str(workspace.resolve())}


def demo(workspace: Path):
    """生成明确标注的合成闭环，供无实录环境核对工具。"""
    if (workspace / MANIFEST).exists():
        raise ValueError("演示需要新的工作目录")
    with tempfile.TemporaryDirectory(prefix="nuwax-lio-demo-") as temporary:
        folder = Path(temporary)
        for ident, drift in (("reference", 0.0), ("candidate", 0.4)):
            rows, points = [], []
            for step in range(81):
                angle = step * math.tau / 80
                rows.append({"timestamp_ns": 1_000_000_000 + step * 100_000_000,
                             "p": [3 * math.cos(angle) + drift * step / 80, 3 * math.sin(angle), 0.4],
                             "q": [0, 0, math.sin(angle / 2), math.cos(angle / 2)]})
                for k in range(50):
                    wall = k / 49 * 2.5
                    points.append((4 * math.cos(angle) + drift * step / 80, 4 * math.sin(angle), wall, step / 10))
            cloud, poses = folder / "cloud.xyzt", folder / "poses.json"
            cloud.write_bytes(b"".join(struct.pack("<4f", *p) for p in points))
            write_json(poses, rows)
            add_variant(workspace, ident, "合成参考" if not drift else "合成漂移示例", cloud, poses,
                        "合成数据 · 仅验证工具，不代表任何LIO算法成绩", reference=not drift)
    return {"workspace": str(workspace.resolve()), "variants": 2}


def report(workspace: Path):
    meta = read_manifest(workspace)
    if not meta["variants"]:
        raise ValueError("工作目录没有可对比的结果")
    keys = ("id", "title", "dataset_label", "point_count", "frame_count", "duration_seconds",
            "closure_xy_m", "height_span_m", "height_difference_m", "path_length_m", "source")
    result = {"reference": meta["reference"], "metric_note": NOTE,
              "variants": [{key: v.get(key) for key in keys} for v in meta["variants"]]}
    write_json(workspace / "对比报告.json", result)
    return result


class ComparisonHandler(SimpleHTTPRequestHandler):
    def translate_path(self, path):
        relative = unquote(urlsplit(path).path).lstrip("/")
        if not relative:
            return str(STATIC / "lio-compare/index.html")
        if relative.startswith("static/"):
            root, relative = STATIC, relative[len("static/"):]
        else:
            root = Path(self.directory)
        candidate = (root / relative).resolve()
        if not candidate.is_relative_to(root.resolve()) or not candidate.is_file():
            return str(root / ".missing-file")
        return str(candidate)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for name, help_text in (("add", "加入新点云与轨迹"), ("from-replay", "从原生重放位姿/点云生成对比"), ("import", "导入历史三维对照目录"),
                            ("demo", "生成合成演示"), ("serve", "打开本机三维对比服务"), ("report", "导出轨迹指标")):
        command = commands.add_parser(name, help=help_text)
        command.add_argument("--workspace", type=Path, default=DEFAULT_WORKSPACE, help="对比数据与结果目录")
        if name in {"add", "from-replay"}:
            command.add_argument("--id", required=True)
            command.add_argument("--title", required=True)
            command.add_argument("--source-label", required=True, help="输入来源、采集记录或数据集说明")
            command.add_argument("--reference", action="store_true")
            command.add_argument("--change", default="")
            if name == "add":
                command.add_argument("--cloud", required=True, type=Path)
                command.add_argument("--poses", required=True, type=Path)
                command.add_argument("--source-base-ns", type=int, help="XYZT时间零点；默认首个位姿时间戳")
                command.add_argument("--static-map", action="store_true", help="最终静态地图，允许轨迹为空数组并停用时间筛选")
            else:
                command.add_argument("--folder", required=True, type=Path, help="含重放结果.json、原生位姿与点云.lcm、原生轨迹.jsonl的目录")
                command.add_argument("--voxel-size", type=float, default=0.05, help="显示采样体素米数")
                command.add_argument("--time-bin-seconds", type=float, default=10, help="时间分箱秒数")
        elif name == "import":
            command.add_argument("--from", dest="source", type=Path, required=True)
            command.add_argument("--id", action="append", default=[], help="可重复；省略则导入全部，参考自动带入")
        elif name == "serve":
            command.add_argument("--host", default="127.0.0.1")
            command.add_argument("--port", type=int, default=18769)
    args = parser.parse_args(argv)
    workspace = args.workspace.resolve()
    if args.command == "add":
        result = add_variant(workspace, args.id, args.title, args.cloud, args.poses, args.source_label,
                             args.reference, args.source_base_ns, args.change, {"static_map": args.static_map})
    elif args.command == "from-replay":
        from lio_replay_export import export_replay
        result = export_replay(args, add_variant)
    elif args.command == "import":
        result = import_legacy(args.source.resolve(), workspace, args.id)
    elif args.command == "demo":
        result = demo(workspace)
    elif args.command == "report":
        result = report(workspace)
    else:
        report(workspace)
        server = ThreadingHTTPServer((args.host, args.port), partial(ComparisonHandler, directory=str(workspace)))
        print(f"LIO建图对比：http://{args.host}:{server.server_port}/\n数据目录：{workspace}", flush=True)
        try:
            server.serve_forever(poll_interval=0.2)
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()
        return 0
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
