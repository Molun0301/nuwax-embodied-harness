"""installer.ui 子包：终端 UI 抽象层。

包含 console（rich 单例 + 主题）、progress（多阶段进度条）、
banner（启动横幅 + 探测结果）、ansi（双模式降级）四个模块。
"""

__all__ = ["ansi", "banner", "console", "progress"]
