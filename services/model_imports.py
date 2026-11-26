"""同一进程内感知与记忆共用的本地模型导入入口。"""
from threading import Lock

# torchvision 根包与 transforms 相互引用，两个入口的首次导入不能交叠。
LOCAL_MODEL_IMPORT_LOCK = Lock()
