"""把仿真平台的原始采样时刻换算成来源纳秒。"""


def source_timestamp(payload, sim_stamp):
    mapping = payload['source_clock_mapping']
    return mapping['unix_anchor_ns'] + sim_stamp - mapping['sim_anchor_ns']
