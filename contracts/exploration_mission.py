"""单层探索行程预算；距离表达地图路线与采样行程，不表达电池能耗。"""
from typing import Optional

from pydantic import BaseModel, ConfigDict, Field, model_validator


class ExplorationReturnPolicy(BaseModel):
    """显式启用的返航策略，空策略保持原探索行为。"""

    model_config = ConfigDict(extra="forbid", frozen=True, strict=True, allow_inf_nan=False)
    maximum_distance_m: Optional[float] = Field(default=None, gt=0, le=1_000_000)
    reserve_distance_m: float = Field(default=0.0, ge=0)
    return_on_completion: bool = False

    @model_validator(mode="after")
    def _validate_budget(self):
        if self.maximum_distance_m is None:
            if self.reserve_distance_m != 0:
                raise ValueError("返航保留路程需要明确总路程预算")
        elif self.maximum_distance_m <= self.reserve_distance_m:
            raise ValueError("总路程必须大于保留路程")
        return self
