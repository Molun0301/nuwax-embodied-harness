"""Dependency-leaf assigned asset identity; re-exported by map_workspace."""
from pydantic import ConfigDict, field_validator

from contracts.base import ContractModel


class AssetIdentityV2(ContractModel):
    """Assigned ownership of one immutable finalized asset, never a digest.

    Replacing an asset allocates a new ID. Existing checkpoint/commit receipts
    describe its source; there is no independent asset revision clock.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")
    asset_id: str

    @field_validator("asset_id", mode="before")
    @classmethod
    def _validate_asset_id(cls, value: object) -> str:
        if (not isinstance(value, str) or len(value) != 32
                or any(character not in "0123456789abcdef" for character in value)
                or value == "0" * 32):
            raise ValueError("asset_id must be a nonzero 32-character lowercase hexadecimal assigned ID")
        return value
