from typing import Optional

from jh1.typealias import Vec2
from ._apriltag_predef import ELEVATED_CORNER_TAG_IDS, PIECE_TAG_IDS


class VariCluster:
    def __init__(
        self,
        tag_id: int, detection_count: int,
        position: Vec2, stdev: float
    ) -> None:
        self.tag_id = tag_id
        self.detection_count = detection_count
        self.position = position
        self.stdev = stdev


    def get_vert_idx_if_corner(self) -> Optional[int]:
        return ELEVATED_CORNER_TAG_IDS.get(self.tag_id, None)


    def is_piece_tag(self) -> bool:
        return self.tag_id in PIECE_TAG_IDS
