import pupil_apriltags as apriltag

# Starting from lower-left corner, then CCW, the keys denote the order of corners in the quad
# (BL: 0, BR: 1, TR: 2, TL: 3)
# FLAT_CORNER_TAG_IDS = {1: 0, 2: 1, 3: 2, 4: 3}
ELEVATED_CORNER_TAG_IDS = {5: 0, 6: 1, 7: 2, 8: 3}

# Upper-case for white, lower-case for black
PIECE_TAG_IDS = {
    10: 'P', 11: 'R', 12: 'N', 13: 'B', 14: 'Q', 15: 'K',
    20: 'p', 21: 'r', 22: 'n', 23: 'b', 24: 'q', 25: 'k'
}


def instantiate_detector() -> apriltag.Detector:
    return apriltag.Detector(
        families="tag25h9",
        nthreads=12,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.5,
    )
