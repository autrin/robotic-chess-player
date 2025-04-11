import cv2
from jh1.core import Gameplay

PIECE_MAP = {
    10: 'P', 11: 'R', 12: 'N', 13: 'B', 14: 'Q', 15: 'K',  # White pieces
    20: 'p', 21: 'r', 22: 'n', 23: 'b', 24: 'q', 25: 'k'  # Black pieces
}

if __name__ == "__main__":
    engine_path = r"C:\Program Files\stockfish\stockfish-windows-x86-64-avx2.exe"

    video_source = cv2.VideoCapture(0)

    game = Gameplay(
        engine_path=engine_path,
        video_source=video_source,
        piece_map=PIECE_MAP
    )
    game.loop()
