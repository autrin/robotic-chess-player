class BoardState:
    def __init__(self):
        self.prev = [
            list("rnbqkbnr"),
            list("pppppppp"),
            list("........"),
            list("........"),
            list("........"),
            list("........"),
            list("PPPPPPPP"),
            list("RNBQKBNR")
        ]
        self.curr = [row[:] for row in self.prev]

    def update(self, detected_pieces, cells, piece_map):
        for i, cell in enumerate(cells):
            marked = False
            for piece in detected_pieces:
                x, y = piece.center.astype(int)
                if cell['BL'][0] < x < cell['BR'][0] and cell['BL'][1] < y < cell['TL'][1]:
                    self.curr[i // 8][i % 8] = piece_map[piece.tag_id]
                    marked = True
                    break
            if not marked:
                self.curr[i // 8][i % 8] = '.'

    def get_move(self):
        source = dest = None
        for i in range(8):
            for j in range(8):
                if self.prev[i][j] != self.curr[i][j]:
                    if self.prev[i][j] != '.' and self.curr[i][j] == '.':
                        source = (i, j)
                    else:
                        dest = (i, j)
        if source and dest:
            s = chr(ord('a') + source[1]) + str(8 - source[0])
            d = chr(ord('a') + dest[1]) + str(8 - dest[0])
            return s + d
        return None

    def sync(self):
        self.prev = [row[:] for row in self.curr]
