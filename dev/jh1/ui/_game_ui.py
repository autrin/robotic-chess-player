import cv2


class GameUI:
    def __init__(self):
        pass

    def show_frame(self, title, frame):
        cv2.imshow(title, frame)

    def wait_quit(self):
        return cv2.waitKey(1) & 0xFF == ord('q')

    def destroy(self):
        cv2.destroyAllWindows()
