#!/usr/bin/env python3
import cv2
import numpy as np
import pupil_apriltags as apriltag

import FilePathFinder
import math

"""
This script is designed to capture video from a camera, process the images to detect AprilTags 
(fiducial markers), and use that information to infer the geometry of a chessboard.
"""


class CameraFeedClass:
    # use april tags, mode := paper | block
    def __init__(self, camID=0, mode="block"):
        self.cam = None
        self.camID = camID
        self.aprilDetector = apriltag.Detector(families="tag25h9",
                                               nthreads=4,
                                               quad_decimate=1.0,
                                               quad_sigma=0.0,
                                               refine_edges=1,
                                               decode_sharpening=1.5)
        # self.whiteChessPieceDetector = apriltag.Detector(apriltag.DetectorOptions(families="tag25h9"))
        # self.blackChessPieceDetector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
        self.PieceDetector = apriltag.Detector(families="tag25h9",
                                               nthreads=4,
                                               quad_decimate=1.0,
                                               quad_sigma=0.0,
                                               refine_edges=1,
                                               decode_sharpening=1.5)
        self.mode = mode

        self.pieceCounter = [[0, 0, 0, 0, 0, 0, 0, 0],
                             [0, 0, 0, 0, 0, 0, 0, 0],
                             [0, 0, 0, 0, 0, 0, 0, 0],
                             [0, 0, 0, 0, 0, 0, 0, 0],
                             [0, 0, 0, 0, 0, 0, 0, 0],
                             [0, 0, 0, 0, 0, 0, 0, 0],
                             [0, 0, 0, 0, 0, 0, 0, 0],
                             [0, 0, 0, 0, 0, 0, 0, 0]]
        self.referenceImage = cv2.imread("./resources/ChessboardReference.png")
        self.hasBeenCalibrated = False
        self.chessBoardVertices = []
        self.callCounter = 0
        self.chessBoardCells = []
        self.chessBoardResetCounter = 150
        self.chessBoardResetCounterThreshold = 150
        self.previosBoard = [
            list("rnbqkbnr"),
            list("pppppppp"),
            list("........"),
            list("........"),
            list("........"),
            list("........"),
            list("PPPPPPPP"),
            list("RNBQKBNR")
            #          ['.', '.', '.', '.', '.', '.', '.', '.'],           # 8
            # ['p', 'p', '.', '.', '.', 'p', 'b', 'k'],            # 7
            # ['.', '.', 'q', '.', 'p', '.', '.', 'p'],            # 6
            # ['.', '.', '.', '.', 'n', '.', 'p', '.'],            # 5
            # ['.', '.', '.', '.', 'N', 'Q', '.', '.'],            # 4
            # ['.', '.', 'B', '.', '.', '.', '.', 'P'],            # 3
            # ['.', 'P', '.', '.', '.', 'P', 'P', '.'],            # 2
            # ['.', '.', '.', '.', '.', '.', 'K', '.'],            # 1
        ]

        self.currentBoard = [
            list("rnbqkbnr"),
            list("pppppppp"),
            list("........"),
            list("........"),
            list("........"),
            list("........"),
            list("PPPPPPPP"),
            list("RNBQKBNR")
            #          ['.', '.', '.', '.', '.', '.', '.', '.'],           # 8
            # ['p', 'p', '.', '.', '.', 'p', 'b', 'k'],            # 7
            # ['.', '.', 'q', '.', 'p', '.', '.', 'p'],            # 6
            # ['.', '.', '.', '.', 'n', '.', 'p', '.'],            # 5
            # ['.', '.', '.', '.', 'N', 'Q', '.', '.'],            # 4
            # ['.', '.', 'B', '.', '.', '.', '.', 'P'],            # 3
            # ['.', 'P', '.', '.', '.', 'P', 'P', '.'],            # 2
            # ['.', '.', '.', '.', '.', '.', 'K', '.'],            # 1
        ]

    def convertToTagDetectableImage(self, image):
        grayFrame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        grayFrame = cv2.equalizeHist(grayFrame)  # Boost contrast
        clahe = cv2.createCLAHE(clipLimit=6.0, tileGridSize=(4, 4))
        grayFrame = clahe.apply(grayFrame)
        grayFrame = self.adjust_gamma(grayFrame)
        return grayFrame

    def openCamera(self):
        """
        Opens the camera using the specified camera ID.
        Checks if the camera stream is available; if not, prints an error and exits.
        """
        self.cam = cv2.VideoCapture(self.camID)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        if not self.cam.isOpened():
            print(f"ERROR: can't open camera id={self.camID}")
            # exit()

    # this might be unnecessary
    def getCenterPositionofDetection(self, detections):
        """
        Returns a dictionary mapping each tag's ID to its center.
        """
        centerMap = {}
        if isinstance(detections[0], tuple):
            for d in detections:
                centerMap[d[0][0]] = d[2]  # For each tag, extracts its center coordinates (converted to integers).
        else:
            for d in detections:
                centerMap[d.tag_id] = d.center.astype(
                    int)  # For each tag, extracts its center coordinates (converted to integers).
        return centerMap

    def drawCenterCircleForTags(self, frame, centers):
        for id in centers.keys():  # For every detected center
            # print(centers[id])
            cv2.circle(frame, tuple(centers[id]), 3, (0, 0, 255),
                       2)  # Draws a small red circle on the frame at that position.
            cv2.putText(frame, str(id), tuple(centers[id]), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255),
                        2)  # Puts the tag's ID next to the circle.

    def calculateEuclidianDist(self, point0, point1):
        """
        Computes the Euclidean distance between two 2D points.
        Uses basic distance formula to return an integer value.
        """
        return int(math.sqrt((int(point0[0]) - int(point1[0])) * (int(point0[0]) - int(point1[0]))
                             + (int(point0[1]) - int(point1[1])) * (int(point0[1]) - int(point1[1]))))

    # returns real number. Do not convert the output of this function into an int. Only convert the final coordinate
    def calculateSlope(self, point0, point1):
        """
        Computes the slope (rise over run) between two points.
        Prints x-values for debugging.
        Returns 0 if the x difference is zero to avoid division by zero.
        """
        # print(f"p0x = {point0[0]}, p1x = {point1[0]}")
        if point0[0] - point1[0] == 0:
            return 0
        return (point0[1] - point1[1]) / (point0[0] - point1[0])

    # logic for setting the boundaries of both the outer chess board and each square within the board
    # inputs might be off
    def get_chessboard_boundaries(self, detections):
        if detections is None:
            return None

        detections = self.getChessBoardCorners(detections)
        if detections is None or len(detections) < 4:
            return None

        # TL = top left apriltag, BR = bottom right apriltag, BL = bottom left apriltag ...
        if isinstance(detections[0], tuple):
            TL = detections[3][2]
            BL = detections[0][2]
            BR = detections[1][2]
            TR = detections[2][2]

            # TL = [int(TL[2][0]),int(TL[2][1])]
            # BL = [int(BL[1][0]),int(BL[1][1])]
            # BR = [int(BR[0][0]),int(BR[0][1])]
            # TR = [int(TR[3][0]), int(TR[3][1])]
        else:
            TL = detections[3].corners
            BL = detections[0].corners
            BR = detections[1].corners
            TR = detections[2].corners

            TL = [int(TL[2][0]), int(TL[2][1])]
            BL = [int(BL[1][0]), int(BL[1][1])]
            BR = [int(BR[0][0]), int(BR[0][1])]
            TR = [int(TR[3][0]), int(TR[3][1])]
        return [TL, BL, BR, TR]

    def getHomoGraphicAppliedImage(self, sourceImage, fourPoints, refImg=None, ow=1280, oh=720):
        if refImg is None:
            refImg = self.referenceImage

        refdi = cv2.cvtColor(refImg, cv2.COLOR_BGR2GRAY)
        refImgDetections = self.aprilDetector.detect(img=refdi)
        if not refImgDetections:
            return None, None

        refImgPoints = self.get_chessboard_boundaries(refImgDetections)
        if not refImgPoints:
            return None, None

        source = np.float32(fourPoints)
        ref = np.float32(refImgPoints)

        mat = cv2.getPerspectiveTransform(source, ref)
        warped = cv2.warpPerspective(sourceImage, mat, (ow, oh))
        return warped, mat

    def transform_detections(self, detections, H):
        transformed = []

        for d in detections:
            if isinstance(d, tuple):
                tag_id = d[0][0]
                center = np.array([d[2][0], d[2][1], 1.0])
            else:
                tag_id = d.tag_id
                center = np.array([d.center[0], d.center[1], 1.0])

            warped_center = H @ center
            warped_center /= warped_center[2]
            transformed.append((tag_id, warped_center[:2]))
        return transformed

    def getAxesIntervalDots(self, TL, BL, BR):
        width = self.calculateEuclidianDist(BL, BR)
        height = self.calculateEuclidianDist(TL, BL)

        x = np.linspace(0, width, 9)
        y = np.linspace(0, height, 9)

        # Uses np.meshgrid to create a grid (representing the chessboard squares).
        mx, my = np.meshgrid(x, y)

        return mx, my

    def plotBoardDots(self, frame, mx, my, TL):
        # if self.chessBoardResetCounter == self.chessBoardResetCounterThreshold:
        #        self.chessBoardVertices = []
        counter = 0
        for i in range(my.shape[0]):
            for j in range(mx.shape[1]):
                x = int(mx[i, j]) + TL[0]
                y = int(my[i, j]) + TL[1]
                if not self.hasBeenCalibrated:
                    self.chessBoardVertices.append([x, y])
                else:
                    self.chessBoardVertices[counter][0] = x
                    self.chessBoardVertices[counter][1] = y

                counter += 1

                # cv2.circle(frame,(x,y), 3,(0, 0, 255), -1)

        # if self.chessBoardResetCounter == self.chessBoardResetCounterThreshold:
        # self.chessBoardCells = []
        # print(len(self.chessBoardVertices))
        skipCounter = 0
        counter = 0
        for index in range(0, 71):
            if skipCounter == 8:
                skipCounter = 0
                continue

            if not self.hasBeenCalibrated:
                self.chessBoardCells.append(
                    {"TL": [self.chessBoardVertices[index][0], self.chessBoardVertices[index][1]],
                     "TR": [self.chessBoardVertices[index + 1][0], self.chessBoardVertices[index + 1][1]],
                     "BL": [self.chessBoardVertices[index + 9][0], self.chessBoardVertices[index + 9][1]],
                     "BR": [self.chessBoardVertices[index + 10][0], self.chessBoardVertices[index + 10][1]]})

            else:
                self.chessBoardCells[counter]["TL"] = [self.chessBoardVertices[index][0],
                                                       self.chessBoardVertices[index][1]]
                self.chessBoardCells[counter]["TR"] = [self.chessBoardVertices[index + 1][0],
                                                       self.chessBoardVertices[index + 1][1]]
                self.chessBoardCells[counter]["BL"] = [self.chessBoardVertices[index + 9][0],
                                                       self.chessBoardVertices[index + 9][1]]
                self.chessBoardCells[counter]["BR"] = [self.chessBoardVertices[index + 10][0],
                                                       self.chessBoardVertices[index + 10][1]]

            counter += 1
            skipCounter += 1

        if not self.hasBeenCalibrated:
            self.hasBeenCalibrated = True

        counter = 0
        for entry in self.chessBoardCells:
            # print(f"cell{counter}")
            # t = entry["BL"]
            # print(f"BL={t}")
            # t = entry["BR"]
            # print(f"BR={t}")
            # t = entry["TL"]
            # print(f"TL={t}")
            # t = entry["TR"]
            # print(f"TR={t}")
            # print()
            counter += 1

        # print(len(self.chessBoardCells))
        # self.chessBoardResetCounter = 0
        # print("reset counter disabled")

    def resetCounters(self):
        self.callCounter = 0
        for c in range(0, len(self.chessBoardCells)):
            self.pieceCounter[c // 8][c % 8] = 0

    # marks pieces on the currentboard
    def markPieces(self, pieces, caller):
        if len(self.chessBoardCells) > 0 and len(pieces) > 0:
            for c in range(0, len(self.chessBoardCells)):
                marked = False
                cell = self.chessBoardCells[c]
                for piece in pieces:
                    pieceCenterX, pieceCenterY = (int(piece[2][0]), int(piece[2][1]))
                    # print(f"Piece center: ({pieceCenterX}, {pieceCenterY}) | "f"Cell BL: {cell['BL']}, Cell BR: {cell['BR']}, Cell TL: {cell['TL']}")
                    if cell["BL"][0] < pieceCenterX and pieceCenterX < cell["BR"][0] and \
                        cell["BR"][1] > pieceCenterY and cell["TL"][1] < pieceCenterY:
                        if isinstance(piece, tuple):
                            if piece[0][0] in caller.pieceMap:
                                self.currentBoard[c // 8][c % 8] = caller.pieceMap[piece[0][0]]
                        else:
                            self.currentBoard[c // 8][c % 8] = caller.pieceMap[piece.tag_id]
                        self.pieceCounter[c // 8][c % 8] += 1
                        marked = True
                        break

                if not marked:
                    self.currentBoard[c // 8][c % 8] = '.'

    def markPiecesTransformed(self, transformed_tags, caller):
        if len(self.chessBoardCells) == 0:
            return

        for c, cell in enumerate(self.chessBoardCells):
            marked = False
            for tag_id, center in transformed_tags:
                x, y = center.astype(int)
                if cell["BL"][0] < x < cell["BR"][0] and cell["TL"][1] < y < cell["BR"][1]:
                    if tag_id in caller.pieceMap:
                        self.currentBoard[c // 8][c % 8] = caller.pieceMap[tag_id]
                        self.pieceCounter[c // 8][c % 8] += 1
                        marked = True
                        break
            if not marked:
                self.currentBoard[c // 8][c % 8] = '.'


    def drawLine(self, frame, p0, p1):
        """
        Draws a green line between two given points on the frame.
        Useful for visualizing board boundaries or axes.
        """
        cv2.line(frame, p0, p1, color=(0, 255, 0), thickness=1)

    # Get the april tags responsible for the board corners
    def getChessBoardCorners(self, detections):
        """
        Filters the detected AprilTags to find those with IDs 0, 1, and 2.
        These are assumed to be the corners of the chessboard.
        If fewer than 3 are found, it prints a warning and returns None.
        """
        ret = []
        # print(detections)
        for d in detections:
            if isinstance(d, tuple):
                if self.mode == "paper" and \
                    (d[0][0] == 1 or d[0][0] == 2 or d[0][0] == 3 or d[0][0] == 4) \
                    and d[0][0] not in ret:
                    ret.append(d)
                elif self.mode == "block":
                    if (d[0][0] == 5 or d[0][0] == 6 or d[0][0] == 7 or d[0][0] == 8) \
                        and d[0][0] not in ret:
                        # print(d)
                        ret.append(d)

            else:
                # print(d.tag_id)
                if self.mode == "paper" and \
                    (d.tag_id == 1 or d.tag_id == 2 or d.tag_id == 3 or d.tag_id == 4) \
                    and d.tag_id not in ret:
                    ret.append(d)
                elif self.mode == "block":
                    if (d.tag_id == 5 or d.tag_id == 6 or d.tag_id == 7 or d.tag_id == 8) \
                        and d.tag_id not in ret:
                        # print(d.tag_id)
                        ret.append(d)

        # print("")
        if len(ret) < 4:
            print("Failed to detect all four corners, detecting april tags again")
            return None
        # sort the array
        if isinstance(ret[0], tuple):
            ret = sorted(ret, key=lambda x: x[0][0])
        else:
            ret = sorted(ret, key=lambda x: x.tag_id)
        return ret

    def adjust_gamma(self, image, gamma=1.5):
        """
        Adjusts the gamma of the image to correct for lighting.
        Builds a lookup table and applies it to the image using OpenCV's cv2.LUT.
        """
        invGamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** invGamma) * 255
                          for i in range(256)]).astype("uint8")
        return cv2.LUT(image, table)

    # for debugging
    def drawAprilTagCorner(self, frame, detections, cornerPos):
        """
        For debugging: draws a circle at a specified corner (cornerPos) of each detected AprilTag.
        This helps to verify the exact locations of tag corners.
        """
        for d in detections:
            cv2.circle(frame, (int(d.corners[cornerPos][0]), int(d.corners[cornerPos][1])), 3, (255, 0, 255), 2)

    def drawPieces(self, frame, pieces, callerClass, fp):

        for pc in pieces:
            cx, cy = None, None
            if isinstance(pc, tuple):
                cx, cy = pc[2]
            else:
                cx, cy = pc.center.astype(int)

            if cx <= fp[0][0] or cx >= fp[3][0] or cy <= fp[0][1] or cy >= fp[1][1]:
                continue

            color = (0, 255, 0)
            if isinstance(pc, tuple):
                if pc[0][0] >= 20 and pc[0][0] <= 25:
                    color = (0, 0, 255)
            else:
                if pc.tag_id >= 20 and pc.tag_id <= 25:
                    color = (0, 0, 255)

            cv2.circle(frame, (cx, cy), 3, color, 2)
            cv2.putText(frame, callerClass.pieceMap[pc.tag_id],
                        (cx, cy), cv2.FONT_HERSHEY_PLAIN,
                        2, color, 2)

    def drawBordersandDots(self, frame, detections, grayFrame=None):
        if grayFrame is None:
            grayFrame = frame

        centers = self.getCenterPositionofDetection(detections)
        # self.drawCenterCircleForTags(frame,centers)
        cbc = self.getChessBoardCorners(detections)
        if cbc:
            corners = self.get_chessboard_boundaries(cbc)
            mx, my = self.getAxesIntervalDots(corners[0], corners[1], corners[2])
            self.plotBoardDots(frame, mx, my, corners[0])
            # self.drawLine(frame,corners[0],corners[1])
            # self.drawLine(frame,corners[1],corners[2])
            # print("DRAWING")

    # moved to GamePlay.py 
    # def startLoop(self):
    #     while True:
    #         ret,frame = self.cam.read()

    #         grayFrame = self.convertToTagDetectableImage(frame)
    #         detections = self.aprilDetector.detect(img=grayFrame)
    #         #self.drawBordersandDots(frame,detections,grayFrame)

    #         if detections:
    #             fp = self.get_chessboard_boundaries(detections)
    #             if fp:
    #                 warpedFrame = self.getHomoGraphicAppliedImage(grayFrame,fp)
    #                 if warpedFrame is not None:
    #                     detections = self.aprilDetector.detect(img=warpedFrame)
    #                     warpedFrame = cv2.cvtColor(warpedFrame,cv2.COLOR_GRAY2BGR)
    #                     self.drawBordersandDots(warpedFrame,detections)
    #                     cv2.imshow(f"warped",warpedFrame)

    #         cv2.imshow(f"FEED Cam-ID = {self.camID}",frame)
    #         if cv2.waitKey(1) & 0xFF == ord('q'):
    #             break

    def destroyCameraFeed(self, destroyAllWindows=True):
        """
        Clean-Up Function
        """
        self.cam.release()
        if destroyAllWindows:  # Optionally closes all OpenCV windows.
            cv2.destroyAllWindows()

    def find_april_tags(self, img, detector, n_random=60):
        # Convert to grayscale and get image dimensions
        # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = img
        h, w = gray.shape

        # Dictionary to store detections with tag id as key and list of positions as value
        detections = {}

        # Pre-calculate image center for affine transforms
        cx, cy = w / 2, h / 2
        T1 = np.array([[1, 0, -cx],
                       [0, 1, -cy],
                       [0, 0, 1]])
        T2 = np.array([[1, 0, cx],
                       [0, 1, cy],
                       [0, 0, 1]])

        for _ in range(n_random):
            # Generate small random perturbations:
            angle = np.random.uniform(-180, 180)  # degrees
            scale = np.random.uniform(0.9, 1.1)
            shear = np.random.uniform(-0.4, 0.4)  # shear factor (skew)
            tx = np.random.uniform(-80, 80)  # translation in x (pixels)
            ty = np.random.uniform(-80, 80)  # translation in y (pixels)
            brightness = np.random.uniform(-40, 40)  # brightness adjustment
            contrast = np.random.uniform(0.5, 2.0)  # contrast adjustment

            # Build the transformation matrices (in homogeneous coordinates)
            theta = np.deg2rad(angle)
            R = np.array([[np.cos(theta), -np.sin(theta), 0],
                          [np.sin(theta), np.cos(theta), 0],
                          [0, 0, 1]])
            S = np.array([[scale, 0, 0],
                          [0, scale, 0],
                          [0, 0, 1]])
            Sh = np.array([[1, shear, 0],
                           [0, 1, 0],
                           [0, 0, 1]])

            # Compose transformation: rotation, shear, then scaling.
            A = R @ Sh @ S

            # Build translation matrix (in homogeneous coordinates)
            T_translate = np.array([[1, 0, tx],
                                    [0, 1, ty],
                                    [0, 0, 1]])

            # Total transformation: center shift, then transform, then shift back
            M_total = T2 @ T_translate @ A @ T1
            M_affine = M_total[:2, :]  # 2x3 matrix for cv2.warpAffine

            # Apply the geometric transformation
            warped = cv2.warpAffine(gray, M_affine, (w, h),
                                    flags=cv2.INTER_LINEAR,
                                    borderMode=cv2.BORDER_REPLICATE)
            # Apply brightness and contrast adjustments
            warped = cv2.convertScaleAbs(warped, alpha=contrast, beta=brightness)

            # Run the apriltag detector on the perturbed image
            detections_transformed = detector.detect(warped)

            # Compute the inverse affine transformation matrix
            M_inv = np.linalg.inv(M_total)
            for det in detections_transformed:
                # Get the detection center in warped coordinates and convert to homogeneous coordinate
                center_warped = np.array([det.center[0], det.center[1], 1])
                # Inverse transform to get original image coordinates
                original_center = M_inv @ center_warped
                tag_id = det.tag_id
                # Add the position to the list for this tag id
                detections.setdefault(tag_id, []).append((original_center[0], original_center[1]))

        # Return the dictionary mapping tag id to a list of positions
        return detections

    def count_clusters(self, detections, radius=16):
        """
        Takes in detection results from find_april_tags and returns a list of clusters.

        Each cluster is a tuple: ([id1, id2, ...], count, (x, y))
        """
        # Flatten detections into a list of (tag_id, (x, y))
        all_points = [(tag_id, np.array(pos)) for tag_id, positions in detections.items() for pos in positions]
        used = np.zeros(len(all_points), dtype=bool)
        clusters = []

        for i, (tag_id, pt) in enumerate(all_points):
            if used[i]:
                continue

            # Start new cluster with this point
            cluster_ids = [tag_id]
            cluster_pts = [pt]
            used[i] = True

            # Find neighbors within radius
            for j in range(i + 1, len(all_points)):
                if used[j]:
                    continue
                _, candidate_pt = all_points[j]
                if np.linalg.norm(pt - candidate_pt) <= radius:
                    cluster_ids.append(all_points[j][0])
                    cluster_pts.append(candidate_pt)
                    used[j] = True

            cluster_ids = list(set(cluster_ids))  # unique ids
            cluster_center = np.mean(cluster_pts, axis=0)
            clusters.append((cluster_ids, len(cluster_pts), (float(cluster_center[0]), float(cluster_center[1]))))

        return [c for c in clusters if c[1] > 1]  # Only get clusters with at least 2 supporting detections

    def detectPlz(self, img, detector):
        clusters = self.count_clusters(self.find_april_tags(img, detector))
        sortedCluster = sorted(clusters, key=lambda x: x[0][0])
        return sortedCluster


if __name__ == "__main__":
    # ocr = CameraFeed(1)
    # ocr.openCamera()
    # ocr.startLoop()
    print("CameraFeed.py")
    # ocr.destroyCameraFeed()
