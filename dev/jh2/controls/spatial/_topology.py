from typing import List, Dict
import numpy as np

from jh2.controls.spatial._waypoint import Waypoint


class Topology:
    def __init__(self, waypoints: List[Waypoint], connectivity: Dict[int, int]):
        self.waypoints: List[Waypoint] = waypoints
        self.adjacency: Dict[int, int] = connectivity


    @staticmethod
    def standard_board(
        piece_elevation: float,
        travel_elevation: float,
    ) -> 'Topology':
        """
        TODO
        generates a

        """
        # s =
