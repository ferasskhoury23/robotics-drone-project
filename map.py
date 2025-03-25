
import pandas as pd
from shapely.geometry import LineString, Polygon
import numpy as np
from typing import List

import heapq
import random



def sort_points_clockwise( points):
    center = np.mean(points, axis=0)
    angles = [np.arctan2(p[1] - center[1], p[0] - center[0]) for p in points]
    return np.array([p for _, p in sorted(zip(angles, points), reverse=True)])


class MapLoading:
    map_path: str #='obstacles_100m_above_sea_level.csv'
    x_limit_Min : int #= -1250
    x_limit_Max : int #= -670
    y_limit_Min : int #= -1200
    y_limit_Max : int  #= -700
    start : tuple
    midway : tuple
    end : tuple
    obstacles_map: pd.DataFrame  # Stores the full obstacle dataset
    filtered_map: pd.DataFrame  # Stores the filtered obstacle data
    all_polygons: List[Polygon]  # Stores the obstacle polygons
    filtered_polygons: List[Polygon]

    def __init__(self,start,midway,end , map_path: str ='obstacles_100m_above_sea_level.csv' , x_limit_Min : int = -1250 , x_limit_Max : int = -670 , y_limit_Min : int = -1250 , y_limit_Max : int = -700):
        self.reduced_graph = None
        self.visibility_graph = None
        self.start = start
        self.midway = midway
        self.end = end

        self.map_path = map_path
        self.x_limit_Min , self.x_limit_Max = x_limit_Min, x_limit_Max
        self.y_limit_Min , self.y_limit_Max = y_limit_Min, y_limit_Max

        self.obstacles_map = pd.read_csv(self.map_path)
        self.obstacles_map.columns = self.obstacles_map.columns.str.strip()
        self.filtered_map = self.obstacles_map.query(
            "`x obs meter` >= @self.x_limit_Min and `x obs meter` <= @self.x_limit_Max and "
            "`y obs meter` >= @self.y_limit_Min and `y obs meter` <= @self.y_limit_Max"
        )
        self.all_polygons = self.load_polygons(False)
        self.filtered_polygons = self.load_polygons(True)
        self.generate_full_visibility_graph(start, midway)
        #self.reduced_graph = self.generate_reduced_visibility_graph(start, midway)

    def load_polygons(self,is_filtered = False):
        # strip column names to avoid errors due to whitespace
        obstacles_map_cpy = self.filtered_map if is_filtered else self.obstacles_map
        obstacles_map_cpy = obstacles_map_cpy[['x obs meter', 'y obs meter', 'obs type']].copy()

        polygons = []
        grouped = obstacles_map_cpy.groupby('obs type')  # Grouping by 'obs type'
        #grouped returns a collection name,data
        for _, group_data in grouped:
            #checking that the shape is valid
            if len(group_data) >= 3:
                points = group_data[['x obs meter', 'y obs meter']].values
                #sorting points and generate polygons
                polygons.append(Polygon(sort_points_clockwise(points)))
        return polygons


    def generate_full_visibility_graph(self, start, end):

        nodes=[start, end]  # Start with essential nodes
        nodes.extend([tuple(point) for polygon in self.filtered_polygons for point in polygon.exterior.coords])

        visibility_graph = {node: [] for node in nodes}

        for i, node1 in enumerate(nodes):
            for j, node2 in enumerate(nodes):
                if i < j:
                    line = LineString([node1, node2])
                    if not any(line.crosses(polygon) or line.within(polygon) for polygon in self.filtered_polygons):
                        weight = line.length # returns the Euclidean distance
                        visibility_graph[node1].append((node2, weight))
                        visibility_graph[node2].append((node1, weight))

        self.visibility_graph = visibility_graph


    def generate_reduced_visibility_graph(self, start, end):
        numbers = set(random.sample(range(1, 2998), 2000))
        count = 1
        nodes = [start, end]  # Start with essential nodes
        nodes.extend([tuple(point) for polygon in self.filtered_polygons for point in polygon.exterior.coords])

        reduced_graph = {node: [] for node in nodes}

        for i, node1 in enumerate(nodes):
            for j, node2 in enumerate(nodes):
                if i < j:
                    line = LineString([node1, node2])
                    if not any(line.crosses(polygon) or line.within(polygon) for polygon in self.filtered_polygons):
                        if count in numbers:
                            weight = line.length  # returns the Euclidean distance
                            reduced_graph[node1].append((node2, weight))
                            reduced_graph[node2].append((node1, weight))
                        count += 1
        self.reduced_graph = reduced_graph
        return reduced_graph








