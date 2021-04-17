from abc import ABC, abstractmethod
import random
import os
import sys
from core.Util import *
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("No environment variable SUMO_HOME!")
import traci
import sumolib
import numpy as np 
import copy

STRAIGHT = "s"
TURN_AROUND = "t"
LEFT = "l"
RIGHT = "r"
SLIGHT_LEFT = "L"
SLIGHT_RIGHT = "R"

class RouteController(ABC):
    """
    Base class for routing policy

    To implement a scheduling algorithm, implement the make_decisions() method.
    Please use the boilerplate code from the example, and implement your algorithm between
    the 'Your algo...' comments.

    make_decisions takes in a list of vehicles and network information (connection_info).
        Using this data, it should return a dictionary of {vehicle_id: decision}, where "decision"
        is one of the directions defined by SUMO (see constants above). Any scheduling algorithm
        may be injected into the simulation, as long as it is wrapped by the RouteController class
        and implements the make_decisions method.

    :param connection_info: object containing network information, including:
                            - out_going_edges_dict {edge_id: {direction: out_edge}}
                            - edge_length_dict {edge_id: edge_length}
                            - edge_index_dict {edge_index_dict} keep track of edge ids by an index
                            - edge_vehicle_count {edge_id: number of vehicles at edge}
                            - edge_list [edge_id]

    """
    def __init__(self, connection_info: ConnectionInfo):
        self.connection_info = connection_info
        self.direction_choices = [STRAIGHT, TURN_AROUND,  SLIGHT_RIGHT, RIGHT, SLIGHT_LEFT, LEFT]

    def compute_local_target(self, decision_list, vehicle):
        current_target_edge = vehicle.current_edge
        try:
            path_length = 0
            i = 0

            #the while is used to make sure the vehicle will not assume it arrives the destination beacuse the target edge is too short.
            while path_length <= max(vehicle.current_speed, 20):
                if current_target_edge == vehicle.destination:
                    break
                if i >= len(decision_list):
                    raise UserWarning(
                        "Not enough decisions provided to compute valid local target. TRACI will remove vehicle."
                    )

                choice = decision_list[i]
                if choice not in self.connection_info.outgoing_edges_dict[current_target_edge]:
                    raise UserWarning(
                            "Invalid direction. TRACI will remove vehicle."
                        )
                current_target_edge = self.connection_info.outgoing_edges_dict[current_target_edge][choice]
                path_length += self.connection_info.edge_length_dict[current_target_edge]

                if i > 0:
                    if decision_list[i - 1] == decision_list[i] and decision_list[i] == 't':
                        # stuck in a turnaround loop, let TRACI remove vehicle
                        return current_target_edge

                i += 1

        except UserWarning as warning:
            print(warning)

        return current_target_edge


    @abstractmethod
    def make_decisions(self, vehicles, connection_info, avg_deadline):
        pass


class RandomPolicy(RouteController):
    """
    Example class for a custom scheduling algorithm.
    Utilizes a random decision policy until vehicle destination is within reach,
    then targets the vehicle destination.
    """
    def __init__(self, connection_info):
        super().__init__(connection_info)
    
    def find_route(self,connection_info, current_edge, destination, deadline):
        decision_list = []
        unvisited = {edge: 1000000000 for edge in self.connection_info.edge_list} # map of unvisited edges
        visited = {} # map of visited edges
        current_distance = self.connection_info.edge_length_dict[current_edge]
        unvisited[current_edge] = current_distance
        path_lists = {edge: [] for edge in self.connection_info.edge_list} #stores shortest path to each edge using directions
        while True:
            if current_edge not in self.connection_info.outgoing_edges_dict.keys():
                continue
            for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[current_edge].items():
                if outgoing_edge not in unvisited:
                    continue
                edge_length = self.connection_info.edge_length_dict[outgoing_edge]
                new_distance = current_distance + edge_length
                if new_distance < unvisited[outgoing_edge]:
                    unvisited[outgoing_edge] = new_distance
                    current_path = copy.deepcopy(path_lists[current_edge])
                    current_path.append(direction)
                    path_lists[outgoing_edge] = copy.deepcopy(current_path)
                    #print("{} + {} : {} + {}".format(path_lists[current_edge], direction, path_edge_lists[current_edge], outgoing_edge))

            visited[current_edge] = current_distance
            del unvisited[current_edge]
            if not unvisited:
                break
            if current_edge==destination:
                break
            possible_edges = [edge for edge in unvisited.items() if edge[1]]
            current_edge, current_distance = sorted(possible_edges, key=lambda x: x[1])[0]
        

        for direction in path_lists[destination]:
            decision_list.append(direction)

        return decision_list,current_distance
    



    def make_decisions(self, vehicles, connection_info, avg_deadline):
        """
        A custom scheduling algorithm can be written in between the 'Your algo...' comments.
        -For each car in the vehicle batch, your algorithm should provide a list of future decisions.
        -Sometimes short paths result in the vehicle reaching its local TRACI destination before reaching its
         true global destination. In order to counteract this, ask for a list of decisions rather than just one.
        -This list of decisions is sent to a function that returns the 'closest viable target' edge
          reachable by the decisions - it is not the case that all decisions will always be consumed.
          As soon as there is enough distance between the current edge and the target edge, the compute_target_edge
          function will return.
        -The 'closest viable edge' is a local target that is used by TRACI to control vehicles
        -The closest viable edge should always be far enough away to ensure that the vehicle is not removed
          from the simulation by TRACI before the vehicle reaches its true destination

        :param vehicles: list of vehicles to make routing decisions for
        :param connection_info: object containing network information
        :return: local_targets: {vehicle_id, target_edge}, where target_edge is a local target to send to TRACI
        """

        local_targets = {}
        for vehicle in vehicles:
            current_edge = vehicle.current_edge

            edges = [] #Keeps track of all edges chosen
            decision_list = []
            shortest_path = [] #shortest path currently given
            while current_edge != vehicle.destination:  
                edges.append(current_edge)  
                if shortest_path == []:
                    shortest_path, distance = self.find_route(connection_info, current_edge, vehicle.destination, vehicle.deadline) #get the shortest path from edge 2
                
                if shortest_path == []:
                    break 

                direct = shortest_path[0] #take 1st choice in shortest_path given 
                out_edge1 = self.connection_info.outgoing_edges_dict[current_edge][direct] #get edge resulted from first choice in shortest_path 
                out_edge = out_edge1
                if vehicle.deadline > avg_deadline: #if vehicle's deadline is bigger than average of all vehicle deadlines 
                    for choice in self.connection_info.outgoing_edges_dict[current_edge].keys(): #check all other choices of current edge 
                        if choice != direct: #if choice is not the choice given in shortest path 
                            out_edge2 = self.connection_info.outgoing_edges_dict[current_edge][choice] #get edge resulted from this choice
                            #if the new edge is not a deadend and this deadend edge is not the destination 
                            if out_edge2 == vehicle.destination or len(self.connection_info.outgoing_edges_dict[out_edge2].keys()) != 0:
                                #if out_edge2 not in edges: to ensure that vehicle doesnt go in loops 
                                #if number on vehicles in new edge <= number of vehicles in edge resulted from shortest path
                                #and edge resulted from shortest path is not the destination
                                #then take another choice 
                                if out_edge2 not in edges and self.connection_info.edge_vehicle_count[out_edge2] <= self.connection_info.edge_vehicle_count[out_edge] and out_edge != vehicle.destination:
                                    shortest_path2, distance2 = self.find_route(connection_info, out_edge2, vehicle.destination, vehicle.deadline)
                                    #if count of vehicles on edges are =, take one with shorter length 
                                    if(self.connection_info.edge_vehicle_count[out_edge2] != self.connection_info.edge_vehicle_count[out_edge] or distance2 < distance):
                                        distance = distance2
                                        shortest_path = shortest_path2
                                        shortest_path.insert(0, choice)
                                        out_edge = out_edge2
               
                final_choice = shortest_path[0]
                shortest_path.pop(0)
                decision_list.append(final_choice)
                current_edge = out_edge

            local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)

        return local_targets
