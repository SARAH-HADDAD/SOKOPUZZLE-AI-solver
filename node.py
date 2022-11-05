from collections import deque
from copy import deepcopy
import numpy as np
from math import inf
import itertools


class Node:

    # The wall_space_obstacle is a static variable of the class Node
    wall_space_obstacle = []

    deadlock_matrix = []

    def __init__(self, sokoPuzzle, parent=None, move=""):
        self.state = sokoPuzzle  # Un état
        self.parent = parent  # Un lien vers le nœud père
        if self.parent == None:
            self.moves = move
            # The function g represents the cost from the initial node to the current node. For the initial node, g=0
            self.g = 0
        else:
            self.moves = self.parent.moves + move
            """ The function g represents the cost from the initial node to the current node. 
            For a node which is different from the initial node, 
            g = self.parent.g (which is the cost from the initial node to the parent node) 
                + 1 (which is the cost from the parent node to the current node = the cost of one move) """
            self.g = self.parent.g + 1

    # Generate the successors
    def succ(self):
        succs = deque()
        for m in self.state.moves:
            succState = deepcopy(self.state)
            if succState.isdead_lock(Node.deadlock_matrix):
                if succState.executeMove(m, Node.wall_space_obstacle):
                    succs.append(Node(succState, self, m))
        return succs

    # Return the search solution
    def getSolution(self):
        node = self
        solution = []
        while node:
            height = len(node.state.robot_block)
            width = len(node.state.robot_block[0])
            state = deepcopy(Node.wall_space_obstacle)
            for i, j in itertools.product(range(height), range(width)):
                if node.state.robot_block[i][j] == 'R':
                    if state[i][j] == ' ':
                        state[i][j] = 'R'
                    else:
                        state[i][j] = '.'
                elif node.state.robot_block[i][j] == 'B':
                    if state[i][j] == ' ':
                        state[i][j] = 'B'
                    else:
                        state[i][j] = '*'
            solution.append(state)
            node = node.parent
        solution = solution[::-1]
        # print(solution)
        return solution

    # The estimation of the evaluation function f
    def F_Evaluation(self, heuristic=1):
        # The function h represents an estimation of the cost from the current node to the goal node using a heuristic.
        heuristics = {1: self.heuristic1(),
                      2: self.heuristic2(),
                      3: self.heuristic3()}
        self.h = heuristics[heuristic]

        # The function f is the total cost, i.e., the sum of g and h.
        self.f = self.g + self.h

    """ First heuristic: Number of left storages """

    def heuristic1(self):

        # Retrieve all the storages
        wall_space_obstacle = np.array(Node.wall_space_obstacle)
        S_indices_x, S_indices_y = np.where(wall_space_obstacle == 'S')

        # Count the number of the left storages
        left_storage = len(S_indices_x)
        for ind_x, ind_y in zip(S_indices_x, S_indices_y):
            if self.state.robot_block[ind_x][ind_y] == 'B':
                left_storage -= 1

        return left_storage

    """ Second heuristic: 2 * Number of left storages + Min Manhattan Distance between the blocks and the storages """

    def heuristic2(self):

        # Retrieve all the storages
        wall_space_obstacle = np.array(Node.wall_space_obstacle)
        S_indices_x, S_indices_y = np.where(wall_space_obstacle == 'S')

        # Retrieve all the blocks
        robot_block = np.array(self.state.robot_block)
        B_indices_x, B_indices_y = np.where(robot_block == 'B')

        # Get the distance between each block and the nearest storage, and meanwhile, count the number of left storages
        sum_distance = 0
        storage_left = len(S_indices_x)
        for b_ind_x, b_ind_y in zip(B_indices_x, B_indices_y):
            min_distance = +inf
            for s_ind_x, s_ind_y in zip(S_indices_x, S_indices_y):
                # Manhattan distance between a storage and a block
                distance = abs(b_ind_x-s_ind_x) + abs(b_ind_y-s_ind_y)
                # if the distance between a block and a storage is 0 => the block is in the storage => we decrease the number of left storages
                if distance == 0:
                    storage_left -= 1
                # Check if the current storage is the nearest to the current block
                if distance < min_distance:
                    min_distance = distance
            # Get the sum of the distances between each block and the nearest storage
            sum_distance += min_distance

        return sum_distance + 2*storage_left

    """ Third heuristic: 2 * Number of left storages + Min Manhattan Distance between the blocks and the storages 
                        + Min Manhattan Distance between the robot and the blocks """

    def heuristic3(self):

        # Retrieve all the storages
        wall_space_obstacle = np.array(Node.wall_space_obstacle)
        S_indices_x, S_indices_y = np.where(wall_space_obstacle == 'S')

        # Retrieve all the blocks
        robot_block = np.array(self.state.robot_block)
        B_indices_x, B_indices_y = np.where(robot_block == 'B')

        # Get the distance between each block and the nearest storage and
        # the distance between the robot and the nearest block, and meanwhile, count the number of the left storages
        sum_distance = 0
        storage_left = len(S_indices_x)
        min_distance_br = +inf
        for b_ind_x, b_ind_y in zip(B_indices_x, B_indices_y):

            # Get the distance between the robot and the nearest block
            # Manhattan distance between the robot and a block
            distance_br = abs(
                b_ind_x-self.state.robot_position[0]) + abs(b_ind_y-self.state.robot_position[1])
            # Check if the current block is the nearest to the robot
            if distance_br < min_distance_br:
                min_distance_br = distance_br

            # Get the distance between each block and the nearest storage
            min_distance = +inf
            for s_ind_x, s_ind_y in zip(S_indices_x, S_indices_y):
                # Manhattan distance between a storage and a block
                distance = abs(b_ind_x-s_ind_x) + abs(b_ind_y-s_ind_y)
                # if the distance between a block and a storage is 0 => the block is in the storage => we decrease the number of left storages
                if distance == 0:
                    storage_left -= 1
                # Check if the current storage is the nearest to the current block
                if distance < min_distance:
                    min_distance = distance
            # Get the sum of the distances between each block and the nearest storage
            sum_distance += min_distance

        return sum_distance + min_distance_br + 2*storage_left
