from sokoPuzzle import SokoPuzzle
from node import Node
from collections import deque
import itertools
from copy import deepcopy
import numpy as np
import pygame
import time


class Search:

    """ Uninformed/Blind Search """
    @staticmethod
    def breadthFirst(initial_node):

        print("BreadthFirst:")
        # Check if the start element is the goal
        if initial_node.state.isGoal(Node.wall_space_obstacle):
            return initial_node, 0

        # Create the OPEN FIFO queue and the CLOSED list
        open = deque([initial_node])  # A FIFO queue
        closed = list()

        step = 0
        while True:

            print(f'*** Step {step} ***')
            step += 1

            # Check if the OPEN queue is empty => goal not found
            if len(open) == 0:
                return None, -1

            # Get the first element of the OPEN queue
            current = open.popleft()

            # Put the current node in the CLOSED list
            closed.append(current)

            # Generate the successors of the current node
            succ = current.succ()
            while len(succ) != 0:
                child = succ.popleft()

                # Check if the child is not in the OPEN queue and the CLOSED list
                if (child.state.robot_block not in [n.state.robot_block for n in closed] and
                        child.state.robot_block not in [n.state.robot_block for n in open]):

                    # Put the child in the OPEN queue
                    open.append(child)

                    # Check if the child is the goal
                    if child.state.isGoal(Node.wall_space_obstacle):
                        print(f'*** Step {step} ***')
                        print("Goal reached")
                        return child, step

    @staticmethod
    # A* Search Algorithm
    def A(initial_node, heuristic=1):

        print("A star:")

        # Check if the initial node is the goal
        if initial_node.state.isGoal(Node.wall_space_obstacle):
            return initial_node, 0

        # Evaluate the cost f for the initial node
        initial_node.F_Evaluation(heuristic)

        # Create the OPEN list with the initial node as the first element
        open = [initial_node]

        # Create the CLOSED list
        closed = list()

        step = 0
        while True:  # While the open list is not empty

            step += 1
            print(f'*** Step {step} ***')

            # Check if the OPEN list is empty => goal not found
            if len(open) == 0:
                return None, -1

            # Get the index of the node with least f in the OPEN list
            min_index, _ = min(
                enumerate(open), key=lambda element: element[1].f)
            current = open[min_index]

            # Remove the current node (i.e. the node with least f) from the OPEN list
            open.remove(current)

            # Put the current node in the CLOSED list
            closed.append(current)

            # Check if the current state is a goal
            if current.state.isGoal(Node.wall_space_obstacle):
                print("Goal reached")
                return current, step

            # Generate the successors of the current node
            succD = current.succD()
            while len(succD) != 0:
                # Pop a child node from the list of successors
                child = succD.popleft()
                # Evaluate the cost f for this child node
                child.F_Evaluation(heuristic)

                # If the child is in the OPEN list
                if child.state.robot_block in [n.state.robot_block for n in open]:
                    # Get the index of the child in the OPEN list
                    index = [n.state.robot_block for n in open].index(
                        child.state.robot_block)
                    # Replace the node in the OPEN list by the new one if its cost f is less than the old one
                    if open[index].f > child.f:
                        # Remove the old node from the OPEN list
                        open.remove(open[index])
                        # Put the new node with the minimal cost f in the OPEN list
                        open.append(child)

                # If the child is not in the OPEN list
                else:
                    # If the child is not in the CLOSED list
                    if child.state.robot_block not in [n.state.robot_block for n in closed]:
                        # Put the child in the OPEN list
                        open.append(child)

                    # If the child is in the CLOSED list
                    else:
                        # Get the index of the child in the CLOSED list
                        index = [n.state.robot_block for n in closed].index(
                            child.state.robot_block)
                        # Remove the node from the CLOSED list and add the new one in the OPEN list if its cost f is less than the old one
                        if closed[index].f > child.f:
                            # Remove the child from the CLOSED list
                            closed.remove(closed[index])
                            # Put the child in the OPEN list
                            open.append(child)


""" ***************************************************** Main function **************************************************** """

board1 = [['O', 'O', 'O', 'O', 'O', 'O'],
          ['O', 'S', ' ', 'B', ' ', 'O'],
          ['O', ' ', 'O', 'R', ' ', 'O'],
          ['O', ' ', ' ', ' ', ' ', 'O'],
          ['O', ' ', ' ', ' ', ' ', 'O'],
          ['O', 'O', 'O', 'O', 'O', 'O']]

board2 = [['O', 'O', 'O', 'O', 'O', 'O', 'O', 'O', 'O'],
          ['O', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'O'],
          ['O', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'O'],
          ['O', ' ', ' ', 'O', 'O', 'O', ' ', ' ', 'O'],
          ['O', ' ', ' ', ' ', ' ', 'O', '.', ' ', 'O'],
          ['O', ' ', ' ', ' ', ' ', ' ', 'O', ' ', 'O'],
          ['O', ' ', ' ', 'B', ' ', ' ', 'O', ' ', 'O'],
          ['O', ' ', ' ', ' ', ' ', ' ', 'O', ' ', 'O'],
          ['O', 'O', 'O', 'O', 'O', 'O', 'O', 'O', 'O']]

board3 = [['O', 'O', 'O', 'O', 'O', 'O', 'O', 'O'],
          ['O', ' ', ' ', ' ', 'O', ' ', ' ', 'O'],
          ['O', ' ', ' ', 'B', 'R', ' ', ' ', 'O'],
          ['O', ' ', ' ', ' ', 'O', 'B', ' ', 'O'],
          ['O', 'O', 'O', 'O', 'O', ' ', 'S', 'O'],
          ['O', 'O', 'O', 'O', 'O', ' ', 'S', 'O'],
          ['O', 'O', 'O', 'O', 'O', 'O', 'O', 'O']]

board4 = [['O', 'O', 'O', 'O', 'O', 'O', 'O'],
          ['O', 'O', ' ', ' ', 'O', 'O', 'O'],
          ['O', 'O', ' ', ' ', 'O', 'O', 'O'],
          ['O', 'O', ' ', '*', ' ', ' ', 'O'],
          ['O', 'O', 'B', 'O', 'B', ' ', 'O'],
          ['O', ' ', 'S', 'R', 'S', ' ', 'O'],
          ['O', ' ', ' ', ' ', ' ', 'O', 'O'],
          ['O', 'O', 'O', ' ', ' ', 'O', 'O'],
          ['O', 'O', 'O', 'O', 'O', 'O', 'O']]

board5 = [['O', 'O', 'O', 'O', 'O', 'O', 'O', 'O', 'O'],
          ['O', 'O', 'O', 'S', 'O', ' ', ' ', 'O', 'O'],
          ['O', ' ', ' ', ' ', ' ', 'B', ' ', 'O', 'O'],
          ['O', ' ', 'B', ' ', 'R', ' ', ' ', 'S', 'O'],
          ['O', 'O', 'O', ' ', 'O', ' ', 'O', 'O', 'O'],
          ['O', 'O', 'O', 'B', 'O', ' ', 'O', 'O', 'O'],
          ['O', 'O', 'O', ' ', ' ', 'S', 'O', 'O', 'O'],
          ['O', 'O', 'O', 'O', 'O', 'O', 'O', 'O', 'O']]

""" This function will create from a board (a level): a static board (wall_space_obstacle) and a dynamic board (robot_block) 
    The static board will be the same in the whole search process (we will use it just for comparison), 
    so it's better to declare it as a static variable in the class Node 
    This function will also create the initial node"""


def create_initial_node(board=None):

    height = len(board)
    width = len(board[0])

    # Separate walls, spaces and obstacles board from robot and boxes board
    robot_block = [['']*width for _ in range(height)]
    wall_space_obstacle = [['']*width for _ in range(height)]
    # creer matrice deadlock initialement vide
    deadlock_matrice = [['']*width for _ in range(height)]

    for i, j in itertools.product(range(height), range(width)):
        # Create a matrix of deadlocks
        if board[i][j] == ' ' or board[i][j] == 'R':
            if ((board[i-1][j] == 'O' and board[i][j-1] == 'O') or (board[i-1][j] == 'O' and board[i][j+1] == 'O') or (board[i][j-1] == 'O' and board[i+1][j] == 'O') or (board[i][j+1] == 'O' and board[i+1][j]) == 'O'):
                deadlock_matrice[i][j] = 'D'

        if board[i][j] == 'R':
            robot_position = (i, j)
            robot_block[i][j] = 'R'
            wall_space_obstacle[i][j] = ' '
        elif board[i][j] == 'B':
            robot_block[i][j] = 'B'
            wall_space_obstacle[i][j] = ' '
        elif board[i][j] == 'S' or board[i][j] == 'O' or board[i][j] == ' ':
            robot_block[i][j] = ' '
            wall_space_obstacle[i][j] = board[i][j]
        elif board[i][j] == '*':
            robot_block[i][j] = 'B'
            wall_space_obstacle[i][j] = 'S'
        else:  # self.board[i][j] == '.'
            robot_position = (i, j)
            robot_block[i][j] = 'R'
            wall_space_obstacle[i][j] = 'S'

    Node.wall_space_obstacle = wall_space_obstacle
    Node.deadlock_matrice = deadlock_matrice
    initial_node = Node(SokoPuzzle(robot_block, robot_position))

    return initial_node


level = board2
initial_node = create_initial_node(board=level)


goalNode, num_steps = Search.A(initial_node, heuristic=3)
if goalNode:
    print(f"Optimal Solution found after {num_steps} steps")
    solution = goalNode.getSolution()
else:
    print("Optimal solution not found")
