import numpy as np

""" Representations:
O => Obstacle
S => Storage
B => Block
R => Robot
* => Block on a storage
. => Robot on a storage """


class SokoPuzzle:

    # The player and the blocks which are dynamic elements
    def __init__(self, robot_block, robot_position):

        # Initialize the SokoPuzzle Board
        self.robot_block = robot_block
        self.robot_position = robot_position

        # List of the robot's moves
        self.moves = ["U", "D", "L", "R"]

    def isGoal(self, wall_space_obstacle):

        # Retrieve all the storage cells
        S_indices_x, S_indices_y = np.where(
            np.array(wall_space_obstacle) == 'S')

        # Check if the storage cells contain blocks
        for ind_x, ind_y in zip(S_indices_x, S_indices_y):
            if self.robot_block[ind_x][ind_y] != 'B':
                return False
        return True

    # Check if it is a deadlock:

    def isdead_lock(self, deadlock_matrix):

        S_indices_x, S_indices_y = np.where(np.array(deadlock_matrix) == 'D')
        for ind_x, ind_y in zip(S_indices_x, S_indices_y):
            if self.robot_block[ind_x][ind_y] == 'B':
                return False

        return True

    def executeMove(self, action, wall_space_obstacle):
        if action == "U":
            return (self.up(wall_space_obstacle))
        if action == "D":
            return (self.down(wall_space_obstacle))
        if action == "L":
            return (self.left(wall_space_obstacle))
        if action == "R":
            return (self.right(wall_space_obstacle))

    def up(self, wall_space_obstacle):

        # Get the robot position
        robot_x, robot_y = self.robot_position

        # Move the robot up: U => [-1, 0]
        robot_x = robot_x-1

        # Check if the robot is moving towards a block
        if self.robot_block[robot_x][robot_y] == 'B':
            # Moving the box
            box_x, box_y = robot_x-1, robot_y
            # If the robot is not pushing another box and is moving the box towards an empty space or storage
            if self.robot_block[box_x][box_y] != 'B' and (wall_space_obstacle[box_x][box_y] == ' ' or wall_space_obstacle[box_x][box_y] == 'S'):
                self.robot_position = (robot_x, robot_y)
                self.robot_block[robot_x+1][robot_y] = ' '
                self.robot_block[robot_x][robot_y] = 'R'
                self.robot_block[box_x][box_y] = 'B'
                return True

        else:  # The robot is moving towards an empty space, a storage or a wall
            if wall_space_obstacle[robot_x][robot_y] == ' ' or wall_space_obstacle[robot_x][robot_y] == 'S':
                self.robot_position = (robot_x, robot_y)
                self.robot_block[robot_x+1][robot_y] = ' '
                self.robot_block[robot_x][robot_y] = 'R'
                return True

        return False

    def down(self, wall_space_obstacle):

        # Get the robot position
        robot_x, robot_y = self.robot_position

        # Move the robot down: D => [1, 0]
        robot_x = robot_x+1

        # Check if the robot is moving towards a block
        if self.robot_block[robot_x][robot_y] == 'B':
            # Moving the box
            box_x, box_y = robot_x+1, robot_y
            # If the robot is not pushing another box and is moving the box towards an empty space or storage
            if self.robot_block[box_x][box_y] != 'B' and (wall_space_obstacle[box_x][box_y] == ' ' or wall_space_obstacle[box_x][box_y] == 'S'):
                self.robot_position = (robot_x, robot_y)
                self.robot_block[robot_x-1][robot_y] = ' '
                self.robot_block[robot_x][robot_y] = 'R'
                self.robot_block[box_x][box_y] = 'B'
                return True

        else:  # The robot is moving towards an empty space, a storage or a wall
            if wall_space_obstacle[robot_x][robot_y] == ' ' or wall_space_obstacle[robot_x][robot_y] == 'S':
                self.robot_position = (robot_x, robot_y)
                self.robot_block[robot_x-1][robot_y] = ' '
                self.robot_block[robot_x][robot_y] = 'R'
                return True

        return False

    def left(self, wall_space_obstacle):

        # Get the robot position
        robot_x, robot_y = self.robot_position

        # Move the robot left: L => [0, -1]
        robot_y = robot_y-1

        # Check if the robot is moving towards a block
        if self.robot_block[robot_x][robot_y] == 'B':
            # Moving the box
            box_x, box_y = robot_x, robot_y-1
            # If the robot is not pushing another box and is moving the box towards an empty space or storage
            if self.robot_block[box_x][box_y] != 'B' and (wall_space_obstacle[box_x][box_y] == ' ' or wall_space_obstacle[box_x][box_y] == 'S'):
                self.robot_position = (robot_x, robot_y)
                self.robot_block[robot_x][robot_y+1] = ' '
                self.robot_block[robot_x][robot_y] = 'R'
                self.robot_block[box_x][box_y] = 'B'
                return True

        else:  # The robot is moving towards a space, a storage or a wall
            if wall_space_obstacle[robot_x][robot_y] == ' ' or wall_space_obstacle[robot_x][robot_y] == 'S':
                self.robot_position = (robot_x, robot_y)
                self.robot_block[robot_x][robot_y+1] = ' '
                self.robot_block[robot_x][robot_y] = 'R'
                return True

        return False

    def right(self, wall_space_obstacle):

        # Get the robot position
        robot_x, robot_y = self.robot_position

        # Move the robot right: R => [0, 1]
        robot_y = robot_y+1

        # Check if the robot is moving towards a block
        if self.robot_block[robot_x][robot_y] == 'B':
            # Moving the box
            box_x, box_y = robot_x, robot_y+1
            # If the robot is not pushing another box and is moving the box towards an empty space or storage
            if self.robot_block[box_x][box_y] != 'B' and (wall_space_obstacle[box_x][box_y] == ' ' or wall_space_obstacle[box_x][box_y] == 'S'):
                self.robot_position = (robot_x, robot_y)
                self.robot_block[robot_x][robot_y-1] = ' '
                self.robot_block[robot_x][robot_y] = 'R'
                self.robot_block[box_x][box_y] = 'B'
                return True

        else:  # The robot is moving towards an empty space, a storage or a wall
            if wall_space_obstacle[robot_x][robot_y] == ' ' or wall_space_obstacle[robot_x][robot_y] == 'S':
                self.robot_position = (robot_x, robot_y)
                self.robot_block[robot_x][robot_y-1] = ' '
                self.robot_block[robot_x][robot_y] = 'R'
                return True

        return False
