import numpy as np
from math import pi
from math import floor, ceil
import random

class Environment:
    def __init__(self, dim1, dim2, maxSolarAngle):
        self.dim1 = dim1 #perpendicular to solar light, runs parallel to solar path
        self.dim2 = dim2 #mostly parallel with solar light, runs perpendicular to solar path
        self.maxSolarAngle = maxSolarAngle #angle from vertical to sun

        val1 = np.tan((self.maxSolarAngle/180)*pi)
        val2 = np.tan(((self.maxSolarAngle + 1)/180)*pi)
        acceptable_percent_change = (val2/val1) - 1 #really hope this isn't zero
        scale_factor = 1/acceptable_percent_change
        self.y_displacement_solar = round(scale_factor * self.dim2) + 1
        self.z_displacement_solar = round(self.y_displacement_solar/val1) + 1

        #print(self.y_displacement_solar)
        #print(self.z_displacement_solar)

    def reset(self, tree_height=5):
        #creates a self.state and returns it
        #The state is a self.dim1 x self.dim2 board where each entry is a vector C
        #C = [height, heat_effect, 1(whether you can place a tree there)] for now
        #heat effect is a negative number, more negative for a worser material like asphalt but less negative or even positive for something like a tree

        #we now make our board
        self.counter = 0
        self.tree_height = tree_height
        initial_C = [0, -1, 1] #we will overwrite this for certain positions to give features
        y_vector = []
        for _ in range(self.dim2):
            y_vector.append(initial_C[:])
        board = []
        for _ in range(self.dim1):
            board.append(y_vector[:])

        road = self.make_road()
        for position in road:
            board[position[0]][position[1]] = [0, -10, 0]

        for _ in range(10):
            building = self.make_building()
            for position in building:
                if board[position[0]][position[1]][2] == 1:
                    board[position[0]][position[1]] = [(2*tree_height), -5, 0]

        self.state = board
        return self.state

    def make_road(self):
        road_width = random.randrange(floor(self.dim2 / 8), floor(self.dim2 / 4))
        y_bottom_0 = random.randrange(floor((3/4)*self.dim2))
        y_bottom_1 = random.randrange(floor((3/4)*self.dim2))
        road_positions = []
        for i in range(self.dim1):
            y_bottom = y_bottom_0 + (i / self.dim1) * (y_bottom_1 - y_bottom_0)
            y_bottom = int(y_bottom)
            for j in range(y_bottom, y_bottom + road_width):
                if (j >= 0) and (j < self.dim2):
                    road_positions.append((i, j))
        return road_positions

    def make_building(self):
        dimension = floor(min(self.dim1, self.dim2) / 10)
        x_start = random.randrange(floor(8.5 * min(self.dim1, self.dim2) / 10))
        y_start = random.randrange(floor(8.5 * min(self.dim1, self.dim2) / 10))
        building_positions = []
        for i in range(x_start, x_start + dimension):
            for j in range(y_start, y_start + dimension):
                building_positions.append((i, j))
        return building_positions

    def step(self, action):
        i = action // self.dim2
        j = action % self.dim2
        initial_heat_score = self.calculate_heat_buildup()
        if self.state[i][j][2] == 1:
            #we can place a tree here
            self.state[i][j] = [self.tree_height, 0, 0]
        final_heat_score = self.calculate_heat_buildup()
        reward = final_heat_score - initial_heat_score
        self.counter += 1
        done = (self.counter == self.dim1)
        return (self.state, reward, done)


    def calculate_heat_buildup(self):
        heat_score = 0
        angle_y_step = 1 / (5*self.dim2) #the change in degrees is 1
        max_delta_x_theta = 2 * np.arctan(self.dim1 / (2*self.y_displacement_solar)) * (180/pi)
        angle_x_step = (max_delta_x_theta / self.dim1) / 5

        numAdds = 0 #debugging variable
        numGoodAdds1 = 0
        numGoodAdds2 = 0
        numGoodAdds3 = 0
        num15Adds = 0

        for i in range(self.dim1):
            if i < floor(self.dim1 / 2):
                solar_ascent_rate = 0.9 *  self.z_displacement_solar / floor(self.dim1 / 2)
                solar_start_pos = 0.1 *  self.z_displacement_solar
                solar_current_pos = i*solar_ascent_rate + solar_start_pos
            else:
                solar_descent_rate = -0.9 *  self.z_displacement_solar / (self.dim1 - floor(self.dim1 / 2))
                solar_start_pos = self.z_displacement_solar
                solar_current_pos = i*solar_descent_rate + solar_start_pos

            if i == 0:
                x_angle_start = 90
            else:
                x_angle_start = np.arctan(self.y_displacement_solar/i) *(180/pi)
                x_angle_start -= 1

            if i == self.dim1:
                x_angle_end = 90
            else:
                x_angle_end = (pi - np.arctan(self.y_displacement_solar/(self.dim1 - i))) * (180/pi)
                x_angle_end += 1

            y_angle_start = np.arctan(self.y_displacement_solar / solar_current_pos) * (180/pi)
            y_angle_start -= 1
            y_angle_end = np.arctan((self.y_displacement_solar + self.dim2) / solar_current_pos) * (180/pi)
            y_angle_end += 1

            current_x_angle = x_angle_start #+ angle_x_step
            current_y_angle = y_angle_start #+ angle_y_step

            #print("x: " + str(x_angle_start) + " " + str(x_angle_end))
            #print("y: " + str(y_angle_start) + " " + str(y_angle_end))

            while current_y_angle < y_angle_end:
                #print("first while loop")
                while current_x_angle < x_angle_end:
                    #print("yee")
                    if current_x_angle != 90:
                        slope_x = np.tan(current_x_angle * (pi/180))
                        #print(slope_x)
                        j = i
                        intersected = False
                        while j < self.dim1 and not intersected:
                            y_displacement = slope_x * (i - j)
                            #print(y_displacement)
                            #if (y_displacement - self.y_displacement_solar >= 0):
                                #print(y_displacement - self.y_displacement_solar)
                            if (y_displacement - self.y_displacement_solar >= 0) and (y_displacement - self.y_displacement_solar < self.dim2):
                                #calculate the height of the ray
                                slope_y = np.tan(current_y_angle * (pi/180))
                                amount_descended = y_displacement / slope_y
                                ray_height = solar_current_pos - amount_descended #THIS IS YOUR ERROR, replace self.z_displacement_solar with solar_current_pos
                                y_coord = floor(y_displacement - self.y_displacement_solar)
                                threshold = self.state[j][y_coord][0]
                                #print(threshold)
                                if ray_height <= threshold:
                                    intersected = True
                                    #print("something happened")
                                    heat_score += self.state[i][y_coord][1]
                                    numAdds += 1
                                    if self.state[i][y_coord][1] == 00:
                                        numGoodAdds1 += 1
                                    #print(self.state[i][y_coord][1])
                            j += 1
                        j = i
                        while j >= 0 and not intersected:
                            y_displacement = slope_x * (i - j)
                            #print(y_displacement)
                            #if (y_displacement - self.y_displacement_solar >= 0):
                                #print(y_displacement - self.y_displacement_solar)
                            if (y_displacement - self.y_displacement_solar >= 0) and (y_displacement - self.y_displacement_solar < self.dim2):
                                #calculate the height of the ray
                                slope_y = np.tan(current_y_angle * (pi/180))
                                amount_descended = y_displacement / slope_y
                                ray_height = solar_current_pos - amount_descended #THIS IS YOUR ERROR, replace self.z_displacement_solar with solar_current_pos
                                y_coord = floor(y_displacement - self.y_displacement_solar)
                                threshold = self.state[j][y_coord][0]
                                if ray_height <= threshold:
                                    intersected = True
                                    #print("something happened")
                                    heat_score += self.state[i][y_coord][1]
                                    numAdds += 1
                                    if self.state[i][y_coord][1] == 0:
                                        numGoodAdds2 += 1
                            j -= 1
                    current_x_angle += angle_x_step
                j = 0
                intersected = False
                #print("while loop starting")
                while j < self.dim2 and intersected == False:
                    y_displacement = self.y_displacement_solar + j
                    #calculate the height of the ray
                    slope_y = np.tan(current_y_angle * (pi/180))
                    amount_descended = y_displacement / slope_y
                    ray_height = solar_current_pos - amount_descended #THIS IS YOUR ERROR, replace self.z_displacement_solar with solar_current_pos
                    #print(ray_height)
                    threshold = self.state[i][j][0]
                    if ray_height <= threshold:
                        intersected = True
                        #print("something happened")
                        heat_score += self.state[i][j][1]
                        numAdds += 1
                        if self.state[i][j][1] == 0:
                            numGoodAdds3 += 1
                            if i == 15:
                                num15Adds += 1
                    j += 1
                current_y_angle += angle_y_step

        #print(numAdds)
        #print(numGoodAdds1)
        #print(numGoodAdds2)
        #print(numGoodAdds3)
        #print(num15Adds)
        return heat_score

    def function_with_old_code(self):
        #right now we are going to make a road
        road_vector = [0, -10, 0]
        y_bottom = round(self.dim2/2) #we will have this depend on x later
        y_top = y_bottom + 4

        board[23][y_bottom - 1] = [tree_height, 0, 0]
        for i in range(self.dim1):
            for j in range(y_bottom, y_top):
                board[i][j] = road_vector[:]
            #board[i][y_bottom] = [5, 0, 0] #add a tree vector

        #print(board)
        #print()
        #print()
        #board[15][y_bottom] = [tree_height, 0, 0] #add a tree vector
        self.state = board
        #print(self.state[15][y_bottom - 1])
















if __name__ == "__main__":
    env = Environment(30, 30, 30)
    env.reset()
    #print(env.state)
    for i in range(30):
        rowstring = ""
        for j in range(30):
            rowstring += str(env.state[i][j][1]) + " "
        print(rowstring)

    print(env.calculate_heat_buildup() * -1)
