#! /usr/bin/env python3
# import fix_python3_path
import pyivp
import yaml
#This must be modify a lot!!!
class PatternBlockGoalPointGenerator:
    def __init__(self):
        self.goal_list = []  # goal list is a list of goal points, each goal point is a list of 4 elements [x, y, z, heading]

    def generate_pattern_block(self):
        pattern_block = pyivp.string_to_seglist("format=lawnmower, x=16, y=-4, height=32, width=8, lane_width=8, rows=north-south, startx=0, starty=0, degs=0")
        goal_pair = pattern_block.size()
        goal_list = []
        for i in range(goal_pair):
            s_x = int(pattern_block.get_vx(i))
            s_y = int(pattern_block.get_vy(i))
            goal_list.append([s_x, s_y, 0])
        self.goal_list = goal_list
        print("Generate goal list")
        print(self.goal_list)
        print("=============================================================")

    def add_turn_point_when_in_corner(self):
        new_goal_list = []
        for i in range(1, len(self.goal_list) - 1):
            prev_point = self.goal_list[i - 1]
            curr_point = self.goal_list[i]
            next_point = self.goal_list[i + 1]
            
            if not (prev_point[0] == curr_point[0] == next_point[0] or prev_point[1] == curr_point[1] == next_point[1]):
                new_goal_list.append(curr_point)
        
        self.goal_list = self.goal_list[:1] + new_goal_list + self.goal_list[-1:]
        print("add_turn_point_when_in_corner")
        print(self.goal_list)
        print("=============================================================")

    def add_midpoint_in_straight_line(self):
        i = 0
        while i < len(self.goal_list) - 1:
            curr_point = self.goal_list[i]
            next_point = self.goal_list[i + 1]
            dist = ((next_point[0] - curr_point[0]) ** 2 + (next_point[1] - curr_point[1]) ** 2) ** 0.5
            if dist > 4:
                mid_point = [(curr_point[0] + next_point[0]) / 2, (curr_point[1] + next_point[1]) / 2, 0]
                self.goal_list.insert(i + 1, mid_point)
            else:
                i += 1
        print("add_midpoint_in_straight_line")
        print(self.goal_list)
        print("=============================================================")

    def add_heading_angle(self):
        for i in range(len(self.goal_list)):
            if i == 0:
                self.goal_list[i].append(0)
            else:
                prev_point = self.goal_list[i - 1]
                curr_point = self.goal_list[i]
                if i == len(self.goal_list) - 1 or (curr_point[0] == self.goal_list[i + 1][0] and curr_point[1] == self.goal_list[i + 1][1]):
                    self.goal_list[i].append(self.goal_list[i - 1][3])
                else:
                    heading = 0
                    if curr_point[0] != prev_point[0]:
                        heading = -90 if curr_point[0] > prev_point[0] else 90
                    if curr_point[1] != prev_point[1]:
                        heading = 0 if curr_point[1] > prev_point[1] else 180
                    self.goal_list[i].append(heading)
        print("add_heading_angle")
        print(self.goal_list)
        print("=============================================================")

    def format_waypoints(self, filename):
        waypoints = [point for point in self.goal_list if len(point) == 4]
        if len(waypoints) != len(self.goal_list):
            print("Invalid goal point format")
            return
        
        waypoints_dict = {"waypoints": waypoints}
        with open(filename, 'w') as file:
            yaml.dump(waypoints_dict, file, default_flow_style=None)

if __name__ == "__main__":
    generator = PatternBlockGoalPointGenerator()
    generator.generate_pattern_block()
    generator.add_turn_point_when_in_corner()
    generator.add_midpoint_in_straight_line()
    generator.add_heading_angle()
    generator.format_waypoints("waypoints.yaml")
    print(f"Waypoints saved to waypoints.yaml")
