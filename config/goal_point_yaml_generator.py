import pyivp
import copy

class PatternBlockGoalPointGenerator:
    def __init__(self, width, height):
        self.goal_list = []  # goal list is a list of goal points, each goal point is a list of 4 elements [x, y, z, heading]
        self.width = width
        self.height = height

    def seglist_to_goalpairs(self, pattern_block):
        pts = []
        pts_str = pattern_block.get_spec(1)
        start_index = pts_str.find("pts={") + len("pts={")
        end_index = pts_str.find("}", start_index)
        pts_str = pts_str[start_index:end_index]
        print(pts_str)
        pts_pairs = pts_str.split(":")
        for pair in pts_pairs:
            x, y = pair.split(",")
            pts.append([float(x), float(y), 0, 0])  # Add 0 as z-coordinate and heading
        return pts

    def generate_pattern_block(self):
        startx = 0
        starty = 0
        lane_width = 8
        interestx = (self.width + startx) / 2 + lane_width / 2
        interesty = -(self.height + starty) / 2
        patter_block_str = f"format=lawnmower, x={interestx}, y={interesty}, height={self.height}, width={self.width}, lane_width={lane_width}, rows=north-south, startx={startx}, starty={starty}, degs=180"
        pattern_block = pyivp.string_to_seglist(patter_block_str)
        self.goal_list = self.seglist_to_goalpairs(pattern_block)
        # print("Generate goal list")
        # print(self.goal_list)
        # print("=============================================================")

    def add_midpoint_in_straight_line(self):
        i = 0
        while i < len(self.goal_list) - 1:
            curr_point = self.goal_list[i]
            next_point = self.goal_list[i + 1]
            dist = ((next_point[0] - curr_point[0]) ** 2 + (next_point[1] - curr_point[1]) ** 2) ** 0.5
            if dist > 4:
                if curr_point[0] == next_point[0] or curr_point[1] == next_point[1]:
                    mid_point = [(curr_point[0] + next_point[0]) / 2, (curr_point[1] + next_point[1]) / 2, 0, 0]
                    self.goal_list.insert(i + 1, mid_point)
                else:
                    raise ValueError("Points are not aligned either in x or y direction")
            else:
                i += 1
        # print("add_midpoint_in_straight_line")
        # print(self.goal_list)
        # print("=============================================================")

    def add_turn_point_when_in_corner(self):
        new_goal_list = []  
        for i in range(0, len(self.goal_list) - 1):
            if i == 0:
                prev_point = [0.0, 0.0, 0, 0]
                curr_point = self.goal_list[i]
                next_point = self.goal_list[i + 1]
            else:
                prev_point = self.goal_list[i - 1]
                curr_point = self.goal_list[i]
                next_point = self.goal_list[i + 1]
            
            if not (prev_point[0] == curr_point[0] == next_point[0] or prev_point[1] == curr_point[1] == next_point[1]):
                new_goal_list.append(copy.deepcopy(curr_point))
                new_goal_list.append(copy.deepcopy(curr_point))  # Add turn point (duplicate the current point)
            else:
                new_goal_list.append(copy.deepcopy(curr_point))
        
        new_goal_list.append(copy.deepcopy(self.goal_list[-1]))  # Add the last point
        self.goal_list = new_goal_list
        # print("add_turn_point_when_in_corner")
        # print(self.goal_list)
        # print("=============================================================")

    def add_heading_angle(self):
        self.goal_with_heading = []
        for i in range(1, len(self.goal_list) - 1):
            # print(f"Now i is:{i}, goal_list[i] is:{self.goal_list[i]}")
            curr_point = self.goal_list[i]
            if i < len(self.goal_list) - 1:
                next_point = self.goal_list[i + 1]
            else:
                next_point = curr_point
                
            if next_point[0] == curr_point[0] and next_point[1] == curr_point[1]:  # next point is the same as current point -> corner point
                self.goal_list[i][3] = self.goal_list[i - 1][3]  # copy heading from previous point
                # print("Corner point copy heading from previous point")
            elif next_point[0] == curr_point[0] or next_point[1] == curr_point[1]:  # next point is in the same line as current point
                if curr_point[0] == next_point[0]:  # x is the same -> moving in y direction
                    heading = -90 if curr_point[1] > next_point[1] else 90
                    self.goal_list[i][3] = heading
                    # print(f"heading is:{heading}")
                elif curr_point[1] == next_point[1]:  # y is the same -> moving in x direction
                    heading = 180 if curr_point[0] > next_point[0] else 0
                    self.goal_list[i][3] = heading
            #         print(f"heading is:{heading}")
            # print(f"goal_list[i] is:{self.goal_list[i]}")
            self.goal_with_heading.append(copy.deepcopy(self.goal_list[i]))        
        # Handle the last point separately
        if len(self.goal_list) > 1:
            self.goal_list[-1][3] = self.goal_list[-2][3]  # Copy heading from the previous point

        print("goal_with_heading_angle")
        print(self.goal_list)
        # print("=============================================================")

    def format_waypoints(self, filename):
        waypoints = [[int(point[0]), int(point[1]), int(point[2]), int(point[3])] for point in self.goal_with_heading if len(point) == 4]
        if len(waypoints) != len(self.goal_with_heading):
            print("Invalid goal point format")
            return

        with open(filename, 'w') as file:
            file.write("waypoint: [\n")
            for wp in waypoints:
                file.write(f"  [{wp[0]}, {wp[1]}, {wp[2]}, {wp[3]}],\n")
            file.write("]\n")

if __name__ == "__main__":
    generator = PatternBlockGoalPointGenerator(width=24, height=8)  # width can be 8*N, it will search 8*N+4 distance.
    generator.generate_pattern_block()
    generator.add_midpoint_in_straight_line()
    generator.add_turn_point_when_in_corner()
    generator.add_heading_angle()
    generator.format_waypoints("waypoints.yaml") # you can change file name here
    print(f"Waypoints saved to waypoints.yaml")
