import threading
from line_following import line_following, pull_forward, Position
import matplotlib.pyplot as plt

class SharedData:
    def __init__(self):
        # Create the lock.
        self.lock = threading.Lock()
        
        # Initialize the mode flags. This could also be a mode variable?
        self.command = "pause"
        self.command_changed = False
        
        # Also set up some target coordinates.
        self.x_desired = 0
        self.y_desired = 0

        self.x_pose = 0
        self.y_pose = 0
        self.heading_pose = 0

        self.robot_x = 0
        self.robot_y = 0
        self.robot_heading = 0

        self.file_name = ""
        self.clear = False
        
    def acquire(self):
        return self.lock.acquire()

    def release(self):
        self.lock.release()

def exploring(robot, explore_turn_count):
    # direction = robot.auto_explore_next(explore_turn_count)
    direction = robot.next_move(explore_turn_count, False)

    if direction == "L" or direction == "R":
        explore_turn_count += 1
    elif direction == "S":
        explore_turn_count = 0

    return (direction, explore_turn_count)

def navigating(robot, goal_changed, x_desired, y_desired, navigate_turn_count):
    direction = robot.next_move(navigate_turn_count, goal_changed, x_desired, y_desired)

    if direction == "L" or direction == "R":
        navigate_turn_count += 1
    elif direction == "S":
        navigate_turn_count = 0

    return(direction, navigate_turn_count)

def runui(shared):
    # Place everything in a try-except block to catch errors and exceptions.
    try:
        # Keep running unless told to stop.
        running = True
        while running:
            # Grab the user input - implicitely sleep while waiting.
            user_command = input("Pick from the following: explore, goal, pause, "
                                 "step, resume, left, right, straight, save, "
                                 "load, pose, clear, or quit: ") 

            if user_command == "goal":
                # If the user wants to set a goal, ask for coordinates.
                x_desired = int(input("Enter the x coordintate: "))
                y_desired = int(input("Enter the y coordinate: "))
            elif user_command == "pose":
                # If the user wants to set the pose, ask for coordinates.
                x_pose = int(input("Enter the x coordinate: "))
                y_pose = int(input("Enter the y coordinate: "))
                heading_pose = int(input("Enter the heading: "))
            elif user_command == "save":
                file_name = input("Enter the file name: ")
            elif user_command == "load":
                file_name = input("Enter the file name: ")
                x_pose = int(input("Enter the x coordinate: "))
                y_pose = int(input("Enter the y coordinate: "))
                heading_pose = int(input("Enter the heading: "))
            elif user_command == "clear":
                clear = True
            elif user_command == "quit":
                # Tell the loop to stop (do NOT jump out of the protected
                # if statement without releasing)!
                running = False

            if shared.acquire():
                shared.command = user_command
                shared.command_changed = True

                if user_command == "goal":
                    shared.x_desired = x_desired
                    shared.y_desired = y_desired
                elif user_command == "pose":
                    shared.x_pose = x_pose
                    shared.y_pose = y_pose
                    shared.heading_pose = heading_pose
                elif user_command == "save":
                    shared.file_name = file_name
                elif user_command == "load":
                    shared.file_name = file_name
                    shared.x_pose = x_pose
                    shared.y_pose = y_pose
                    shared.heading_pose = heading_pose
                elif user_command == "clear":
                    shared.clear = clear

                # Release the shared data.
                shared.release()
    except BaseException as ex:
        print("Ending Run-UI due to exception: %s" % repr(ex))

def runrobot(robot, shared):
    try:
        # Default command field to "pause"
        command = "pause"
        command_changed = False

        x_desired = 0
        y_desired = 0

        x_pose = 0
        y_pose = 0
        heading_pose = 0

        file_name = ""
        clear = False

        explore_turn_count = 0
        navigate_turn_count = 0
        direction = None
        goal_changed = False
        prev_mode = None
        

        while True:
            # Acquire the shared data
            if shared.acquire():
                command = shared.command
                command_changed = shared.command_changed
                shared.command_changed = False
                file_name = shared.file_name
                x_pose = shared.x_pose
                y_pose = shared.y_pose
                heading_pose = shared.heading_pose

                shared.robot_x = robot.x
                shared.robot_y = robot.y
                shared.robot_heading = robot.heading

                if x_desired != shared.x_desired or y_desired != shared.y_desired:
                    x_desired = shared.x_desired
                    y_desired = shared.y_desired
                    goal_changed = True
            
                shared.release()

            if command == "explore":
                if command_changed:
                    robot.robot_map.reset_blockages()
                prev_mode = "explore"
                (direction, explore_turn_count) = exploring(robot, explore_turn_count)
                if direction == None:
                    command = "pause"
            elif command == "goal":
                prev_mode = "goal"
                if command_changed:
                    robot.robot_map.reset_blockages()
                (direction, navigate_turn_count) = navigating(robot, goal_changed, x_desired, y_desired, navigate_turn_count)
                if direction == None:
                    command = "pause"
            elif command == "pause":
                robot.drive.stop()
                direction = None
            elif command == "step":
                if prev_mode == "explore":
                    (direction, explore_turn_count) = exploring(robot, explore_turn_count)
                elif prev_mode == "goal":
                    (direction, navigate_turn_count) = navigating(robot, goal_changed, x_desired, y_desired, navigate_turn_count)
                else:
                    direction = None
                # Set next command
                command = "pause"
            elif command == "resume":
                if prev_mode == "explore":
                    (direction, explore_turn_count) = exploring(robot, explore_turn_count)
                    # Set next command
                    command = "explore"
                elif prev_mode == "goal":
                    (direction, navigate_turn_count) = navigating(robot, goal_changed, x_desired, y_desired, navigate_turn_count)
                    # Set next command
                    command = "goal"
            elif command == "left":
                direction = "L"
                command = "pause"
            elif command == "right":
                direction = "R"
                command = "pause"
            elif command == "straight":
                direction = "S"
                command = "pause"
            elif command == "save":
                robot.robot_map.save(file_name)
                command = "pause"
            elif command == "load":
                robot.robot_map = robot.robot_map.load(file_name)
                robot.x = x_pose
                robot.y = y_pose
                robot.heading = heading_pose
                robot.robot_map.show(robot.x, robot.y, robot.heading)
                command = "pause"
            elif command == "pose":
                robot.x = x_pose
                robot.y = y_pose
                robot.heading = heading_pose
                robot.robot_map.show(robot.x, robot.y, robot.heading)
                command = "pause"
            elif command == "clear":
                robot.robot_map.reset_blockages()
            
            if prev_mode != "goal" and prev_mode != "explore":
                prev_mode = command
            
            # Update the shared data with the next command
            if shared.acquire():
                if not shared.command_changed:
                    shared.command = command
                shared.release()

            goal_changed = False

            if direction == "S":
                robot.execute_straight()
            elif direction == "L" or direction == "R":
                robot.execute_turn(direction)
            elif direction == None:
                robot.drive.stop()
    except BaseException as ex:
        print("Ending Run-Robot due to exception %s" % repr(ex))