import platform
import sys
import time
import getch
import matplotlib.pyplot as plt

if platform.system() == "Linux":
    sys.path.append("lib")
elif platform.system() == "Windows":
    sys.path.append("bin")

from pyAffaction import *

def plot_closest_objects(first_object_data):
    filtered_data = [(time, obj) for time, obj, dist, vel in first_object_data if obj is not None]

    unique_objects = list(set(obj for _, obj in filtered_data))

    object_mapping = {obj: i for i, obj in enumerate(unique_objects)}

    plt.figure(figsize=(10, 6))

    for obj in unique_objects:
        obj_times = [time for time, obj_name in filtered_data if obj_name == obj]
        obj_indices = [object_mapping[obj]] * len(obj_times)  

        plt.scatter(obj_times, obj_indices, label=obj, s=20)  

    # Customize plot
    plt.yticks(list(object_mapping.values()), list(object_mapping.keys()))  
    plt.xlabel("Time")
    plt.ylabel("Objects")
    plt.title("Closest Object to Gaze Over Time")
    plt.legend()
    plt.grid(True)

    # Show the plot
    plt.show()

def compute_gaze_history(first_object_data):
    gaze_history = []
    
    current_object = None
    gaze_start_time = None
    
    for i, entry in enumerate(first_object_data):
        current_time, object_name, angle_diff, gaze_vel = entry

        # Initialize the first object and gaze start time
        if current_object is None and gaze_start_time is None:
            current_object = object_name
            gaze_start_time = current_time
            continue  # Skip to the next iteration since we just initialized

        # If we're gazing at a new object
        if object_name != current_object:
            # Compute the time spent on the previous object
            gaze_duration = current_time - gaze_start_time
            gaze_history.append((current_object, int(gaze_duration * 1000)))  # Store in ms
            
            # Switch to the new object
            current_object = object_name
            gaze_start_time = current_time  # Start time for the new object

    # Handle the last object gazed at (after the loop ends)
    if current_object is not None and gaze_start_time is not None:
        gaze_duration = first_object_data[-1][0] - gaze_start_time
        gaze_history.append((current_object, int(gaze_duration * 1000)))

    return gaze_history





setLogLevel(-1)

sim = LlmSim()
sim.noTextGui = True
sim.unittest = False
sim.speedUp = 3
sim.noLimits = False
sim.verbose = False
sim.saveGazeData = False
sim.gazeDataFileName = "test_from_python.csv"
sim.xmlFileName = "g_example_gaze.xml"
sim.init(True)
sim.maxGazeAngleDiff = 60.0
sim.addTTS("native")

camera_name = "camera_0" 
success = sim.addLandmarkZmq()


event_key = 'e'  # Specify the key you want to wait for
quit_key = 'q'
start_time = 0
end_time = 0
gaze_data = None
threshold_gaze_vel = 0.0025
threshold_angle = 10.0 # deg

objects_not_wanted = ['Johnnie', 'hand_left_robot', 'hand_right_robot']
if success:
    sim.run()

    time.sleep(2)
    print("Remember to calibrate the Aruco camera. Click on the simulation window, then press Shift+W.")
    input("Press Enter to start the test...")

    while True:
        print("Press 'e' to start the gaze data collection, then press 'e' again to finish.")
        key = getch.getch()  # Wait for a key press
        if key == event_key:
            gaze_data = sim.get_gaze_data()
            start_time = gaze_data[-1].get("time")
        elif key == quit_key:
            sim.stop()
            break
        key = getch.getch()

        if key == event_key:
            gaze_data = sim.get_gaze_data()
            end_time = gaze_data[-1].get("time")
        elif key== quit_key:
            sim.stop()
            break



        first_object_data = []


        for data_point in gaze_data:
            current_time = data_point.get("time")
            current_gaze_vel = data_point.get("gaze_velocity")

            if current_time >= start_time:
        
                objects = data_point.get("objects", [])
            
                if objects:

                    # Initialize with the first object
                    selected_object_name = objects[0].get("name")
                    selected_object_angle_diff = objects[0].get("angleDiff")
                    selected_object_distance = objects[0].get("distance")

                    # Iterate over the rest of the objects
                    for i in range(1, len(objects)):
                        current_object_name = objects[i].get("name")
                        current_object_angle_diff = objects[i].get("angleDiff")
                        current_object_distance = objects[i].get("distance")

                        # Check if the angle difference is below the threshold
                        if abs(selected_object_angle_diff - current_object_angle_diff) < threshold_angle and selected_object_name in objects_not_wanted:
                            # If the current object is closer, update the selected object
                            if current_object_distance < selected_object_distance:
                                selected_object_name = current_object_name
                                selected_object_angle_diff = current_object_angle_diff
                                selected_object_distance = current_object_distance
                        


                    if current_gaze_vel <= threshold_gaze_vel:
                        first_object_data.append((current_time-start_time, selected_object_name, selected_object_angle_diff, current_gaze_vel))
                # else:
                #     first_object_data.append((current_time-start_time, None, None, current_gaze_vel))

        for entry in first_object_data:
            print(f"Time: {entry[0]}, First Object: {entry[1]}, Angle diff: {entry[2]}, Gaze Vel: {entry[3]}")

        gaze_history = compute_gaze_history(first_object_data)



        # Print the gaze history in the desired format
        gaze_history_list = [f'"{obj}": {time_spent}ms' for obj, time_spent in gaze_history]
        print("Gaze History:", gaze_history_list)

        plot_closest_objects(first_object_data)