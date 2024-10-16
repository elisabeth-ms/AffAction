import os
import json
import matplotlib.pyplot as plt
import random

# Function to find the folder of a specific test
def find_test_folder(base_directory, test_number):
    test_folder_name = f'test{test_number}'
    test_folder_path = os.path.join(base_directory, test_folder_name)

    if os.path.exists(test_folder_path):
        return test_folder_path
    else:
        print(f"Test folder '{test_folder_name}' not found.")
        return None

# Function to read JSON data from a file
def read_json_file(filepath):
    with open(filepath, 'r') as f:
        return json.load(f)
    
def compute_gaze_history_closest_object(gaze_data, start_time):
    gaze_history = []
    objects_timestamps = []  # List to store start and end times for each object
    current_object = None
    gaze_start_time = None
    
    for entry in gaze_data:
        current_time = entry['time'] - start_time
        if current_time < 0:
            continue  # Skip entries before the start time
        
        # Get the first object (which has the lowest angleDiff) or handle off-target gaze
        if entry['objects']:
            object_name = entry['objects'][0]['name']
        else:
            object_name = 'off-target gaze'

        # Initialize the first object and gaze start time
        if current_object is None and gaze_start_time is None:
            current_object = object_name
            gaze_start_time = current_time
            continue  # Skip to the next iteration since we just initialized

        # If we're gazing at a new object
        if object_name != current_object:
            # Compute the time spent on the previous object
            gaze_duration = current_time - gaze_start_time
            gaze_history.append((current_object, gaze_duration))  # Store in seconds
            objects_timestamps.append((current_object, gaze_start_time, current_time))
            
            # Switch to the new object
            current_object = object_name
            gaze_start_time = current_time  # Start time for the new object

    # Handle the last object gazed at (after the loop ends)
    if current_object is not None and gaze_start_time is not None:
        gaze_duration = current_time - gaze_start_time
        gaze_history.append((current_object, gaze_duration))
        objects_timestamps.append((current_object, gaze_start_time, current_time))

    return gaze_history, objects_timestamps

def compute_multi_object_gaze_history(gaze_data, start_time, threshold_angle=60.0, max_average_angle_diff=45.0):
    gaze_history = []
    current_object = None
    current_segment_start = None
    current_angle_diffs = {}

    for entry in gaze_data:
        current_time = entry['time'] - start_time
        if current_time < 0:
            continue  # Skip entries before the start time

        # Collect angle diffs for all available objects in this gaze entry
        angle_diffs = {obj['name']: obj['angleDiff'] for obj in entry['objects']}
        all_objects_in_frame = set(angle_diffs.keys())

        # Fill missing objects with threshold_angle
        all_objects = set(obj['name'] for gaze_entry in gaze_data for obj in gaze_entry['objects'])
        for obj in all_objects:
            if obj not in angle_diffs:
                angle_diffs[obj] = threshold_angle  # Assign threshold angle for missing objects

        # Get the first object (smallest angleDiff) in this frame
        if entry['objects']:
            main_object = entry['objects'][0]['name']
            main_angle_diff = entry['objects'][0]['angleDiff']
        else:
            main_object = 'off-target gaze'
            main_angle_diff = None

        # Start a new segment if the object changes
        if main_object != current_object:
            if current_object is not None:
                # Compute segment duration and filter objects with average angleDiff > max_average_angle_diff
                segment_duration = current_time - current_segment_start
                filtered_angle_diffs = {obj: avg_angle for obj, avg_angle in current_angle_diffs.items() if avg_angle <= max_average_angle_diff and obj != current_object}
                
                if filtered_angle_diffs:
                    gaze_history.append((segment_duration, {current_object: current_angle_diffs[current_object]}, filtered_angle_diffs))
            
            # Start new segment
            current_object = main_object
            current_segment_start = current_time
            current_angle_diffs = angle_diffs
        else:
            # Update angle diffs if the object remains the same
            current_angle_diffs = {obj: (current_angle_diffs[obj] + angle_diffs[obj]) / 2 for obj in current_angle_diffs}

    # Handle the final segment
    if current_object is not None:
        segment_duration = gaze_data[-1]['time'] - current_segment_start
        filtered_angle_diffs = {obj: avg_angle for obj, avg_angle in current_angle_diffs.items() if avg_angle <= max_average_angle_diff and obj != current_object}
        
        if filtered_angle_diffs:
            gaze_history.append((segment_duration, {current_object: current_angle_diffs[current_object]}, filtered_angle_diffs))

    return gaze_history



def filter_multi_object_gaze_history(gaze_history, excluded_objects):
    filtered_gaze_history = []

    for entry in gaze_history:
        time_spent, main_object_dict, filtered_objects = entry

        # Check if main_object is not one of the robot hands
        main_object_name = list(main_object_dict.keys())[0]
        if main_object_name in excluded_objects:
            continue  # Skip this entry

        # Filter out robot hands from filtered_objects
        filtered_objects = {obj: angle for obj, angle in filtered_objects.items() if obj not in excluded_objects}
        
        # Only add the entry if valid objects remain
        if filtered_objects or main_object_name not in excluded_objects:
            filtered_gaze_history.append((time_spent, main_object_dict, filtered_objects))

    return filtered_gaze_history

def filter_gaze_history_closset_object(gaze_history, excluded_objects):
    """
    Filters the gaze history by excluding the robot's hands ('hand_left_robot' and 'hand_right_robot').

    :param gaze_history: A list of tuples representing the gaze history [(object_name, duration_in_seconds), ...]
    :return: A filtered gaze history list excluding the robot's hands.
    """
    filtered_history = [
        (obj, duration) for obj, duration in gaze_history 
        if obj not in excluded_objects
    ]
    return filtered_history



# Function to process gaze and speech data for a specific test
def process_test_data(base_directory, test_number):
    # Find the test folder
    test_folder = find_test_folder(base_directory, test_number)
    if not test_folder:
        return

    # Define directories for gaze and speech data
    gaze_folder = os.path.join(test_folder, 'gaze')
    speech_folder = os.path.join(test_folder, 'speech')

    if not os.path.exists(gaze_folder) or not os.path.exists(speech_folder):
        print("Gaze or speech folder not found in the test folder.")
        return

    # Get all JSON files in the gaze and speech folders
    gaze_files = sorted(os.listdir(gaze_folder))
    speech_files = sorted(os.listdir(speech_folder))

    # Filter JSON files
    gaze_files = [f for f in gaze_files if f.endswith('.json')]
    speech_files = [f for f in speech_files if f.endswith('.json')]

    # Process corresponding gaze and speech files
    for gaze_file, speech_file in zip(gaze_files, speech_files):
        if gaze_file != speech_file:
            print(f"Mismatch between gaze and speech file names: {gaze_file} and {speech_file}")
            continue
        
        print(f"Processing: {gaze_file}")

        # Read gaze and speech data
        gaze_data = read_json_file(os.path.join(gaze_folder, gaze_file))
        speech_data = read_json_file(os.path.join(speech_folder, speech_file))

        start_listening_time = speech_data['listening_start_time']
        end_listening_time = speech_data['listening_end_time']
        excluded_objects = ['hand_left_robot', 'hand_right_robot']


        gaze_history, objects_timestamps = compute_gaze_history_closest_object(gaze_data, start_listening_time)
        filtered_gaze_history = filter_gaze_history_closset_object(gaze_history, excluded_objects)

        print("Gaze History:", gaze_history)
        print("Filtered Gaze History:", filtered_gaze_history)

        gaze_history = compute_multi_object_gaze_history(gaze_data, start_listening_time, 60.0, 20.0)
        filtered_gaze_history = filter_multi_object_gaze_history(gaze_history, excluded_objects)
        print("Multi-Object Gaze History:", gaze_history)
        print("Filtered Multi-Object Gaze History:", filtered_gaze_history)
        
        print("speech_data: ", speech_data['transcript'])
        # Extract relevant information and plot the data
        plot_gaze_and_speech_data(gaze_data, speech_data)
        plot_angle_diff_over_time(gaze_data, start_listening_time, end_listening_time)
        plt.show()




# Function to assign a random color
def get_random_color():
    return (random.random(), random.random(), random.random())

def plot_angle_diff_over_time(gaze_data, start_time=0, end_time=None):
    # Extract all unique objects from gaze data
    unique_objects = set()
    for entry in gaze_data:
        for obj in entry['objects']:
            unique_objects.add(obj['name'])
    
    # Initialize data dictionary for each object
    object_data = {obj: {'times': [], 'angleDiffs': []} for obj in unique_objects}
    
    # Loop through the gaze data and fill in time and angleDiff values for each object
    for entry in gaze_data:
        current_time = entry['time']
        if current_time >= start_time and current_time <= end_time:
            objects_in_frame = {obj_data['name']: obj_data['angleDiff'] for obj_data in entry['objects']}
            
            for obj in unique_objects:
                if obj in objects_in_frame:
                    object_data[obj]['times'].append(current_time-start_time)
                    object_data[obj]['angleDiffs'].append(objects_in_frame[obj])
                else:
                    # If the object is not in the frame, add None to leave a gap
                    object_data[obj]['times'].append(current_time-start_time)
                    object_data[obj]['angleDiffs'].append(None)

    # Plot all objects' angleDiffs on the same graph with different colors
    plt.figure(figsize=(10, 6))
    for obj, data in object_data.items():
        plt.plot(data['times'], data['angleDiffs'], label=obj, marker='o')

    # Customize plot
    plt.xlabel('Time (seconds)')
    plt.ylabel('Angle Difference (degrees)')
    plt.title('Angle Difference Over Time for Objects')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()



# Function to plot gaze and speech data
def plot_gaze_and_speech_data(gaze_data, speech_data):
    # Extract time and object from gaze data
    gaze_times = [entry['time'] for entry in gaze_data]
    gaze_objects = [entry['objects'][0]['name'] if entry['objects'] else "None" for entry in gaze_data]

    # Extract speech word data
    speech_words = [(word['word'], word['start_time'], word['end_time']) for word in speech_data['words']]
    listening_start_time = speech_data['listening_start_time']
    listening_end_time = speech_data['listening_end_time']
      # Create unique colors for gaze objects
    unique_gaze_objects = list(set(gaze_objects))
    gaze_colors = {obj: get_random_color() for obj in unique_gaze_objects}
    # Plot the gaze data
    plt.figure(figsize=(10, 6))

    # Plot gaze data (top plot)
    plt.subplot(2, 1, 1)
    for i, obj in enumerate(gaze_objects):
        if gaze_times[i] >= listening_start_time and gaze_times[i] <= listening_end_time:
            plt.plot(gaze_times[i]-listening_start_time, unique_gaze_objects.index(obj), 'o', color=gaze_colors[obj], label=obj if i == 0 else "")
    plt.title("Gaze Data Over Time")
    plt.xlabel("Time")
    plt.ylabel("Objects")
    plt.xlim(0, listening_end_time - listening_start_time)
    plt.yticks(range(len(unique_gaze_objects)), unique_gaze_objects)
    plt.grid(True)

  # Create unique colors for speech words
    unique_words = [word for word, _, _ in speech_words]
    word_colors = {word: get_random_color() for word in unique_words}

    # Plot speech data (bottom plot)
    plt.subplot(2, 1, 2)
    for word, start_time, end_time in speech_words:
        color = word_colors[word]
        plt.hlines(y=0, xmin=start_time, xmax=end_time, color=color, linewidth=6, label=word)
        plt.text((start_time + end_time) / 2, 0, word, ha='center', va='bottom', fontsize=12, color='black')
   

    plt.title("Speech Data (Word Timestamps)")
    plt.xlabel("Time (seconds)")
    plt.xlim(0, listening_end_time - listening_start_time)
    plt.grid(True)
    plt.yticks([])  # Remove yticks for the speech timeline
    plt.tight_layout()

# Main function to run the analysis
def main():
    base_directory = "../../inputData"  # Example: "../../inputData"
    test_number = input("Enter the test number to analyze: ")

    try:
        test_number = int(test_number)
        process_test_data(base_directory, test_number)
    except ValueError:
        print("Invalid test number. Please enter a valid integer.")

if __name__ == "__main__":
    main()
