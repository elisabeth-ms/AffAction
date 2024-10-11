import pandas as pd
import matplotlib.pyplot as plt


def load_data(filename):
    data = pd.read_csv(filename, header=None)
    data = data.replace('head_kinect_link', 'camera')
    return data


def extract_all_objects(data):
    plot_data = []
    start_time = -1
    
    for index, row in data.iterrows():
        time = row[0]  # First column is time
        if start_time < 0:
            start_time = time
        time = time - start_time
        
        # Extract distances for all objects from the CSV
        # Assuming the object names start from column 2 and distances from column 3, alternating
        objects = row[2::2].values  # Object names
        distances = row[3::2].values  # Corresponding distances
        
        # Append data for each object at the current time
        for obj, dist in zip(objects, distances):
            plot_data.append([time, obj, dist])
    
    return pd.DataFrame(plot_data, columns=["Time", "Object", "Distance"])


def plot_object_distances(df):
    plt.figure(figsize=(10, 6))

    unique_objects = df['Object'].unique()
    
    # Plot the distance of each object over time
    for obj in unique_objects:
        obj_times = df[df['Object'] == obj]['Time']
        obj_distances = df[df['Object'] == obj]['Distance']
        
        plt.plot(obj_times, obj_distances, label=obj)

    # Customize plot
    plt.xlabel("Time")
    plt.ylabel("Distance")
    plt.title("Distance of Each Object Over Time")
    plt.legend()
    plt.grid(True)

def extract_closest_objects(data):
    plot_data = []
    start_time = -1
    for index, row in data.iterrows():
        time = row[0]  # First column is time
        if start_time<0:
            start_time=time
        time = time - start_time
        closest_distance = row[3]  
        closest_object = row[2]
        current_gaze_vel = row[1]

        
        plot_data.append([time, closest_object, closest_distance, current_gaze_vel])
    
    return pd.DataFrame(plot_data, columns=["Time", "Object", "Distance", "Vel"])


def plot_closest_objects(df):
    plt.figure(figsize=(10, 6))

    # Map objects to integers for the plot (to position them on y-axis)
    unique_objects = df['Object'].unique()
    object_mapping = {obj: i for i, obj in enumerate(unique_objects)}

    # Scatter plot: plot a point for each closest object at each time
    for obj in unique_objects:
        obj_times = df[df['Object'] == obj]['Time']
        obj_indices = [object_mapping[obj]] * len(obj_times)  # Position for the object on y-axis

        plt.scatter(obj_times, obj_indices, label=obj, s=100)  # s=100 sets the marker size

    # Customize plot
    plt.yticks(list(object_mapping.values()), list(object_mapping.keys()))  # Set y-ticks to object names
    plt.xlabel("Time")
    plt.ylabel("Objects")
    plt.title("Closest Object to Gaze Over Time")
    plt.legend()
    plt.grid(True)

def plot_gaze_velocity(df):
    plt.figure(figsize=(10, 6))  # Create a new figure for gaze velocity

    # Plot gaze velocity over time
    plt.plot(df['Time'], df['Vel'], label="Gaze Velocity", color='orange')

    # Customize plot
    plt.xlabel("Time")
    plt.ylabel("Gaze Velocity")
    plt.title("Gaze Velocity Over Time")
    plt.grid(True)
    plt.legend()

def generate_time_spent_list(df):
    time_spent_list = []
    previous_time = df.iloc[0]['Time']  # Start with the first time value
    previous_object = df.iloc[0]['Object']  # Start with the first object
    
    for index, row in df.iterrows():
        current_time = row['Time']
        current_object = row['Object']
        
        if current_object == previous_object:
            # If it's the same object, just continue
            continue
        else:
            # Calculate the time spent on the previous object
            time_spent = current_time - previous_time
            time_spent_list.append(f"{previous_object}: {time_spent:.2f}s")
            
            # Update the previous object and time
            previous_time = current_time
            previous_object = current_object

    # Capture the final object viewing time (if there's no more change)
    time_spent = df.iloc[-1]['Time'] - previous_time
    time_spent_list.append(f"{previous_object}: {time_spent:.2f}s")

    # Return the formatted list
    return time_spent_list



# File path to the CSV file
csv_file = '../../gazeData/test_from_python.csv'

# Step 4: Run the functions
data = load_data(csv_file)

# Extract distances for all objects over time
all_objects_df = extract_all_objects(data)

# Plot distances for each object over time
plot_object_distances(all_objects_df)

closest_objects_df = extract_closest_objects(data)

# Plot the closest objects
plot_closest_objects(closest_objects_df)

plot_gaze_velocity(closest_objects_df)
plt.show()
