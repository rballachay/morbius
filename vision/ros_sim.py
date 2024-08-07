from dataclasses import dataclass
import json
import requests
import json
import time
import os
import pandas as pd

RECORD=True

"""
This file simulates a controller which is making calls to the vision model to:
    1. See which direction the robot should go based on objects in frame
    2. Determine how far the robot has traversed

The SLAM allows us to determine where we are in the global coordinate frame, and 
our plane segmentation model allows us to determine which objects to avoid. 
"""

with open('config.json') as f:
    config = json.load(f)

# The step size needs to be global so that we create our 
# floor heat map with the correct step size 
STEP_SIZE = config['STEP_SIZE']
PORT = config['PORT']
URL = f"http://localhost:{PORT}"
TOLERANCE = config['TOLERANCE']

# some statuses 
ARRIVED = 200
NOT_ARRIVED = 300

# if we are moving away for more than 
PATIENCE = 1e10
TRAJECTORIES_DIR='.trajectories'

@dataclass
class Vector3:
    x: float
    y: float
    z: float

@dataclass
class TOLERANCES:
    x: float = TOLERANCE
    y: float = 1e10 # this is infinity, we can't move up and down
    z: float = TOLERANCE

# this is our list of destinations we want to reach. we have 
# no global path planning algorithm, so this has to be relatively 
# straight lines where the global optima is equal to the local optima
DESTINATIONS = [Vector3(0, 0, 1000), Vector3(1000, 0, 1000)]

def post(x, y, z, url=URL)->bool:
    payload = { 'x': x, 'y': y,'z': z}
    response = requests.post(url, headers={'Content-Type': 'application/json'}, data=json.dumps(payload))
    if response.status_code!=200:
        print("Failed to update location")
        return False
    return True

def get(url=URL)->dict:
    response = requests.get(url)
    if response.status_code!=200:
        print("Failed get request")
        return {}
    json = response.json()
    return json

def arrived_status(location, destination, tolerances=TOLERANCES):
    locations = [location['x'], location['y'], location['z']]
    destinations = [destination.x, destination.y, destination.z]
    tolerances = [tolerances.x, tolerances.y, tolerances.z]

    status = []
    for loc, dest, tol in zip(locations, destinations, tolerances):
        if abs(loc-dest)<tol:
            status.append(True)
        else:
            status.append(False)
    
    # only two status
    if all(status):
        return ARRIVED
    
    return NOT_ARRIVED

def do_move_robot(location, local_destination):
    # do_move_robot does NOT check if it successfully moved
    x_step_move = local_destination['x']-location['x']
    z_step_move = local_destination['z']-location['z'] 

    print(f"Moving robot {x_step_move:.2f} cm in X and {z_step_move:.2f} cm in Z")
    time.sleep(2) # give a little time  to physically move the robot
    return True

def main():

    if RECORD:
        # Ensure the .trajectories directory exists
        os.makedirs(TRAJECTORIES_DIR, exist_ok=True)

        # Create a CSV file to store the trajectory
        epoch_time = int(time.time())
        csv_file_path = f'{TRAJECTORIES_DIR}/trajectory_{epoch_time}.csv'
        df = pd.DataFrame(columns=['step', 'x', 'y', 'z'])

    # this is our destination index. once we reach 
    # this, we update the index to the next
    dest_i = 0
    counter=0
    step=0

    try:
        while True:
            step+=1

            dest = DESTINATIONS[dest_i]

            # repeat until this is updated
            status=False
            while not status:
                status = post(dest.x, dest.y, dest.z)
            
            # repeat until this is updated
            while True:
                package = get()
                if package:
                    local_destination=package['local_destination']
                    location=package['location']
                    break
            
            status=False
            while not status:
                status = do_move_robot(location, local_destination)

            # repeat until this is updated
            while True:
                package = get()
                if package:
                    location=package['location']
                    break

            robot_status = arrived_status(location, dest)

            if RECORD:
                # Save the location to the DataFrame
                df = pd.concat([df, pd.DataFrame.from_records([{'step': step, 'x': location['x'], 'y': location['y'], 'z': location['z']}])], ignore_index=True)

            # check the 
            if robot_status==ARRIVED:
                print(f"Robot ARRIVED at: x={location['x']:.2f}, y={location['y']:.2f}, z={location['z']:.2f}")
                dest_i+=1
            elif robot_status==NOT_ARRIVED:
                print(f"Robot now at: x={location['x']:.2f}, y={location['y']:.2f}, z={location['z']:.2f}")
                counter+=1
            
            if counter==PATIENCE:
                raise Exception("Robot is lost, please start again")
            
            if dest_i==len(DESTINATIONS):
                print("Arrived at final destination!!! Congrats")
                break
    except:
        if RECORD:
            df.to_csv(csv_file_path, index=False)


def plot_scatter_map():
    import matplotlib.pyplot as plt

    # take the most rejecent trajectory from the directory
    traj_paths = os.listdir(TRAJECTORIES_DIR)
    traj_path = max(filter(lambda x:x.endswith('.csv'),traj_paths))
    df = pd.read_csv(f'{TRAJECTORIES_DIR}/{traj_path}',index_col=0)

    plt.figure(figsize=(10, 6))
    plt.scatter(df['x'], df['z'], c=df['y'], cmap='viridis', s=100)
    plt.colorbar(label='Z Value')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Scatter Plot of Coordinates')
    plt.show()
    
if __name__=='__main__':
    main()