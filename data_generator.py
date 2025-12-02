import numpy as np
import matplotlib.pyplot as plt
import csv
import os
import argparse
import roboticstoolbox as rtb
from spatialmath import SE3


def robot():
    l1 = rtb.DHLink(a=0.287, alpha=np.pi/2, d=0.155, offset=0)
    l2 = rtb.DHLink(a=0.8, alpha=0, d=0, offset=np.pi/2)
    l3 = rtb.DHLink(a=0.8, alpha=0, d=0.0, offset=0)
    l4 = rtb.DHLink(a=0.0, alpha=np.pi/2, d=0.0, offset=np.pi/2)
    l5 = rtb.DHLink(a=0.0, alpha=0, d=0.317, offset=0)

    return rtb.DHRobot(
        [l1, l2, l3, l4, l5],
        name="Frank"
    )

    
def save_to_csv(target_pose, trajectory, dataset_path):
    os.makedirs(dataset_path, exist_ok=True)

    existing_files = [f for f in os.listdir(dataset_path) if f.endswith('.csv')]
    
    if not existing_files:
        file_index = 0
    else:
        indices = [int(f.split('.')[0]) for f in existing_files]
        file_index = max(indices) + 1

    file_path = os.path.join(dataset_path, f"{file_index}.csv")

    q_initial = trajectory.q[0]
    q_final = trajectory.q[-1]

    row = np.concatenate((target_pose, q_initial, q_final))

    header = [
        'target_x', 'target_y', 'target_z', 'target_roll', 'target_pitch', 'target_yaw',
        'q0_init', 'q1_init', 'q2_init', 'q3_init', 'q4_init',
        'q0_final', 'q1_final', 'q2_final', 'q3_final', 'q4_final'
    ]

    with open(file_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(header)
        writer.writerow(row)
    
 
def execute(robot, target_pose, q_initial, q_solution, dataset_path):
    trajectory = rtb.jtraj(q_initial, q_solution, 100)
    save_to_csv(target_pose, trajectory, dataset_path)
 

dataset_path = "./data"
num_iterations = 1000
robot = robot()

for i in range(num_iterations):

    qlim = robot.qlim.T

    # Generate a random joint vector within limits
    q_random = np.array([np.random.uniform(low, high) for low, high in qlim])

    # Compute FK
    T = robot.fkine(q_random)

    pose = {
        "x": T.t[0],
        "y": T.t[1],
        "z": T.t[2],
        "roll": T.rpy()[0],
        "pitch": T.rpy()[1],
        "yaw": T.rpy()[2]
    }
        
    q_initial = np.random.uniform(-np.pi, np.pi, robot.n)
    # robot.set_joints_position(q_initial)

    # q_solution, success = robot.solve_ik(**pose)
    
    T_goal = SE3.Trans(pose['x'], pose['y'], pose['z']) * \
             SE3.RPY(pose['roll'], pose['pitch'], pose['yaw'], order='zyx')

    sol = robot.ikine_LM(T_goal, q0=q_initial)

    q_solution, success = sol.q, sol.success

    if success:
        target_pose = np.array([
            pose["x"], 
            pose["y"], 
            pose["z"], 
            pose["roll"], 
            pose["pitch"], 
            pose["yaw"]
        ])
        
        print(f"Initial joint configuration (q_initial): {np.round(q_initial, 4)}")
        print(f"Target Pose: {np.round(target_pose, 4)}")
    
        execute(robot, target_pose, q_initial, q_solution, dataset_path)
    else:
        continue