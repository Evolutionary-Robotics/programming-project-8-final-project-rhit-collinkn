import pybullet as p
import pybullet_data
import pyrosim.pyrosim as ps
import numpy as np
import time
import fnn
import matplotlib.pyplot as plt

def run_trial(leg_params, connection_mode=p.DIRECT, visualize=False):
    duration = 5000 
    time_step = 1 / 500 

    physicsClient = p.connect(connection_mode)
    if visualize:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    planeId = p.loadURDF("plane.urdf")

    robotId = p.loadURDF("body.urdf")
    ps.Prepare_To_Simulate(robotId)

    leg_joints = {
        'leg1': ['Foot_Torso', 'Torso_knee1', 'knee_ankle1', 'ankle_foot1', 'foot_toe1', 'toe_toenail1'],
        'leg2': ['Foot_Torso2', 'Torso_knee2', 'knee_ankle2', 'ankle_foot2', 'foot_toe2', 'toe_toenail2'],
        'leg3': ['Foot_Torso3', 'Torso_knee3', 'knee_ankle3', 'ankle_foot3', 'foot_toe3', 'toe_toenail3'],
        'leg4': ['Foot_Torso4', 'Torso_knee4', 'knee_ankle4', 'ankle_foot4', 'foot_toe4', 'toe_toenail4'],
    }

    joint_name_to_index = {}
    neural_networks = {}

    for i in range(p.getNumJoints(robotId)):
        joint_info = p.getJointInfo(robotId, i)
        joint_name = joint_info[1].decode('utf-8')
        joint_index = joint_info[0]
        joint_name_to_index[joint_name] = joint_index

    
    for leg_name, joints in leg_joints.items():
        input_size = 1  # 1 touch sensor per tentacle (on toenail joint)
        output_size = len(joints)
        units_per_layer = [input_size, 5, output_size]
        nn = fnn.FNN(units_per_layer)
        nn.setParams(leg_params[leg_name])
        neural_networks[leg_name] = nn

    joint_limits = {name: (-0.5, 0.5) for joints in leg_joints.values() for name in joints}

    scaling_factor = 1.0 

    right_legs = ['leg2', 'leg4']

    start_pos, start_orientation = p.getBasePositionAndOrientation(robotId)

    for step in range(duration):
        for leg_name, joints in leg_joints.items():
            sensor_link = 'toenail' + leg_name[-1]
            sensor_value = ps.Get_Touch_Sensor_Value_For_Link(sensor_link)
            input_value = sensor_value * 2 - 1  
            inputs = np.array([input_value])

            nn = neural_networks[leg_name]
            outputs = nn.forward(inputs)[0]

            if leg_name in right_legs:
                outputs = -outputs

            outputs *= scaling_factor

            outputs = np.clip(outputs, -1, 1)

            target_positions = []
            joint_indices = []
            for i, joint_name in enumerate(joints):
                if joint_name in joint_name_to_index:
                    output = outputs[i]
                    lower_limit, upper_limit = joint_limits[joint_name]
                    position = lower_limit + (upper_limit - lower_limit) * ((output + 1) / 2)
                    target_positions.append(position)
                    joint_indices.append(joint_name_to_index[joint_name])

            p.setJointMotorControlArray(
                bodyIndex=robotId,
                jointIndices=joint_indices,
                controlMode=p.POSITION_CONTROL,
                targetPositions=target_positions,
                positionGains=[0.1] * len(joint_indices),
                velocityGains=[0.5] * len(joint_indices),
                forces=[500] * len(joint_indices)
            )

        p.stepSimulation()
        if visualize:
            time.sleep(time_step)

    end_pos, end_orientation = p.getBasePositionAndOrientation(robotId)

    distance_traveled = np.linalg.norm(np.array(end_pos[:2]) - np.array(start_pos[:2]))

    p.disconnect()

    return distance_traveled


num_trials = 5 
units_per_layer = [1, 5, 6]  
total_params_per_leg = sum(
    units_per_layer[i] * units_per_layer[i + 1] + units_per_layer[i + 1]
    for i in range(len(units_per_layer) - 1)
)
leg_names = ['leg1', 'leg2', 'leg3', 'leg4']

results = []
distances = []  

for trial_number in range(1, num_trials + 1):
    print(f"Running trial {trial_number} without visualization...")

    leg_params = {}
    for leg_name in leg_names:
        params = np.random.uniform(-1, 1, total_params_per_leg)
        leg_params[leg_name] = params

    distance = run_trial(leg_params, connection_mode=p.DIRECT, visualize=False)
    results.append({'trial': trial_number, 'distance': distance, 'params': leg_params})
    distances.append(distance)  
    print(f"Trial {trial_number} completed. Distance traveled: {distance:.2f}")

best_result = max(results, key=lambda x: x['distance'])
best_trial = best_result['trial']
best_distance = best_result['distance']
best_params = best_result['params']

print("\nExperiment Results:")
print(f"{'Trial':<10}{'Distance Traveled':<20}")
for result in results:
    print(f"{result['trial']:<10}{result['distance']:<20.2f}")

plt.figure(figsize=(10, 6))
plt.plot(range(1, num_trials + 1), distances, marker='o', linestyle='-')
plt.title('Distance Traveled in Each Trial')
plt.xlabel('Trial Number')
plt.ylabel('Distance Traveled')
plt.grid(True)
plt.tight_layout()
plt.show()

print(f"\nRunning the best model from Trial {best_trial} with visualization...")
distance = run_trial(best_params, connection_mode=p.GUI, visualize=True)
print(f"Best model distance traveled: {distance:.2f}")
