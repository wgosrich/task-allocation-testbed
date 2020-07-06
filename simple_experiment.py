import numpy as np
import scipy.io as sio
import simple_env
import simple_planner
from datetime import datetime
import os
import centralized_hungarian_nx
import matplotlib.pyplot as plt
import one_to_one_params
import dependency_test_params
from matplotlib import animation

nsteps = 1000
vel = 0.5

def get_list_from_file(filename='matlab_out'):
    d = sio.loadmat(filename)
    list_raw = d['assignment_list']
    print(list_raw)
    a_list = []
    for item in list_raw[0]:
        a_list.append(item[0])
    return a_list


env = simple_env.SimpleEnv(dependency_test_params)
env.set_seed(None)
robot_diameter = env.robot_diameter

planner = simple_planner.SimplePlanner(env)

# compute an assignment
#assignment_list = get_list_from_file() #use this line if retrieving plan from file
assignment_list = planner.plan()
env.assignment_list = assignment_list
env.build_assignment_matrix()

# run controller
t = 0
done = False
current_tasks = np.zeros((env.n_agents,))

while t < nsteps and not done:

    # calculate controls
    actions = np.zeros((env.n_agents,2))
    for robot in range(env.n_agents):
        assigned_tasks = env.assignment_list[robot]
        for task in assigned_tasks:
            if (not env.task_done[task]):
                if not env.task_readiness[task]==1: #task is not ready
                    loc = env.x[robot,:] #don't move
                    dir = np.zeros((2,))
                    actions[robot, :] = dir * vel
                    break
                else:
                    loc = env.tasks[task,:] #move towards location
                    dir = (loc - env.x[robot, :]) / np.linalg.norm(loc - env.x[robot, :])
                    actions[robot, :] = dir * vel
                    break

    # apply controls
    newstate, completion = env.step(actions)

    done = completion.all()
    t += 1

env.plot()

# export data

today = datetime.now()

dir_string = "data/" + today.strftime('%Y%m%d')
try:
    os.mkdir(dir_string)
except FileExistsError:
    pass
np.savez(dir_string+"/data_"+ today.strftime("%H%M"),env.seed_val,assignment_list,t,allow_pickle=True)
sio.savemat("matlab_inputs",{"na":env.n_agents, "nk":env.n_tasks, "dependency":env.task_dependency_matrix, "cost_vector":np.linalg.norm(env.tasks,axis=1)})
