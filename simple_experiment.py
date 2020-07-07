import numpy as np

def set_seed(seed_value):
    if seed_value == None:
        seed_val = np.random.randint(0, 1000)
    else:
        seed_val = seed_value
    np.random.seed(seed_val)
    return seed_val

seed_val = set_seed(0)

import scipy.io as sio
import simple_env
import simple_planner
import simple_controller
from datetime import datetime
import os
import centralized_hungarian_nx
import matplotlib.pyplot as plt
import one_to_one_params
import dependency_test_params
from matplotlib import animation

nsteps = 1000

def get_list_from_file(filename='matlab_out'):
    d = sio.loadmat(filename)
    list_raw = d['assignment_list']
    print(list_raw)
    a_list = []
    for item in list_raw[0]:
        a_list.append(item[0])
    return a_list


env = simple_env.SimpleEnv(dependency_test_params)
robot_diameter = env.robot_diameter

planner = simple_planner.SimplePlanner(env)
controller = simple_controller.SimpleController(env)
# compute an assignment
assignment_list = get_list_from_file() #use this line if retrieving plan from file
print(assignment_list)
#assignment_list = planner.plan()
env.assignment_list = assignment_list
env.build_assignment_matrix()

# run controller
t = 0
done = False
current_tasks = np.zeros((env.n_agents,))

while t < nsteps and not done:

    # calculate controls
    actions = controller.get_actions()

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
np.savez(dir_string+"/data_"+ today.strftime("%H%M"),seed=seed_val,assignment_list=assignment_list,t=t/env.dt,allow_pickle=True)

#generate travel time for matlab
tt = np.zeros((env.n_tasks**2,1))
for ii in range(env.n_tasks):
    for jj in range(env.n_tasks):
        tt[ii*env.n_tasks+jj] = np.linalg.norm(env.tasks[ii,:]-env.tasks[jj,:])
print(tt)

sio.savemat("matlab_inputs",{"na":env.n_agents, "nk":env.n_tasks, "dependency":env.task_dependency_matrix, "cost_vector":np.zeros((env.n_tasks,)), "travel_time":tt})
