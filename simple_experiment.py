import numpy as np
import scipy.io as sio
import simple_env
import simple_planner
import centralized_hungarian_nx
import matplotlib.pyplot as plt
import one_to_one_params
import dependency_test_params
from matplotlib import animation

nsteps = 1000
vel = 0.5

env = simple_env.SimpleEnv(dependency_test_params)
robot_diameter = env.robot_diameter

planner = simple_planner.SimplePlanner(env)

# compute an assignment
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

#export data to matlab
#sio.savemat(mat_inputs,)