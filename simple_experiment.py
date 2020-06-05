import numpy as np
import simple_env
import simple_planner
import matplotlib.pyplot as plt
import one_to_one_params
import dependency_test_params
from matplotlib import animation

nsteps = 1000
vel = 0.5

env = simple_env.SimpleEnv(dependency_test_params)
robot_diameter = env.robot_diameter

planner = simple_planner.SimplePlanner()

# compute an assignment
assignment_list = planner.plan(env.x,env.tasks)
env.assignment_list = assignment_list
env.build_assignment_matrix()

# run controller
t = 0
done = False

while t < nsteps and not done:

    # calculate controls
    actions = np.zeros((env.n_agents,2))
    for robot in range(env.n_agents):
        assigned_tasks = env.assignment_list[robot]
        for task in assigned_tasks:
            if (not env.task_done[task]) and env.task_readiness[task]==1:
                loc = env.tasks[task,:]
                dir = (loc-env.x[robot,:])/np.linalg.norm(loc-env.x[robot,:])
                actions[robot,:] = dir*vel
    # apply controls
    newstate, completion = env.step(actions)

    done = completion.all()
    t += 1

env.plot()

