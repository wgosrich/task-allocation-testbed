import numpy as np
import simple_env
import simple_planner
import matplotlib.pyplot as plt
from matplotlib import animation

nsteps = 1000
vel = 0.5

env = simple_env.SimpleEnv()
robot_diameter = env.robot_diameter

planner = simple_planner.SimplePlanner()

# compute an assignment
assignment = planner.plan(env.x,env.tasks)
env.assignment_matrix = assignment
env.build_agent_assignment()

# run controller
t = 0
done = False
state_history = []

while t < nsteps and not done:
     
    # calculate controls
    actions = np.zeros((env.n_agents,2))
    for robot in range(env.n_agents):
        assigned_tasks = env.agent_assignments[robot]
        for task in assigned_tasks:
            loc = env.tasks[task,:]
            dir = (loc-env.x[robot,:])/np.linalg.norm(loc-env.x[robot,:])
            actions[robot,:] = dir*vel
    # apply controls
    newstate, completion = env.step(actions)

    # record results
    state_history.append(newstate)
    done = completion.all()
    t += 1

env.plot()

