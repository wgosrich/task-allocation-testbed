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

#compute an assignment
assignment = planner.plan(env.x,env.tasks)
env.assignment_matrix = assignment
env.build_agent_assignment()

#run controller
t = 0
done = False
state_history = []

while t<nsteps and not done:
     
    #calculate controls
    actions = np.zeros((env.n_agents,2))
    for robot in range(env.n_agents):
        assigned_tasks = env.agent_assignments[robot]
        for task in assigned_tasks:
            loc = env.tasks[task,:]
            dir = (loc-env.x[robot,:])/np.linalg.norm(loc-env.x[robot,:])
            actions[robot,:] = dir*vel
    #apply controls
    newstate,completion = env.step(actions)

    #record results
    state_history.append(newstate)
    done = completion.all()
    t += 1
tfinal = t

#animate
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
ax.set_aspect('equal')
ax.grid()

circles = []

def init():
    for r in range(env.n_agents):
        circles.append(plt.Circle((state_history[0][r,:]),0.2,facecolor='cornflowerblue',edgecolor='k',linewidth=1))

        ax.add_patch(circles[r])

    for task in range(env.n_tasks):
        ax.add_patch(plt.Circle(env.tasks[task,:],0.3,facecolor='lawngreen',edgecolor='k',linewidth=1))

    return circles

def animate(i):
    for r in range(env.n_agents):
        circles[r].center = state_history[i][r,:]
    return circles


ani = animation.FuncAnimation(fig, animate, range(tfinal),
                              interval=env.dt*1000, blit=False, init_func=init)
plt.show()


