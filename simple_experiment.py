import numpy as np
import simple_env
import simple_planner
import matplotlib.pyplot as plt

nsteps = 1000
vel = 1

env = simple_env.SimpleEnv()
planner = simple_planner.SimplePlanner()

#compute an assignment
assignment = planner.plan(env.x,env.tasks)
env.assignment_matrix = assignment

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


#animate
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
ax.set_aspect('equal')
ax.grid()

line, = ax.plot([], [], 'o-', lw=2)
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)


def init():
    line.set_data([], [])
    time_text.set_text('')
    return line, time_text


def animate(i):
    thisx = [0, x1[i], x2[i]]
    thisy = [0, y1[i], y2[i]]

    line.set_data(thisx, thisy)
    time_text.set_text(time_template % (i*dt))
    return line, time_text


ani = animation.FuncAnimation(fig, animate, range(1, len(y)),
                              interval=dt*1000, blit=True, init_func=init)
plt.show()


