import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

class SimpleEnv():
    """
    A simple 2D environment with robots represented as discs with 1D dynamics:
    x_dot = u

    """

    def __init__(self):
        self.dt = 0.1
        self.eps = 0.1
        self.n_agents = 3
        self.n_tasks = 3
        self.robot_diameter = 0.5

        #initialize robot state
        self.x = (np.random.rand(self.n_agents,2)-0.5)*4

        #choose random goals
        self.tasks = (np.random.rand(self.n_tasks, 2)-0.5)*4

        self.assignment_matrix = np.zeros((self.n_agents,self.n_tasks),dtype=bool)
        self.agent_assignments = []
        self.done = np.zeros((self.n_agents,), dtype=bool)
        self.state_history = []


    def step(self, action):
        """
        :param action: n_agents x 2 numpy array
        :return: new state x, completion states of all tasks
        """
        #TODO: check if action is valid

        self.x = self.x + self.dt*action

        self.check_progress()
        self.state_history.append(self.x)
        return self.x, self.done

    def check_progress(self):
        for robot in range(self.n_agents):
            assigned_tasks = self.agent_assignments[robot]
            for task in assigned_tasks:
                loc = self.tasks[task,:]
                dist = np.linalg.norm(loc-self.x[robot,:])
                if dist<self.eps:
                    self.done[task] = True

    def build_agent_assignment(self):
        # build agent assignment list
        for robot in range(self.n_agents):
            inds = np.arange(self.n_tasks)
            assigned_tasks = inds[self.assignment_matrix[robot, :]]
            self.agent_assignments.append(assigned_tasks)

    def plot(self):
        # animate
        fig = plt.figure()
        ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
        ax.set_aspect('equal')
        ax.grid()

        circles = []

        def init():
            for r in range(self.n_agents):
                circles.append(
                    plt.Circle((self.state_history[0][r, :]), 0.2, facecolor='cornflowerblue', edgecolor='k', linewidth=1))

                ax.add_patch(circles[r])

            for task in range(self.n_tasks):
                ax.add_patch(plt.Circle(self.tasks[task, :], 0.3, facecolor='lawngreen', edgecolor='k', linewidth=1))

            return circles

        def animate(i):
            for r in range(self.n_agents):
                circles[r].center = self.state_history[i][r, :]
            return circles

        ani = animation.FuncAnimation(fig, animate, range(len(self.state_history)),
                                      interval=self.dt * 1000, blit=False, init_func=init)
        plt.show()
