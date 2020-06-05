import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

class SimpleEnv():
    """
    A simple 2D environment with robots represented as discs with 1D dynamics:
    x_dot = u

    """

    def __init__(self, params):
        self.dt = params.dt
        self.eps = params.eps
        self.n_agents = params.n_agents
        self.n_tasks = params.n_tasks
        self.robot_diameter = 0.5

        #initialize robot state
        self.x = (np.random.rand(self.n_agents,2)-0.5)*4

        # ---------task variables-------------
        # import task locations
        self.tasks = params.tasks

        # import task dependency matrix:
        # a "1" in row i, column j represents the dependence of task i on task j
        self.task_dependency_matrix = params.task_dependency_matrix

        # task_readiness vector represents the percentage of dependencies that are fulfilled for task i
        # a value of 1 means the task is ready
        self.task_readiness = np.zeros((self.n_tasks,))
        self.task_readiness_history = []
        self.assignment_matrix = np.zeros((self.n_agents,self.n_tasks),dtype=bool) # a "1" in row i, column j means task j is assigned to robot i
        self.assignment_list = [] # ordered list of tasks assigned to each agent, by task number
        self.task_done = np.zeros((self.n_tasks,), dtype=bool)
        self.task_done_history = []
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
        return self.x, self.task_done

    def check_progress(self):
        for robot in range(self.n_agents):
            assigned_tasks = self.assignment_list[robot]
            for task in assigned_tasks:
                loc = self.tasks[task,:]
                dist = np.linalg.norm(loc-self.x[robot,:])
                if dist<self.eps:
                    self.task_done[task] = True
        donecopy = np.copy(self.task_done)
        self.task_done_history.append(donecopy)

        # propagate updates to task_readiness vector
        self.update_task_readiness()

    def build_agent_assignment(self):
        # build agent assignment list
        for robot in range(self.n_agents):
            inds = np.arange(self.n_tasks)
            assigned_tasks = inds[self.assignment_matrix[robot, :]]
            self.assignment_list.append(assigned_tasks)

    def build_assignment_matrix(self):
        for robot in range(self.n_agents):
            assignment = self.assignment_list[robot]
            self.assignment_matrix[robot,assignment] = True
            
    def update_task_readiness(self):
        task_dependency_count = np.sum(self.task_dependency_matrix,axis=1) # count the number of tasks each task is dependent upon
        task_dependency_current = self.task_dependency_matrix @ self.task_done
        self.task_readiness = np.nan_to_num(np.divide(task_dependency_current,task_dependency_count),nan=1,posinf=1) #if zero dependencies, gives 1
        self.task_readiness_history.append(np.copy(self.task_readiness))
        
    def plot(self):
        # animate
        fig = plt.figure()
        ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
        ax.set_aspect('equal')
        ax.grid()
        ax.set_title('Simple Environment')

        Writer = animation.writers['ffmpeg']
        writer = Writer(fps=20, metadata=dict(artist='Me'), bitrate=1800)

        circles = []
        task_circles = []

        def init():
            for task in range(self.n_tasks):
                task_circle = plt.Circle(self.tasks[task, :], 0.3, facecolor='white', edgecolor='k', linewidth=1,alpha=1)
                task_circles.append(task_circle)
                ax.add_patch(task_circles[task])

            for r in range(self.n_agents):
                circles.append(
                    plt.Circle((self.state_history[0][r, :]), 0.2, facecolor='cornflowerblue', edgecolor='k', linewidth=1))
                ax.add_patch(circles[r])

            return circles+task_circles

        def animate(i):
            for r in range(self.n_agents):
                circles[r].center = self.state_history[i][r, :]

            current_task_done = self.task_done_history[i]
            current_readiness = self.task_readiness_history[i]
            for task in range(self.n_tasks):
                c = ''
                if current_task_done[task]:
                    c = 'forestgreen'
                elif current_readiness[task] == 1:
                    c = 'yellow'
                else:
                    c = 'tomato'
                task_circles[task].set_facecolor(c)

            return circles+task_circles

        ani = animation.FuncAnimation(fig, animate, range(len(self.state_history)),
                                      interval=self.dt * 1000, blit=False, init_func=init)
        ani.save('imgs/most_recent_animation.mp4',writer=writer)
        plt.show()
