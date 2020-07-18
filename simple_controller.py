import numpy as np

class SimpleController():

    def __init__(self, env):
        self.env = env
        self.vel = 0.5

    def get_actions(self):
        env = self.env

        # calculate controls
        actions = np.zeros((env.n_agents, 2))
        for robot in range(env.n_agents):
            assigned_tasks = env.assignment_list[robot]
            for task in assigned_tasks:
                if not env.task_done[task]:
                    loc = env.tasks[task, :]
                    dir = (loc - env.x[robot, :]) / np.linalg.norm(loc - env.x[robot, :])
                    actions[robot, :] = dir * self.vel
                    break
        return actions

