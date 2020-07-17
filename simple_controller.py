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
                if (not env.task_done[task]):
                    if not True: #env.task_readiness[task] == 1:  # task is not ready
                        loc = env.x[robot, :]  # don't move
                        dir = np.zeros((2,))
                        actions[robot, :] = dir * self.vel
                        break
                    else:
                        loc = env.tasks[task, :]  # move towards location
                        dir = (loc - env.x[robot, :]) / np.linalg.norm(loc - env.x[robot, :])
                        actions[robot, :] = dir * self.vel
                        break

        return actions
