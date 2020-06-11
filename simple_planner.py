import numpy as np


class SimplePlanner():

    def __init__(self,env):
        self.env = env

    def plan(self):
        agent_state = self.env.x
        task_state = self.env.tasks

        """
        :param agent_state:  n_agents x state_space_dim numpy array, where row i represents the state of agent i
        :param task_state: n_tasks x task_state_space_dim numpy array, where row i represents the state (location) of agent i
        :return: assignment_list: a list where entry i is an ordered list of the tasks (by number) assigned to robot i. e.g.
            [2,7,5]
        """
        n_agents = agent_state.shape[0]
        n_tasks = task_state.shape[0]
        assignment_list = [[] for _ in range(n_agents)]
        i2 = 0
        for i in range(n_tasks):
            assignment_list[i2].append(i)
            i2 += 1
            if i2 > n_agents - 1:
                i2 = 0

        return assignment_list
