import numpy as np


class Planner:

    def __init__(self,env):
        """
        :param env: the environment instance, so that the planner can reference environment vars
        """
        self.env = env

    def plan(self):
        agent_state = self.env.x
        task_state = self.env.tasks

        """
        :return: assignment_list: a list where entry i is an ordered list of the tasks (by number) assigned to robot i. e.g.
            [2,7,5]
        """
        n_agents = agent_state.shape[0]
        n_tasks = task_state.shape[0]
        assignment_list = [[] for _ in range(n_agents)]

        return assignment_list

    def market_stage(self,t):
        """Market Stage algorithm, from "Optimal Market-based Multi-Robot Task Allocation via Strategic Pricing, by Liu and Shell" """