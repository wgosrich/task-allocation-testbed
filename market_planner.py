import numpy as np


class Planner:

    def __init__(self,env):
        """
        :param env: the environment instance, so that the planner can reference environment vars
        """
        self.env = env
        self.n_agents = env.n_agents
        self.n_tasks = env.n_tasks
        self.assignment = np.zeros((self.n_agents,self.n_tasks))==1 #make boolean false
        self.agent_inds = np.arange(self.n_agents)
        self.conflict_agents = np.array([])
        self.prices = np.zeros((self.n_tasks,))

    def plan(self):
        agent_state = self.env.x
        task_state = self.env.tasks

        """
        :return: assignment_list: a list where entry i is an ordered list of the tasks (by number) assigned to robot i. e.g.
            [2,7,5]
        """

        assignment_list = [[] for _ in range(self.n_agents)]


        return assignment_list

    def market_stage(self,t,):
        """Market Stage algorithm, from "Optimal Market-based Multi-Robot Task Allocation via Strategic Pricing, by Liu and Shell" """
        conflict_tasks = t
        new_conflict_agents = self.agent_inds[self.assignment[:,t]] #get the agent indices that are disputing the current task t

        #initialize list of conflict agents with new agents from conflicted task
        conflict_agents = self.conflict_agents
        for a in new_conflict_agents:
            conflict_agents = np.append(conflict_agents,a) if a not in conflict_agents

        sink = 0
        weak_agent = 0
        weak_task = 0
        assignment_by_task = np.zeros((self.n_tasks,))

        while sink == 0:
            delta = np.inf

            # generate list of other tasks
            other_tasks = list(range(self.n_tasks))
            for task in conflict_tasks:
                other_tasks.remove(task)

            for a in conflict_agents:

                #calculate budget for all tasks
                budget = -1*np.linalg.norm(self.env.tasks-self.env.x[a,:])
                # TODO is the above correct??
                margins = budget - self.prices
                second_task = np.argmax(margins)

                current_task = np.arange(self.n_tasks)[self.assignment[a,:]]
                current_margin = margins[current_task]
                # TODO when expanding this to multiple agents, can instead consider the REWARD, raising task reward until
                # an agent is incentivized to add it into the schedule

                second_margin = margins[second_task]

                if current_margin - second_margin < delta:
                    delta = current_margin - second_margin
                    weak_agent = a
                    weak_task = second_task

            for task in conflict_tasks:
                self.prices[task] = self.prices[task] + delta

            #TODO finish this. at line 13 in the alg in the paper


