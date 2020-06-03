import numpy as np

class SimplePlanner():


    def __init__(self):
        pass


    def plan(self,agent_state,task_state):
        n_agents = agent_state.shape[0]
        n_tasks = task_state.shape[0]
        assignment_list = [[] for _ in range(n_agents)]
        i2 = 0
        for i in range(n_tasks):
            assignment_list[i2].append(i)
            i2+=1
            if i2 > n_agents-1:
                i2 = 0

        return assignment_list