import numpy as np

class SimplePlanner():


    def __init__(self):
        pass

    def plan(self,agent_state,task_state):
        n_agents = agent_state.shape[0]
        n_tasks = task_state.shape[0]
        assert n_agents == n_tasks

        return np.eye(n_tasks,dtype=bool)