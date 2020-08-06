import numpy as np
import math
from scipy.optimize import linear_sum_assignment
import dependency_test_params
import one_to_one_params


class Planner:

    def __init__(self,env):
        self.env = env
    def agent_state_reset(self, col_ind, n_agents, tasks):
        new_agent_state = np.zeros((n_agents,2))
        for i in range(len(col_ind)):
            new_agent_state[i] = tasks[col_ind[i]]
        return new_agent_state

    def create_matrix (self, distanceMatrix, n_agents, task_list,agent_state, task_state ):
        distanceMatrix = np.zeros((n_agents, task_state.shape[0]))
        task_number = task_state.shape[0]
        n_tasks = len(task_state)
        for i in range(n_agents):
            for j in range(task_number):
                distanceMatrix[i,j] = np.linalg.norm(agent_state[i,:]-task_state[j,:])**2
                #distanceMatrix[i][j] = (agent_state[i][0] - task_state[j][0]) ** 2 + (agent_state[i][1] - task_state[j][1]) ** 2
        if n_agents < len(task_list):
            dummy_distanceM = np.zeros((len(task_list) - n_agents, task_number))
            for i4 in range(len(task_list) - n_agents):
                for j in range(task_number):
                    dummy_distanceM[i4][j] = np.max(distanceMatrix)
            new_distanceM = np.r_[distanceMatrix, dummy_distanceM]
        else:
            new_distanceM = distanceMatrix
        row_ind, col_ind = linear_sum_assignment(new_distanceM)
        col_ind = col_ind[:n_agents]
        row_ind = row_ind[:n_agents]
        n_tasks = n_tasks - len(col_ind)
        return row_ind, col_ind, n_tasks

    def retore_position(self, row_ind, col_ind, task_list, i2, n_tasks, n_agents ):
        col_indList = [[] for _ in range(math.ceil(n_tasks / n_agents))]
        new_col_ind = np.zeros(((math.ceil(n_tasks / n_agents)), n_agents))
        for i6, i12 in zip(row_ind, range(len(col_ind))):
            new_col_ind[i2][i6] = task_list[col_ind[i12]]
        task_list = np.array([task_list[i] for i in range(len(task_list)) if (i not in col_ind)])
        task_state = np.array([self.env.tasks[task_list[i]] for i in range(len(task_list))])
        col_indList[i2] = new_col_ind[i2]
        return new_col_ind, col_indList, task_state

    def plan(self):
        n_agents = self.env.x.shape[0]
        n_tasks = self.env.tasks.shape[0]
        n_tasks2 = self.env.tasks.shape[0]
        agent_state = self.env.x
        task_state = self.env.tasks
        task_list = np.arange(n_tasks)
        for i2 in range(math.ceil(n_tasks / n_agents)):
            distanceMatrix = np.zeros((n_agents, task_state.shape[0]))
            row_ind, col_ind, n_tasks = self.create_matrix(distanceMatrix,n_agents,task_list,agent_state,task_state)
            new_col_ind, col_indList, task_state= self.retore_position(row_ind, col_ind, task_list, i2, n_tasks, n_agents)
            reset_col_ind = new_col_ind.astype(int)
            agent_state = self.agent_state_reset(reset_col_ind[i2],n_agents,self.env.tasks)
        assignment_list = [[] for _ in range(n_agents)]
        for i5 in range(math.ceil(n_tasks2 / n_agents)):
            for i7 in range(n_agents):
                assignment_list[i7].append(col_indList[i5][i7])
        A = np.array(assignment_list)
        A = A.astype(int)
        return A
