import numpy as np
import math
from scipy.optimize import linear_sum_assignment


class Planner():

    def __init__(self, env):
        self.env = env

    def agent_state_reset(self, col_ind, n_agents, tasks):
        new_agent_state = [[] for _ in range(n_agents)]
        for i in range(len(col_ind)):
            new_agent_state[i] = tasks[col_ind[i]]
        return new_agent_state

    def plan(self):
        # initialize the states
        n_agents = self.env.x.shape[0]
        n_tasks = self.env.tasks.shape[0]
        n_tasks2 = self.env.tasks.shape[0]
        agent_state = self.env.x
        task_state = self.env.tasks
        tasks_assigned = np.ones((self.env.n_tasks,))
        matrix = self.env.task_dependency_matrix
        col_indList = [[] for _ in range(math.ceil(n_tasks / n_agents))]
        new_col_ind = np.zeros(((math.ceil(n_tasks / n_agents)), n_agents))
        # new_col_ind2 = np.zeros((math.ceil(n_tasks / n_agents)), n_agents)
        task_list = np.arange(n_tasks)
        # task_list2 = np.arange(n_tasks)
        for i2 in range(math.ceil(n_tasks / n_agents)):
            distanceMatrix = np.zeros((n_agents, len(task_list)))
            n_tasks = len(task_state)
            task_dependency_checklist = matrix @ tasks_assigned
            # task_dependency_checklist = np.array([task_dependency_checklist[i] for i in range(len(new_col_ind.flatten())) if i not in new_col_ind.flatten()])
            # task_dependency_checklist = task_dependency_checklist.T
            for i in range(n_agents):
                for j in range(len(task_list)):
                    distanceMatrix[i][j] = (agent_state[i][0] - task_state[j][0]) ** 2 + (
                                agent_state[i][1] - task_state[j][1]) ** 2
            for i_1 in range(n_agents):
                for j_1 in range(len(task_list)):
                    task_dependency_checklist = task_dependency_checklist.astype(int)
                    if task_dependency_checklist[j_1] != 0:
                        distanceMatrix[i_1, j_1] = np.max(distanceMatrix)
            # this matrix is built for applying Hungarian algorithm
            if n_agents < n_tasks:
                dummy_distanceM = np.zeros((n_tasks - n_agents, n_tasks))
                for i4 in range(n_tasks - n_agents):
                    for j in range(n_tasks):
                        dummy_distanceM[i4][j] = np.max(distanceMatrix)
                new_distanceM = np.r_[distanceMatrix, dummy_distanceM]
            else:
                new_distanceM = distanceMatrix
                # add dummy matrix to keep the original matrix satisfy the requriement
            row_ind, col_ind = linear_sum_assignment(new_distanceM)
            col_ind = col_ind[:n_agents]
            for i6 in range(len(col_ind)):
                new_col_ind[i2][i6] = task_list[col_ind[i6]]
                # new_col_ind2[i2][i6] = col_ind[i6]
                # tasks_assigned[task_list[col_ind[i6]]] = 0
                # matrix[i6,i6] = 0
            # task_dependency_checklist = matrix @ tasks_assigned
            tasks_assigned = np.delete(tasks_assigned, col_ind)
            # task_dependency_checklist = np.delete(task_dependency_checklist, col_ind)
            m = [i for i in range(matrix.shape[0]) if i not in col_ind]
            matrix = matrix[m]
            matrix = matrix[:, m]
            task_list = [task_list[i] for i in range(len(task_list)) if (i not in col_ind)]
            task_state = [self.env.tasks[task_list[i]] for i in range(len(task_list))]
            col_indList[i2] = new_col_ind[i2]
            reset_col_ind = new_col_ind.astype(int)
            agent_state = self.agent_state_reset(reset_col_ind[i2], n_agents, self.env.tasks)
        assignment_list = [[] for _ in range(n_agents)]
        for i5 in range(math.ceil(n_tasks2 / n_agents)):
            for i7 in range(n_agents):
                assignment_list[i7].append(col_indList[i5][i7])
        A = np.array(assignment_list)
        A = A.astype(int)
        return A

# don't consider dependency chain
# two main (1) random dependencies - keeping ratio of n_dependencies to n_tasks
# (2) dependencies test params (default one) - change the n_task with the same dependencies
# seed it every time run the program

# n_tasks
# n_tasks, n_dependencies
# seeds
