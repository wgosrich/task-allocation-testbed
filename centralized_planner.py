import numpy as np
import math
from scipy.optimize import linear_sum_assignment
import dependency_test_params
import one_to_one_params


class Planner():

    def __init__(self,env):
        self.env = env

    def agent_state_reset(self, col_ind, n_agents, tasks):
        new_agent_state = [[] for _ in range(n_agents)]
        for i in range(len(col_ind)):
            new_agent_state[i] = tasks[col_ind[i]]
        return new_agent_state

    def plan(self):

        # initialize the states
        #print("agent_state: ",self.env.x)
        #print("task_state", self.env.tasks)
        n_agents = self.env.x.shape[0]
        n_tasks = self.env.tasks.shape[0]
        n_tasks2 = self.env.tasks.shape[0]
        agent_state = self.env.x
        task_state = self.env.tasks
        distanceMatrix = np.zeros((n_agents, n_tasks))
        col_indList = [[] for _ in range(math.ceil(n_tasks / n_agents))]
        #print(col_indList)
        new_col_ind = np.zeros(((math.ceil(n_tasks / n_agents)), n_agents))
        task_list = np.arange(n_tasks)
        #task_list2 = np.arange(n_tasks)
        #print('task_list: ', task_list)
        for i2 in range(math.ceil(n_tasks / n_agents)):
            distanceMatrix = np.zeros((n_agents, len(task_list)))
            n_tasks = len(task_state)
            # print('i2: ', i2)
            # print('col_list before iteration: ', col_indList)
            # print('current task state: ', task_state)
        # compute how many iterations do we need
            #print('current len(task_list): ', len(task_list))
            for i in range(n_agents):
                for j in range(len(task_list)):
                    distanceMatrix[i][j] =(agent_state[i][0] - task_state[j][0])**2 + (agent_state[i][1] - task_state[j][1]) ** 2
            # this matrix is built for applying Hungarian algorithm
            if n_agents < n_tasks:
                dummy_distanceM = np.zeros((n_tasks-n_agents,n_tasks))
                for i4 in range(n_tasks-n_agents):
                    for j in range(n_tasks):
                        dummy_distanceM[i4][j] = np.max(distanceMatrix)
                #print('dummy_list', dummy_distanceM)
                new_distanceM = np.r_[distanceMatrix, dummy_distanceM]
            else:
                new_distanceM = distanceMatrix

                # add dummy matrix to keep the original matrix satisfy the requriement
           # print('This is newM')
            #print(new_distanceM)
            row_ind, col_ind = linear_sum_assignment(new_distanceM)
            col_ind = col_ind[:n_agents]
            n_tasks = n_tasks - len(col_ind)
            #print('This is col_ind:', col_ind)
            # Here is the problem:

            for i6 in range(len(col_ind)):
                new_col_ind[i2][i6] = task_list[col_ind[i6]]
            task_list = [task_list[i] for i in range(len(task_list)) if (i not in col_ind)]
            task_state = [self.env.tasks[task_list[i]] for i in range(len(task_list))]
            # print('task_list after iteration: ', task_list)
            # print('task state after iteration', task_state)
            # print('new_col_ind: ',new_col_ind)
            # print('col_indList before filling: ', col_indList)
            col_indList[i2] = new_col_ind[i2]
            #print('col_List: ', col_indList)



            distanceMatrix = np.delete(distanceMatrix, new_col_ind[i2], axis=1)
            # print('distanceM:')
            # print(distanceMatrix)
            reset_col_ind = new_col_ind.astype(int)
            #print('reset_col_ind: ', reset_col_ind)
            agent_state = self.agent_state_reset(reset_col_ind[i2],n_agents, self.env.tasks)
            #print('This is the current agent states: ', agent_state)
            # print('This is col_indList')
            # print(col_indList)
            # # above here this planner calculates the shortest distance
            # distanceMatrix = np.delete(distanceMatrix,col_ind, axis=1)
            # # Here this planner find the deleted matrix (Needed to be modified)
            # col_indList[i2] = col_ind
        assignment_list = [[] for _ in range(n_agents)]
        # for p in range(n_tasks // n_agents + 1): # 3 // 2 = 1
        #     print(p)
            # for j in range(len(col_indList[p])):
            #     print(j)
        for i5 in range(math.ceil(n_tasks2 / n_agents)):
            #print('This is i5: ', i5)
            for i7 in range(n_agents):
                # print('number of n_tasks', n_tasks)
                # print('This is number of i7')
                # print(i7)
                # print('row_ind before assignment: ', row_ind)
                # print('col_indList before assignment: ', col_indList)
                assignment_list[i7].append(col_indList[i5][i7])
                #print('assignment in the loop: ',assignment_list)
        # for i10 in assignment_list:
        #     for i11 in i10:
        A = np.array(assignment_list)
        A = A.astype(int)
        # print('This is the final assignment_list: ')
        # print(A)
        return A

# Problems:
# 1. what's the assignment_list indicates (what's its entries are?) based on my unerstanding, it is the order of task, follow the order of robots in distanceMatrix
# 2. how to generate the trajectory in python
# i missunderstand the function. I should firstly generate the function of trajectories, then I find the drivative of those trajectories, and then compute the equation (C-CAPT equation)
# initial location is just the initial location, goal location is the goal location, the trajectory matrix is gamma, and X(t) is just teh gamma matrix. -> is this correct?

# Problems 6.16 2020
#1. still not perfectly collision avoided -- modify hungarian algorithm
#2. currently it only satisfies integer times n_agents -- modify the dummy_matrix assignment
#3. still not perfectly optimized.
#4. decouple the program

#1. not fully optimized (debugging)
#2. don't guarantee collision avoidance between iterations - pulse robots after the first iteration (after the first experiment, not now)
#3. precedence

#6.18.2020
# agent reset problem - didn't renew tasks state
# a small tail... this should be solved if we pulse all robots at every iteration.

#6.21.2020
# agent reset opt NO - for uneven (7,8...), it always assign the alst position to the first robot.
# highr than 12 Yes
# 7,8... Yes
# tail problem
# is this a convex problem? - modify the controller that once a position is assigned, the it would be assigned to another.
