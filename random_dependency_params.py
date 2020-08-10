import numpy as np
class Params:

    def __init__(self, args):
        self.n_agents = 3
        self.n_tasks = args.n_tasks

        self.dt = 0.1 #time step value
        self.eps = 0.1 #error tolerance for goal completion

        #choose random task locations, scale to be between -2 and 2
        self.tasks = (np.random.rand(self.n_tasks, 2)-0.5)*4

        self.durations = np.random.rand(self.n_tasks, 1)*3 #choose random task durations between 0 and 3

        #generate task dependency matrix
        self.task_dependency_matrix = np.zeros((self.n_tasks,self.n_tasks))
        # test case: make task 4 dependent on task 1
        self.task_dependency_matrix[3,0] = 1
        self.task_dependency_matrix[3,1] = 1
        self.task_dependency_matrix[5,3] = 1


