import numpy as np
class Params:

    def __init__(self, args):
        self.n_agents = args.n_agents
        self.n_tasks = args.n_tasks

        n_dependencies = args.n_dependencies

        self.dt = 0.1 #time step value
        self.eps = 0.1 #error tolerance for goal completion

        #choose random task locations, scale to be between -2 and 2
        self.tasks = (np.random.rand(self.n_tasks, 2)-0.5)*4

        self.durations = np.random.rand(self.n_tasks, 1)*3 #choose random task durations between 0 and 3

        #generate task dependency matrix
        mask = np.tril(np.random.rand(self.n_tasks,self.n_tasks)) - np.eye(self.n_tasks) #lower triangular random matrix with negative diagonal

        self.task_dependency_matrix = np.zeros_like(mask)

        for i in range(n_dependencies):
            maxind = np.unravel_index(np.argmax(mask), mask.shape)
            self.task_dependency_matrix[maxind] = 1
            mask[maxind] = 0



