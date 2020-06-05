import numpy as np

n_agents = 3
n_tasks = 3

dt = 0.1 #time step value
eps = 0.1 #error tolerance for goal completion

#choose random task locations, scale to be between -2 and 2
tasks = (np.random.rand(n_tasks, 2)-0.5)*4

#generate task dependency matrix
task_dependency_matrix = np.zeros((n_tasks,n_tasks))

