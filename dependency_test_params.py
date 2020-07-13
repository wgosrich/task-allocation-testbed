import numpy as np

n_agents = 3
n_tasks = 8

dt = 0.1 #time step value
eps = 0.1 #error tolerance for goal completion

#choose random task locations, scale to be between -2 and 2
tasks = (np.random.rand(n_tasks, 2)-0.5)*4

durations = np.random.rand(n_tasks, 1)*3 #choose random task durations between 0 and 3

#generate task dependency matrix
task_dependency_matrix = np.zeros((n_tasks,n_tasks))
# test case: make task 4 dependent on task 1
task_dependency_matrix[3,0] = 1
task_dependency_matrix[3,1] = 1
task_dependency_matrix[5,3] = 1
