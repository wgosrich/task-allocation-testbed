import numpy as np

class SimpleEnv():
    """
    A simple 2D environment with robots represented as discs with 1D dynamics:
    x_dot = u

    """

    def __init__(self):
        self.dt = 0.1
        self.n_agents = 3
        self.n_goals = 3

        #initialize robot state
        self.x = np.random.rand(n_agents,2)

        #choose random goals
        self.goals = np.random.rand(n_goals,2)

    def step(self, action):
        """
        :param action: n_agents x 2 numpy array
        :return: new state x
        """
        #TODO: check if action is valid

        self.x = self.x + self.dt*action
        return self.x