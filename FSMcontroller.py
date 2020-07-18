from statemachine import StateMachine, State
import numpy as np



class collisionAvoidance(StateMachine):
    start = State('Start', initial=True)
    straight = State('Straight')
    circle = State('Circle')
    finish = State('Finish')
    # the lines above describe three possible state robots have: start -> straight, straight -> finish, straight ->
    # circle, circle -> ctraight
    begin = start.to(straight)
    encounter = straight.to(circle)
    avoid = circle.to(straight)
    end = straight.to(finish)
    # these are inputs of FSM

    def __init__(self, env):
        self.env = env
        self.vel = 0.5

    def colllision_detection(self):
        # if np.linalg.norm(loc - env.x[robot,:]) <= certain value
        
    def decison_making(self):
        env = self.env
        for robot in range(env.n_agents):
        # create different robot instances and store them in a list
        # First i can press something to start the program, then the program will automatically decide when to do circular motion.
        # begin: every robot- robot.begin
        #   design the dir
        #   return action matrix
    
    def get_actions_straight(self):
        env = self.env
        actions = np.zeros((env.n_agents, 2))
        for robot in range(env.n_agents):
            assigned_tasks = env.assignment_list[robot]
            for task in assigned_tasks:
                if not env.task_done[task]:
                    loc = env.tasks[task, :]
                    dir = (loc - env.x[robot, :]) / np.linalg.norm(loc - env.x[robot, :])
                    actions[robot, :] = dir * self.vel
                    break
        return actions
    def git_action_circular(self):
        env = self.env
        actions = np.zeros(((env.n_agents, 2)))
        for robot in range(env.n_agents):
            assigned_tasks = env.assignment_list[robot]
            for task in assigned_tasks:
                if not env.task_done[task]:
                    loc = env.tasks[task, :]
                    dir =

        


