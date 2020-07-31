from statemachine import StateMachine, State
import numpy as np


class CollisionAvoidance(StateMachine):
    start = State('Start', initial=True)
    straight = State('Straight')
    circle = State('Circle')
    finish = State('Finish')
    # the lines above describe three possible state robots have: start -> straight, straight -> finish, straight ->
    # circle, circle -> straight
    begin = start.to(straight)
    encounter = straight.to(circle)
    avoid = circle.to(straight)
    end = straight.to(finish)

    # these are inputs of FSM

    # robot = collisionAvoidance()
    # robot.current_state

    def __init__(self, env):
        self.env = env
        self.vel = 0.5

    def collision_detection(self):
        # globally detect distance between each robot at every time duration
        collision = False
        env = self.env
        # actions = np.zeros((env.n_agents, 2))
        for robot in range(env.n_agents):
            for robot2 in range(env.n_agents):
                assigned_tasks = env.assignment_list[robot]
                assigned_tasks2 = env.assignment_list[robot2]
            for task in assigned_tasks:
                for task1 in assigned_tasks2:
                    if not env.task_done[task]:
                        loc = env.tasks[task, :]
                    if not env.task_done[task1]:
                        loc2 = env.tasks[task1, :]
                    if np.linalg.norm(loc - loc2) < 2:  # set the recognition radius is 2
                        collision = True
        return collision
    # collision - boolean array

    def get_actions(self):
        env = self.env
        actions = np.zeros((env.n_agents, 2))
        collision = self.collision_detection()
        for robot in range(env.n_agents):
            for robot1 in range(env.n_agents):
                assigned_tasks = env.assignment_list[robot]
                assigned_tasks1 = env.assignment_list[robot1]
            for task in assigned_tasks:
                for task1 in assigned_tasks1:
                    if not (env.task_done[task] and env.task_done[task1]):
                        loc = env.tasks[task, :]
                        loc1 = env.tasks[task1, :]
                    if collision is False:
                        dir = (loc - env.x[robot, :]) / np.linalg.norm(loc - env.x[robot, :])
                    else:
                        dir = self.make_trajectory(loc, loc1)
                        # relate with the interaction with other robots
                    actions[robot, :] = dir * self.vel
                    break
            return actions

    def make_trajectory(self, loc, loc1):
        x = (loc[0] + loc1[0]) / 2
        y = (loc[1] + loc1[1]) / 2
        dir = np.cross((loc - [x,y]) / np.linalg.norm(loc - [x,y]), [0,0,1])[:2]
        return dir

