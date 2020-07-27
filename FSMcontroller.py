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
        for robot, robot2 in range(env.n_agents), range(env.n_agents):
            assigned_tasks = env.assignment_list[robot]
            assigned_tasks2 = env.assignment_list[robot2]
            for task, task2 in assigned_tasks, assigned_tasks2:
                if not env.task_done[task]:
                    loc = env.tasks[task, :]
                    if not env.task_done[task2]:
                        loc2 = env.tasks[task2, :]
                        if np.linalg.norm(loc - loc2) < 2:  # set the recognition radius is 2
                            collision = True
        return collision

    # def decision_making(self, begin, encounter, avoid, end):
    #     env = self.env
    #     for robot in range(env.n_agents):
    #         robot = collisionAvoidance()
    #         if begin is True:
    #             robot.begin
    #         elif encounter is True:
    #             robot.encounter
    #         elif avoid is True:
    #             robot.avoid
    #         elif end is True:
    #             robot.end

    def get_actions(self, t_duration):
        env = self.env
        actions = np.zeros((env.n_agents, 2))
        collision = self.collision_detection()
        for robot, robot2 in range(env.n_agents):
            assigned_tasks = env.assignment_list[robot]
            assigned_tasks2 = env.assignment_list[robot2]
            for task, task2 in assigned_tasks, assigned_tasks2:
                if not (env.task_done[task] and env.task_done[task2]):
                    loc = env.tasks[task, :]
                    loc2 = env.tasks[task2, :]
                    if collision is False:
                        dir = (loc - env.x[robot, :]) / np.linalg.norm(loc - env.x[robot, :])
                    else:
                        dir = ()
                        # relate with the interaction with other robots
                    actions[robot, :] = dir * self.vel
                    break
            return actions

# initialization
# robot class object
# append
#

# create different robot instances and store them in a list First i can press something to start the program,
# then the program will automatically decide when to do circular motion. begin: every robot- robot.begin design the
# dir return action matrix
#
#     def
#
#     def get_actions_straight(self):
#         env = self.env
#         actions = np.zeros((env.n_agents, 2))
#         for robot in range(env.n_agents):
#             assigned_tasks = env.assignment_list[robot]
#             for task in assigned_tasks:
#                 if not env.task_done[task]:
#                     loc = env.tasks[task, :]
#                     dir = (loc - env.x[robot, :]) / np.linalg.norm(loc - env.x[robot, :])
#                     actions[robot, :] = dir * self.vel
#                     break
#         return actions
#
# # if np. <= 2
#         # dir  =
# #elif
#    # dir = straight
#     def git_action_circular(self,t):
#         env = self.env
#         actions = np.zeros((env.n_agents, 2))
#         for robot in range(env.n_agents):
#             assigned_tasks = env.assignment_list[robot]
#             for task in assigned_tasks:
#                 if not env.task_done[task]:
#                     loc = env.tasks[task, :]
#                     dir =
#                     actions[robot, :] = dir * self.vel
#         return actions
