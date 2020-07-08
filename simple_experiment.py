import numpy as np
import argparse


def set_seed(seed_value):
    if seed_value == None:
        seed_val = np.random.randint(0, 1000)
    else:
        seed_val = seed_value
    np.random.seed(seed_val)
    return seed_val


def get_args():
    parser = argparse.ArgumentParser(description=None)
    parser.add_argument('--planner', default='simple_planner', dest='planner', type=str, help="planner to use: can be simple_planner, from_file, etc.?")
    parser.add_argument('--seed', default=None, dest='seed_val', type=int, help="seed value for random number generator, int")
    parser.add_argument('--filename', default=None, dest='filename_str', type=str, help="if importing from file, specify which file. Otherwise, choose the most recent matlab_out")
    return parser.parse_args()



def get_list_from_file(filename='matlab_out'):
    d = sio.loadmat(filename)
    list_raw = d['assignment_list']
    print(list_raw)
    a_list = []
    for item in list_raw[0]:
        a_list.append(item[0])
    return a_list




if __name__ == "__main__":
    args = get_args()
    seed_val = set_seed(args.seed_val)

    import scipy.io as sio
    import simple_env
    import simple_planner
    import simple_controller
    from datetime import datetime
    import os
    import centralized_hungarian_nx
    import matplotlib.pyplot as plt
    import one_to_one_params
    import dependency_test_params
    from matplotlib import animation

    # create environment ---------------------------------------------------
    nsteps = 1000
    env = simple_env.SimpleEnv(dependency_test_params)
    robot_diameter = env.robot_diameter

    #process arguments------------------------------------------------------
    if args.planner == 'simple_planner':
        planner = simple_planner.SimplePlanner(env)
        assignment_list = planner.plan()
    elif args.planner == 'centralized_planner':
        raise Exception('centralized planner not yet implemented.')
        assignment_list = planner.plan()

    elif args.planner == 'from_file':
        if args.filename_str != None:
            assignment_list = get_list_from_file(args.filename_str)
        else:
            assignment_list = get_list_from_file()
    else:
        raise Exception('')


    controller = simple_controller.SimpleController(env)
    env.assignment_list = assignment_list
    env.build_assignment_matrix()

    # run controller
    t = 0
    done = False
    current_tasks = np.zeros((env.n_agents,))

    while t < nsteps and not done:
        # calculate controls
        actions = controller.get_actions()

        # apply controls
        newstate, completion = env.step(actions)

        done = completion.all()
        t += 1

    env.plot()

    # export data

    today = datetime.now()

    dir_string = "data/" + today.strftime('%Y%m%d')
    try:
        os.mkdir(dir_string)
    except FileExistsError:
        pass
    np.savez(dir_string + "/data_" + today.strftime("%H%M"), seed=seed_val, assignment_list=assignment_list,
             t=t / env.dt, allow_pickle=True)

    # generate travel time for matlab
    tt = np.zeros((env.n_tasks ** 2, 1))
    for ii in range(env.n_tasks):
        for jj in range(env.n_tasks):
            tt[ii * env.n_tasks + jj] = np.linalg.norm(env.tasks[ii, :] - env.tasks[jj, :])

    sio.savemat("matlab_inputs", {"na": env.n_agents, "nk": env.n_tasks, "dependency": env.task_dependency_matrix,
                                  "cost_vector": np.zeros((env.n_tasks,)), "travel_time": tt})