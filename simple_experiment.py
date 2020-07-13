import numpy as np
import argparse
import scipy.io as sio
from datetime import datetime
import os
import importlib

def set_seed(seed_value):
    if seed_value == None:
        seed_val = np.random.randint(0, 1000)
    else:
        seed_val = seed_value
    np.random.seed(seed_val)
    return seed_val


def get_args():
    parser = argparse.ArgumentParser(description=None)
    parser.add_argument('--planner', default='simple_planner', dest='planner', type=str,
                        help="planner to use: can be simple_planner, from_file, etc.?")
    parser.add_argument('--controller', default='simple_controller', dest='controller', type=str,
                        help="controller to use: can be simple_controller, others to be implemented")
    parser.add_argument('--seed', default=None, dest='seed_val', type=int,
                        help="seed value for random number generator, int")
    parser.add_argument('--filename', default=None, dest='filename_str', type=str,
                        help="if importing from file, specify which file. Otherwise, choose the most recent matlab_out")
    parser.add_argument('--params', default='dependency_test_params',dest='params_name',type=str,help='set the parameter file to be used')
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

    import simple_env
    import simple_planner
    import simple_controller
    import centralized_hungarian_nx

    # create environment ---------------------------------------------------------
    nsteps = 1000
    params = importlib.import_module(args.params_name)
    env = simple_env.SimpleEnv(params)
    robot_diameter = env.robot_diameter

    # process planner argument------------------------------------------------------
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
        raise Exception('invalid argument for --planner')

    # process controller argument-------------------------------------------------
    if args.controller == 'simple_controller':
        controller = simple_controller.SimpleController(env)
    else:
        raise Exception('invalid argument for --controller')

    env.assignment_list = assignment_list
    env.build_assignment_matrix()

    # run controller---------------------------------------------------------------
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
    ll = env.n_agents+env.n_tasks
    ltasks = np.concatenate((env.state_history[0],env.tasks),axis=0)
    tt = np.zeros((ll ** 2, 1))
    for ii in range(ll):
        for jj in range(ll):
            tt[ii * ll + jj] = np.linalg.norm(ltasks[ii, :] - ltasks[jj, :])
    print(env.durations)
    print(np.zeros((env.n_agents, 1)))
    ldurations = np.concatenate((np.zeros((env.n_agents, 1)), env.durations))
    print(ldurations)

    ldependency = np.zeros((ll,ll))
    ldependency[env.n_agents:,env.n_agents:] = env.task_dependency_matrix
    sio.savemat("matlab_inputs", {"na": env.n_agents, "nk": env.n_tasks+env.n_agents, "dependency": ldependency,
                                  "cost_vector": ldurations, "travel_time": tt})
