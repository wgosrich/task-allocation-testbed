import numpy as np
import argparse
import scipy.io as sio
from datetime import datetime
import os
import importlib
import time

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
                        help="controller to use: can be simple_controller, FSMcontroller")
    parser.add_argument('--seed', default=None, dest='seed_val', type=int,
                        help="seed value for random number generator, int")
    parser.add_argument('--filename', default=None, dest='filename_str', type=str,
                        help="if importing from file, specify which file. Otherwise, choose the most recent matlab_out")
    parser.add_argument('--params', default='dependency_test_params',dest='params_name',type=str,help='set the parameter file to be used')
    parser.add_argument('--n_tasks', default=8, dest='n_tasks', type=int, help='number of tasks to use in simulation')
    parser.add_argument('--save', default=False, dest='save', action='store_true', help='save the data to the csv file')
    return parser.parse_args()


def get_list_from_file(filename='matlab_out'):
    d = sio.loadmat(filename)
    list_raw = d['assignment_list']
    plan_time = d['plan_time']
    print(list_raw)
    a_list = []
    for item in list_raw[0]:
        a_list.append(item[0])
    return a_list, plan_time[0][0]


if __name__ == "__main__":
    args = get_args()
    seed_val = set_seed(args.seed_val)

    import simple_env
    import simple_planner
    import simple_controller
    import centralized_hungarian_nx
    import FSMcontroller

    # create environment ---------------------------------------------------------
    nsteps = 1000
    params = importlib.import_module(args.params_name)
    params = params.Params(args)
    env = simple_env.SimpleEnv(params)
    robot_diameter = env.robot_diameter

    # process planner argument------------------------------------------------------
    if args.planner == 'from_file':
        if args.filename_str != None:
            assignment_list, plan_time = get_list_from_file(args.filename_str)
        else:
            assignment_list, plan_time = get_list_from_file()

    else:
        planner_file = importlib.import_module(args.planner)
        planner = planner_file.Planner(env)
        start_time = time.time()
        assignment_list = planner.plan()
        end_time = time.time()
        plan_time = end_time-start_time

    # process controller argument-------------------------------------------------
    if args.controller == 'simple_controller':
        controller = simple_controller.SimpleController(env)
    elif args.controller == 'FSMcontroller':
        controller = FSMcontroller.CollisionAvoidance(env)
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
             t=t*env.dt, allow_pickle=True)


    if args.save:
        datafile = open('data/data_log_aug.csv','a')
        datastring = '{},{},{},{},{},{},{},{},{}\n'.format(seed_val, args.planner, args.controller, t*env.dt, plan_time, env.n_agents, env.n_tasks, env.task_dependency_matrix.sum(), today.strftime('%Y%m%d')+today.strftime("%H%M"))
        datafile.write(datastring)
        datafile.close()

    # generate travel time for matlab
    ll = env.n_agents+env.n_tasks
    ltasks = np.concatenate((env.state_history[0],env.tasks),axis=0)
    tt = np.zeros((ll ** 2, 1))
    for ii in range(ll):
        for jj in range(ll):
            tt[ii * ll + jj] = np.linalg.norm(ltasks[ii, :] - ltasks[jj, :])

    ldurations = np.concatenate((np.zeros((env.n_agents, 1)), env.durations))

    ldependency = np.zeros((ll,ll))
    ldependency[env.n_agents:,env.n_agents:] = env.task_dependency_matrix
    sio.savemat("matlab_inputs", {"na": env.n_agents, "nk": env.n_tasks+env.n_agents, "dependency": ldependency,
                                  "cost_vector": ldurations, "travel_time": tt})
