%% import from file
load('matlab_inputs','-mat')
tic
[x,A,Aeq,b,assignment_list] = milp_planner(na,nk,dependency,cost_vector,travel_time);
%travel_time
plan_time = toc;

%save assignment list
save('matlab_out.mat')
