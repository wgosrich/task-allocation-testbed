%-------------problem specific inputs---------------------------------
%---------------------------------------------------------------------
na = 2; %number of agents
nk = 12; %number of tasks

%dependency matrix for 12 task setup: (i,j) =1 means task i depends on task
%j
dependency = zeros(12);
dependency(7:12,1:6) = eye(6); %placing on retrieving
dependency(10,7) = 1; dependency(10,8) = 1; %4 on 1 and 2
dependency(11,8) = 1; dependency(11,9) = 1; %5 on 2 and 3
dependency(12,10) = 1; dependency(12,11) = 1; %6 on 4 and 5

%vector of task costs
cost_vector = [2 2 2 6 2 2 1 1 1 2 2 2];

%dummy vector of task travel times
travel_time = zeros(nk*nk,1);

%---------------------------------------------------------------------
%---------------end problem specific inputs---------------------------

[x,A,Aeq,b,assignment_list] = milp_planner(na,nk,dependency,cost_vector, travel_time);

%% test 2
na = 3
nk = 4

task_states = [1,6; 1,1; -1,1;1,-1];

robot_states = [0,0;0,0;0,0];

%calculate costs
% TODO: how to handle robots in different positions, i.e. heterogenous???

cost_vector = zeros(1,nk);
for ii = 1:nk
    cost_vector(ii) = norm(task_states(ii,:));
end
%cost_vector
dependency = zeros(nk);
dependency(4,1) = 1;

travel_time = ones(nk*nk,1);
x0 = zeros(92,1);


[x,A,Aeq,b,assignment_list] = milp_planner(na,nk,dependency,cost_vector,travel_time);


