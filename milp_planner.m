function [x,A,Aeq,b,assignment_list] = milp_planner(na,nk,dependency,cost_vector,travel_time)

    %% MILP formulation of task allocation problem with task dependencies

    %matlab MILP solver


    nconstraints = 1000;
    M = 10000; %arbitrary large number that must be larger than the max time expected

    x_len = na*nk; %length of x
    o_len = na*nk*nk; %length of o
    v_len = x_len;
    z_len = x_len;
    st_len = nk;
    ft_len = nk;

    x_ind = 1; %start ind for x
    o_ind = 1 + x_len; %start ind for o
    v_ind = o_ind + o_len;
    z_ind = v_ind + v_len;
    st_ind = z_ind+z_len;
    ft_ind = st_ind+st_len;
    total_len = ft_ind+ft_len-1

    %structure of x vector:
    % [ x; o; v; z; st; ft]
    % iterates over agents, then tasks:
    % i.e. [x_k1^a1, x_k2^a1,...,x_k1^a2, x_k2^a2,...] 

    %f = objective function

    %intcon = indices of entries in x required to be integers
    intcon = double(1:(st_ind-1));





    %% Aeq = matrix of coefficients on x s.t. Aeq x = beq

    Aeq = zeros(nconstraints,total_len);
    beq = zeros(nconstraints,1);


    %constraint a - each task assigned to one robot
    %nk constraints
    for ii = 0:na-1
        Aeq(1:nk,1+ii*nk:ii*nk+nk) = eye(nk);
    end

    constraint_ind = nk+1;
    beq(1:nk) = ones(nk,1);

    %constraint b - each robot performs one task first
    %na constraints

    for ii = 0:na-1
       Aeq(constraint_ind + ii,(v_ind + ii*nk):(v_ind+(ii+1)*nk-1)) = ones(1,nk);
    end
    beq(constraint_ind:constraint_ind+na-1) = ones(na,1);
    constraint_ind = constraint_ind+na;


    %constraint c - each robot performs one task last
    %na constraints

    for ii = 0:na-1
       Aeq(constraint_ind + ii,(z_ind + ii*nk):(z_ind+(ii+1)*nk-1)) = ones(1,nk);
    end

    beq(constraint_ind:constraint_ind+na-1) = ones(na,1);
    constraint_ind = constraint_ind+na; 


    %constraint e - every task assigned to robot a has a predecessor, except
    %for the first task

    %first, adding the o portion of the constraint 
    for ii = 0:na-1
        for jj = 0:nk-1
            Aeq(constraint_ind+ii*(nk):constraint_ind+ii*(nk)+nk-1,o_ind+jj*(nk)+ii*(nk*(nk)):o_ind+nk-1+jj*(nk)+ii*(nk*(nk))) = eye(nk);
        end
    end

    %now, add the v part
    for ii = 0:na-1
       Aeq(constraint_ind + ii*(nk):constraint_ind + nk-1 + ii*(nk),(v_ind + ii*nk):(v_ind+(ii)*nk+nk-1)) = eye(nk);
    end

    %subtract x_k^a
    for ii = 0:na-1
       Aeq(constraint_ind + ii*(nk):constraint_ind + nk-1 + ii*(nk),(1 + ii*nk):(1+(ii)*nk+nk-1)) = -1*eye(nk);
    end

    constraint_ind = constraint_ind+na*nk;

    %add 2 new constraints to ensure o_11=0, for example
    for ii = 0:na-1
        for jj = 0:nk-1
            Aeq(constraint_ind+ii ,nk* nk*ii+o_ind+jj+1+jj*nk) = 1;
        end
    end

    constraint_ind = constraint_ind+2;

    % constraint f - every task assigned to robot a has a subsequent task,
    % except for the last task

    %first, adding the o portion of the constraint 
    for ii = 0:na-1
        for jj = 0:nk-1
            Aeq(constraint_ind+ii*(nk)+jj,o_ind+jj*(nk)+ii*(nk*(nk)):o_ind+nk-1+jj*(nk)+ii*(nk*(nk))) = ones(1,nk);
        end
    end

    %now, add the z part
    for ii = 0:na-1
       Aeq(constraint_ind + ii*(nk):constraint_ind + nk-1 + ii*(nk),(z_ind + ii*nk):(z_ind+(ii)*nk+nk-1)) = eye(nk);
    end

    %subtract x_k^a
    for ii = 0:na-1
       Aeq(constraint_ind + ii*(nk):constraint_ind + nk-1 + ii*(nk),(1 + ii*nk):(1+(ii)*nk+nk-1)) = -1*eye(nk);
    end

    constraint_ind = constraint_ind+na*nk;
    ['number of equality constraints: ',num2str(constraint_ind-1)]
    Aeq(constraint_ind:end,:) = [];
    beq(constraint_ind:end) = [];
    %% A = matrix of coefficients on x s.t. Ax <= b are inequality constraints


    A = zeros(nconstraints,total_len);
    b = zeros(nconstraints,1);
    %constraint g (new constraint - not from paper
    %if task k depends on k', then stk - ftk' >= 0 ---> -stk + ftk' <= 0

    % build A from dependency matrix
    constraint_ind = 1;
    for i = 0:nk-1
        for j = 0:nk-1
            if dependency(i+1,j+1) %if task i+1 depends on j+1
                A(constraint_ind,st_ind+i) = -1;
                A(constraint_ind,ft_ind+j) = 1;
                constraint_ind = constraint_ind+1;
            end
        end
    end

    % TODO: modify this constraint to actually use duration
    
    % constraint i
    %start time minus finish time must be less than (negative) duration
    A(constraint_ind:constraint_ind+nk-1, st_ind:st_ind+nk-1) = eye(nk);
    A(constraint_ind:constraint_ind+nk-1, ft_ind:ft_ind+nk-1) = -1*eye(nk);
    b(constraint_ind:constraint_ind+nk-1) = -1*cost_vector;

    constraint_ind = constraint_ind+nk;

    %{
    % constraint d -- capacity -- WAIT this constraint doesn't make sense? 
    % it's adding all of the tasks of each robot for a total cumulative work
    % done... that's not what we want
    %workload of all tasks on all robots is 1
    %capacity (q_a) of all robots is 1

    for ii = 0:na-1
       A(constraint_ind + ii,(x_ind + ii*nk):(x_ind+(ii+1)*nk-1)) = ones(1,nk);
    end

    b(constraint_ind:constraint_ind+na-1) = 100*ones(na,1);
    constraint_ind = constraint_ind+na;
    %}

    % IN PROGRESS: modify this constraint to actually use travel time
    % constraint  j -- if tasks k and k' are completed consecutively by robot a
    % (i.e. o_kk'^a is true) then ftk+tt_kk'-stk' <= 0 

    for ia = 0:na-1
        for ik = 0:nk-1
            for ikprime = 0:nk-1
                tt = travel_time(ik*nk + ikprime +1);
                A(constraint_ind,o_ind+ia*(nk^2)+ik*nk+ikprime) = M;
                A(constraint_ind,ft_ind+ik) = 1;
                A(constraint_ind,st_ind+ikprime) = -1;
                b(constraint_ind) = M - tt;
                constraint_ind = constraint_ind+1;
            end
        end
    end

    A(constraint_ind:end,:) = [];
    b(constraint_ind:end) = [];

    A2show = A;
    A2show(A2show==M) = 1;
    image = mat2gray(A2show);
    imshow(image)

    %lb,ub are the lower and upper bounds of x - since x is binary, this is
    %zeros and ones
    lb = zeros(total_len,1);
    ub = ones(total_len,1);
    ub(st_ind:end) = inf;

    

    %objective function:
    f = zeros(total_len,1);
    f(end) = 1;

    options = optimoptions('intlinprog','MaxNodes',200000);
    x = intlinprog(f, intcon, A, b, Aeq, beq, lb, ub,options)
    %% display
    c = clock;
    time = [num2str(c(4)),':',num2str(c(5))];
    namestring = convertCharsToStrings(['data/',date,time,'-data.mat']);
    save(namestring,'x');


    start_times = zeros(nk,3);
    for ii = 0:nk-1
        for ia = 0:na-1
            if x(x_ind+ii+ia*nk)
                start_times(ii+1,:) = [x(st_ind+ii), ii+1, ia+1];
            end
        end
    end
    sorted_start_times = sortrows(start_times);
    
    assignment_list = cell(1,na);
    for ii = 1:nk
        disp(['at time ', num2str(sorted_start_times(ii,1)), ' task ' ,num2str(sorted_start_times(ii,2)), ' is started by robot ', num2str(sorted_start_times(ii,3))]); 
        current_cell = assignment_list{sorted_start_times(ii,3)};
        current_cell(end+1) = sorted_start_times(ii,2);
        assignment_list{sorted_start_times(ii,3)} = current_cell;
    end


    for ia = 0:na-1
        for ik = 0:nk-1
            for ikprime = 0:nk-1
                if x(o_ind+ia*(nk^2)+ik*nk+ikprime)
                   disp(['robot ',num2str(ia+1), ' performs task ' ,num2str(ikprime+1),' after task ',num2str(ik+1)])
                end
            end
        end
    end

    disp(assignment_list)
        
end


