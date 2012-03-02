function bangbang()
% Simulation parameters
xdot_size = 11;
x_size = 11;

dt = 1;
dx = 1;
dxdot = 1;

cost_goal = 0;
g_x = 1;
h_x = 0;

global goal state

%goal = [ceil(xdot_size / 2) ceil(x_size / 2)]'
goal = [0 0]'
state = [-10 0]'

%space = 1000 * ones(xdot_size, x_size);
%space(goal(1), goal(2)) = 0;

% Dynamics
global A B
A = [0 1 
     0 0];
 
B = [0 1]';


while ~atGoal(state)
    state = update(state)
    pause (1)
end
end
%xdot = A*x + B*u

function x = atGoal(current_loc)
    global goal
    
    if current_loc == goal
        x = 1;
       
    else 
        x = 0;
    end

end

function new_state = update(state)
    global A B
    u = 1;
    xdot = A*state + B*u
    
    new_state = zeros(2,1)
    new_state(1) = round(state(1) + xdot(1));
    new_state(2) = round(state(2) + xdot(2));
end