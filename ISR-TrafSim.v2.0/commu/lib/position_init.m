function position_init()
% Initialize mobility model and start it
% Should be called after initializing topology and parameters

global node Nnodes;
global mobility_model poscommu maxspeedcommu maxpause;
global maxx maxy;

% poscommu:
% 1, 2: starting x and y
% 3: starting time
% 4: moving speed or pasue time
% 5, 6: ending x and y

if strcmp(mobility_model, 'random_waypoint') == 0
    return;
end

poscommu(:, 1:2) = node(:, 1:2);
poscommu(:, 3) = zeros(Nnodes, 1);
poscommu(:, 4:6) = [rand(Nnodes, 1)*maxspeedcommu rand(Nnodes, 1)*maxx rand(Nnodes, 1)*maxy];
