function position_update()
% Update position when nodes are moving

global node Nnodes;
global mobility_model poscommu maxspeedcommu maxpause;
global current_time FIRST_TIME;
global maxx maxy;
global intervalcommu;

% poscommu:
% 1, 2: starting x and y
% 3: starting time
% 4: moving speed or pasue time
% 5, 6: ending x and y
%if FIRST_TIME ==0
%    FIRST_TIME=1;
%    for i=1:Nnodes
%        poscommu(i,3) = current_time;
%    end
%    return;
%end

if strcmp(mobility_model, 'random_waypoint') == 0
    return;
end

for i=1:Nnodes
    if poscommu(i, 3) > current_time
        disp(['position: the starting time ' num2str(poscommu(i, 3)) ' for node ' num2str(i) ' is larger than current time ' num2str(current_time)]);
        continue;
    end
    if poscommu(i, 1)==poscommu(i, 5) & poscommu(i, 2)==poscommu(i, 6) & 0
        % I am pause
        while 1
            % if i==1, disp(['position_update: at time ' num2str(current_time) ' node 1 is pause at (x, y)=' num2str(poscommu(1, 1)) ', ' num2str(poscommu(1, 2))]); end
            if (poscommu(i, 3)+poscommu(i, 4)) >= current_time
                node(i, 1:2) = poscommu(i, 1:2);
                break;
            end
            % moving
            poscommu(i, 3) = poscommu(i, 3) + poscommu(i, 4);
            poscommu(i, 5) = rand*maxx;
            poscommu(i, 6) = rand*maxy;
            poscommu(i, 4) = rand*maxspeedcommu;
            tempt = sqrt((poscommu(i, 1)-poscommu(i, 5))^2+(poscommu(i, 2)-poscommu(i, 6))^2)/poscommu(i, 4);
            if (poscommu(i, 3)+tempt) >= current_time
                u=(current_time-poscommu(i, 3))/tempt;
                node(i, 1:2) = poscommu(i, 1:2)*(1-u) + poscommu(i, 5:6)*u;
                break;
            end
            % pause
            poscommu(i, 1:2) = poscommu(i, 5:6);
            poscommu(i, 3) = poscommu(i, 3) + tempt;
            poscommu(i, 4) = rand*maxpause;
        end
    elseif 0
        % I am moving
        while 1
            % if i==1, disp(['position_update: at time ' num2str(current_time) ' node 1 is moving from (x, y)=' num2str(poscommu(1, 1)) ', ' num2str(poscommu(1, 2)) ' at speed=' num2str(poscommu(1, 4))]); end
            tempt = sqrt((poscommu(i, 1)-poscommu(i, 5))^2+(poscommu(i, 2)-poscommu(i, 6))^2)/poscommu(i, 4);
            if (poscommu(i, 3)+tempt) >= current_time
                u=(current_time-poscommu(i, 3))/tempt;
                node(i, 1:2) = poscommu(i, 1:2)*(1-u) + poscommu(i, 5:6)*u;
                break;
            end
            poscommu(i, 1:2) = poscommu(i, 5:6);
            poscommu(i, 3) = poscommu(i, 3) + tempt;
            poscommu(i, 4) = rand*maxpause;
            if (poscommu(i, 3)+poscommu(i, 4)) >= current_time
                node(i, 1:2) = poscommu(i, 1:2);
                break;
            end
            poscommu(i, 3) = poscommu(i, 3) + poscommu(i, 4);
            poscommu(i, 5) = rand*maxx;
            poscommu(i, 6) = rand*maxy;
            poscommu(i, 4) = rand*maxspeedcommu;
        end
    else
        % I am moving
        speed = sqrt((poscommu(i, 1)-poscommu(i, 5))^2+(poscommu(i, 2)-poscommu(i, 6))^2)/0.6;
        theta = atan2((poscommu(i, 6)-poscommu(i, 2)),(poscommu(i, 5)-poscommu(i, 1)));
        deltatime = (current_time - poscommu(i, 3));
        xinc = speed * cos(theta) * deltatime;
        yinc = speed * sin(theta) * deltatime;
        poscommu(i,3) = current_time;
        node(i,1) = node(i,1) + xinc;
        node(i,2) = node(i,2) + yinc;
    end
end
R = rem(current_time,0.0002);
if 1 %R == 0
    disp('*****************PLOT#############################')
    %plot(node(:, 1),node(:,2),'b.');
    %pause(0.00000000000001)
end

% disp(['position_update: at time ' num2str(current_time) ' node 1 is located at (x, y)=' num2str(node(1, 1)) ', ' num2str(node(1, 2))]);

return;
