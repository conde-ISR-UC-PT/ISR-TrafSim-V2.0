%--------------------------------------------------------------------------
%                           ISR-TrafSim v2.0
%                        Copyright (C) 2010-2013
%
%--------------------------------------------------------------------------
% This Matlab file is part of the ISR-TrafSim: a Matlab
% library for traffic simulation and pose estimation in Urban environments,
% namely roundabouts and crossroads.
%
% http://www.isr.uc.pt/~conde/isr-trafsim/
%
%-CITATION---------------------------------------------------------------------------
% If you use this software please cite one of the following papers:
% 1) L.C.Bento, R.Parafita, S.Santos and U.Nunes, An Intelligent Traffic Management
% at Intersections legacy mode for vehicles not equipped with V2V and V2I Communications,
% 16th IEEE Int.Conf. Intelligent Transportation Systems, Netherlands, 2013.
% 2) L.C.Bento, R.Parafita and U.Nunes, Inter-vehicle sensor fusion for accurate vehicle
% localization supported by V2V and V2I communications, 15th IEEE Int.Conf. Intelligent
% Transportation Systems, USA, 2012.
% 3) L.C.Bento, R.Parafita and U.Nunes, Intelligent traffic management at intersections
% supported by V2V and V2I communications, 15th IEEE Int.Conf. Intelligent
% Transportation Systems, USA, 2012.
%
%-DESCRIPTION------------------------------------------------------------------------
%
% Manage the release of vehicles into simulator
%
%-USE----------------------------------------------------------------------
%
%  isr_tfs_launch(arg)
% -> Input(s)
%   » arg         -   Current linear velocity
%   	init        -   Initializes all necessary tables
%   	check       -   Check the release of new vehicles
%
% -> Output(s)
%   » Array with [x y t Pwr Pwf]
%       (x,y,t) -   Updated pose of vehicle
%       Pwr     -   Pulses generated on rear wheel encoder
%       Pwf     -   Pulses generated on front wheel encoder
%
%-DISCLAIMER-------------------------------------------------------------------------
% This program is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY;
% without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
% You can use this source code without licensing fees only for NON-COMMERCIAL research
% and EDUCATIONAL purposes only.
% You cannot repost this file without prior written permission from the authors.
%
%-AUTHORS------------------------------------------------------------------
%   Urbano Nunes*
%   Luis Conde Bento**
%   Ricardo Parafita*
%   Sergio Santos*
%
%  *Institute of Systems and Robotics   - University of Coimbra
% **School of Technology and Management - Polytechnic Institute of Leiria
%--------------------------------------------------------------------------

function [] = launch_isr_trafsim(arg)

if(strcmp(arg,'init'))
    launch_init();
elseif(strcmp(arg,'check'))
    check();
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Init launch tables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = launch_init()
global s
%dbstop at 77 in isr_tfs_launch.m
% Clear Variables
s.curNcars=0; s.rcp=zeros(28,28,2);

% Cover all roads to make launch table
count=0;
vec=randsample(s.rn,s.rn);
for r=1:size(vec,1)
    road=vec(r);
    % If road is a entry road
    if(s.ri(road,8)==1)
        s.rl=[s.rl [road;0;(count)*s.tlaunchr;0;s.tlaunch;0]];
        count=count+1;
    end
end

% Set Particular entrance flux's
for i=1:size(s.rl,2)
    road=s.rl(1,i);
    s.rl(5,i)=s.in_flux(road);
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Checks the appearance of new cars
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = check()
global s

%dbstop at 126 in isr_tfs_launch.m
%dbstop at 163 in isr_tfs_launch.m

% If is necessary launch cars in simulator
if(s.curNcars < s.ncars)
    
    % Check the road launch table accumulators (if exists)
    if(sum(s.rl(2,:))~=0)
        
        % Check road launch table
        for r=1:size(s.rl,2)
            
            % If is necessary launch cars in simulator
            if(s.rl(2,r)>0)
                
                % If is necessary launch cars in simulator
                if(s.curNcars < s.ncars)
                    
                    % Read road number
                    road=s.rl(1,r);
                    
                    % Check if exist space to launch car in simulator
                    if(check_launch_isr_trafsim(road,(s.distlaunch/2)))
                        
                        % Initial velocity of car
                        launchvel=0;
                        
                        % Put road flag ON
                        s.rl(6,r)=1;
                        
                        % Increase cars launched
                        s.curNcars=s.curNcars+1;
                        
                        % Launch car
                        id=launch_car_isr_trafsim(road,launchvel);
                        
                        % Store the ID of last car launched in this road
                        s.rl(4,r)=id;
                        
                        % Decrease the acumulator
                        s.rl(2,r)=s.rl(2,r)-1;
                        %disp('Lança do acumulador')
                    end
                end
            end
        end
    end
    
    % Check road launch table
    for r=1:size(s.rl,2)
        
        % Check time
        if(round2(s.rl(3,r),0.01)==round2(s.time,0.01))
            
            % If is necessary launch cars in simulator
            if(s.curNcars < s.ncars)
                
                % Read road number
                road=s.rl(1,r);
                
                if(s.rl(6,r)==0)
                    % Check if exist space to launch car in simulator
                    if(check_launch_isr_trafsim(road,s.distlaunch))
                        
                        % Initial velocity of car
                        launchvel=s.velmax1;
                        
                        % Increase cars launched
                        s.curNcars=s.curNcars+1;
                        
                        % Launch car
                        id=launch_car_isr_trafsim(road,launchvel);
                        
                        % Store the ID of last car launched in this road
                        s.rl(4,r)=id;
                        % disp('lança')
                    else
                        %disp('Acumula')
                        % Increase the acumulator
                        s.rl(2,r)=s.rl(2,r)+1;
                    end
                    
                    % Schedule next launch in this road
                    s.rl(3,r)=s.rl(3,r)+s.rl(5,r);
                    
                else
                    %disp('Acumula flag')
                    % Increase the acumulator
                    s.rl(2,r)=s.rl(2,r)+1;
                    
                    % Schedule next launch in this road
                    s.rl(3,r)=s.rl(3,r)+s.rl(5,r);
                end
            end
        end
    end
end

% Clear all flags
s.rl(6,1:end)=0;

end



