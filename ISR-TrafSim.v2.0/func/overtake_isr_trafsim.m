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
%-DESCRIPTION--------------------------------------------------------------
%
% Simulate the lane change when two cars are stopped in a semaphore
%
%-DISCLAIMER---------------------------------------------------------------
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

function [  ] = overtake_isr_trafsim()
global c s

for n=1:size(c.listactive,2)    % Compute to all cars in simulator
    
    % Verify if exist conditions to execute the lane change manouver
    change=0;                   % Don't exist condicions to make manouver
    if(c.car(n).dyn(1)<5/3.6)       % If car is sttoped (< 5 km/h)
        
        % Extract car stats
        road=c.car(n).road;
        dest=c.car(n).roaddes;
        
        if(road<30)
            impar=mod(road,2);
            if(impar), road_side=road+1; else road_side=road-1; end
            impar=mod(dest,2);
            if(impar), road_dest_side=dest+1; else road_dest_side=dest-1; end
            
            xpos=c.car(n).pos.x;
            ypos=c.car(n).pos.y;
            setp_xy=[s.r(road,1:s.ri(road,5),1);s.r(road,1:s.ri(road,5),2)];    % Setpoints [3:road-1]
            roadsetp=size(setp_xy,2);
            
            % Calculate distance to all road setpoints
            dist_xy=zeros(1,roadsetp);
            for i=1:roadsetp
                dist_xy(1,i)=dist_isr_trafsim(setp_xy(1,i),setp_xy(2,i),xpos,ypos);
            end
            [~,setp]=min(dist_xy);
            
            cond1=road<30;                                      % If car is not in roundabout
            cond2=(setp>3 && setp<roadsetp-5);                  % Setpoint selectd must be >1 and <roadsetp
            cond3=s.ri(road,8)==1;                              % If is a entrance road
            cond4=(s.rp(road_side,road_dest_side)>0 || s.rp(road_side,dest)>0);             % If exist a avaliable path
            cond5=isempty(c.car(n).vel_prof);                   % Driver have the control
            
            
            if(cond1 && cond2 && cond3 && cond4 && cond5)
                cond6=check_car_free([0 2.5],n);                          % Check space in front/back of car
                cond7=check_road_free_int(road_side,setp,[15 10],n);    % If exist 10 meters free in front and back of car in road_side
                if(cond6 && cond7)
                    change=1;     % Can change lane
                end
            end
        end
    end
    
    % Execute the change lane manouver
    if(change)        % If exist conditions to change lane
        % Extract current trajectory
        actualTraj=c.car(n).traj(:,1:c.car(n).setp);
        
        % Extract new trajectory
        if(s.rp(road_side,road_dest_side)>0)
            [newTraj path ~]=isr_tfs_gen_traj(road_side,road_dest_side);
        elseif(s.rp(road_side,dest)>0)
            [newTraj path ~]=isr_tfs_gen_traj(road_side,dest);
        end
        
        % Get Join point of new trajectory
        pointY=actualTraj(2,end);
        pointxI=actualTraj(1,end)+50*cos(actualTraj(3,end)-pi/2);
        pointxF=actualTraj(1,end)+50*cos(actualTraj(3,end)+pi/2);
        [~, col_] =size(newTraj);
        for i=1:col_-1
            out=isr_tfs_direc_inter3(pointxI,pointY,pointxF,pointY,newTraj(1,i),newTraj(2,i),newTraj(1,i+1),newTraj(2,i+1));
            if(out(1)==1)
                break;
            end
        end
        
        d=zeros(1,size(newTraj,2));
        d_=zeros(1,size(c.car(n).traj,2));
        for i=1:size(newTraj,2)
            d(i)=dist_isr_trafsim(c.car(n).pos.x,c.car(n).pos.y,newTraj(1,i),newTraj(2,i));
        end
        for i=1:size(c.car(n).traj,2)
            d_(i)=dist_isr_trafsim(c.car(n).pos.x,c.car(n).pos.y,c.car(n).traj(1,i),c.car(n).traj(2,i));
        end
        [~,setp2]=min(d);
        [~,setp1]=min(d_);
            
        % Generate new join trajectory
        [c.car(n).traj,c.car(n).path,c.car(n).trajn]=traj_join(c.car(n).traj,setp1,newTraj,setp2,path);
        c.car(n).road=c.car(n).traj(3,1);
        c.car(n).crosscels=view_cross_cels(n);  % Refresh cross cels
        
        dist=zeros(1,size(c.car(n).traj,2));
        for i=1:size(c.car(n).traj,2)
            dist(1,i)=dist_isr_trafsim(c.car(n).pos.x,c.car(n).pos.y,c.car(n).traj(1,i),c.car(n).traj(2,i));
        end
        [~,col]=min(dist);
        c.car(n).setp=col+3;
        
        % Find the best point from trajectory
        % Show final trajectory
        if(s.printchangelane && s.mode==1)
            haux=plot(c.car(n).traj(1,:),c.car(n).traj(2,:),'r');
            pause(0.3)
            delete(haux);
            pause(0.2)
            haux=plot(c.car(n).traj(1,:),c.car(n).traj(2,:),'r');
            pause(0.3)
            delete(haux);
            pause(0.2)
            haux=plot(c.car(n).traj(1,:),c.car(n).traj(2,:),'g');
            pause(2)
            delete(haux);
            pause(0.2)
        end
        s.overtake_n=s.overtake_n+1;
        
        
        
    end
end

end

