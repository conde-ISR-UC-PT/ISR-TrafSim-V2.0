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
% Compute lateral errors
%
%-USE----------------------------------------------------------------------
%
%  isr_tfs_lateral_errors(n)
% -> Input(s)
%   » n         -   Position of vehicle in the vehicles structure
%
% -> Output(s)
%   » Array with [rl,rang,Rl,Rang]
%       (rl,rang)   -   Lateral and angular error for mass center
%       (Rl,Rang)   -   Lateral and angular error for look ahead distance
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

function [rl,rang,Rl,Rang] = lateral_errors_isr_trafsim( n )
global s c

% Default errors
rl=NaN; rang=NaN;   Rl=NaN; Rang=NaN;

if(c.car(n).pos.x>0 && c.car(n).pos.x<150 && c.car(n).pos.y>0 && c.car(n).pos.y<100)
    
    % Read car position and data
    x=c.car(n).pos.x;
    y=c.car(n).pos.y;
    t=c.car(n).pos.t;
    
    % Local trajectory
    min=c.car(n).setp-10;    if(min<1) min=1;end
    max=c.car(n).setp+40;   if(max>c.car(n).trajn) max=c.car(n).trajn; end
    
    % Line perpendicular to the motion
    p0x=x+10*cos(t+pi/2);    p1x=x+10*cos(t-pi/2);    p0y=y+10*sin(t+pi/2);    p1y=y+10*sin(t-pi/2);
    
    for i=min:(max-1)
        res=lineline_isr_trafsim(p0x,p0y,p1x,p1y,c.car(n).traj(1,i),c.car(n).traj(2,i),c.car(n).traj(1,i+1),c.car(n).traj(2,i+1));
        if(res(1)==1)
            ang=atan2( (c.car(n).traj(2,i+1)-c.car(n).traj(2,i)) , (c.car(n).traj(1,i+1)-c.car(n).traj(1,i)) );
            % Calculate lateral distance error and orientation error
            rl=sqrt( (x-res(2))^2 + (y-res(3))^2 );
            rang=t-ang;
            break
        end
    end
    
    % Point ahead
    px=x+s.dla*cos(t);  py=y+s.dla*sin(t);
    
    % Line perpendicular to the point ahead
    p0x=px+50*cos(t+pi/2);   p1x=px+50*cos(t-pi/2);   p0y=py+50*sin(t+pi/2);   p1y=py+50*sin(t-pi/2);
    
    for i=min:(max-1)
        res=lineline_isr_trafsim(p0x,p0y,p1x,p1y,c.car(n).traj(1,i),c.car(n).traj(2,i),c.car(n).traj(1,i+1),c.car(n).traj(2,i+1));
        if(res(1)==1)
            ang=atan2( (c.car(n).traj(2,i+1)-c.car(n).traj(2,i)) , (c.car(n).traj(1,i+1)-c.car(n).traj(1,i)) );
            
            % Calculate lateral distance error and orientation error
            Rl=sqrt( (px-res(2))^2 + (py-res(3))^2 );
            Rang=t-ang;
            break
        end
    end
end
end
