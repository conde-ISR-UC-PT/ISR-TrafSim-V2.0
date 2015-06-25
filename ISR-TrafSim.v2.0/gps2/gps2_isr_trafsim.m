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
% Generate GPS position based on a random error with variable amplitude
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

function [  ] = gps2_isr_trafsim( n )
global c s

c.car(n).gps_update=1;                      % Used in data fusion

% To store receivers positions
rec_pos=zeros(3,2);

% Separate the input arguments
x=c.car(n).pos.x;   y=c.car(n).pos.y;   t=c.car(n).pos.t;   time=s.time;
type=c.car(n).gps_type;    nrecep=c.car(n).gps_nrec;

% Space reservation to store coordinates given by GPS
out=zeros(nrecep,2);

% Number of receivers
if(nrecep==1)
    % One receiver -> Position of reicever is car mass center
    rec_pos(1,1)=x;  rec_pos(1,2)=y;
    
elseif(nrecep==2)
    % Two receivers -> Read from struct the relative positions of receivers and calculates the global position of the
    rel_pos1=gps_data.rel1;
    rel_pos2=gps_data.rel2;
    
    % Calculates the position of receivers in the world
    [rec_pos(1,1), rec_pos(1,2)]=cal_global_pos(x,y,t,rel_pos1);
    [rec_pos(2,1), rec_pos(2,2)]=cal_global_pos(x,y,t,rel_pos2);
    
    
elseif(nrecep==3)
    % Tree receivers -> Read from struct the relative positions of receivers and calculates the global position of the
    rel_pos1=gps_data.rel1;
    rel_pos2=gps_data.rel2;
    rel_pos3=gps_data.rel3;
    
    % Calculates the position of receivers in the world
    [rec_pos(1,1), rec_pos(1,2)]=cal_global_pos(x,y,t,rel_pos1);
    [rec_pos(2,1), rec_pos(2,2)]=cal_global_pos(x,y,t,rel_pos2);
    [rec_pos(3,1), rec_pos(3,2)]=cal_global_pos(x,y,t,rel_pos3);
end

% Call the GPS for each receptor
for i=1:nrecep
    rx=rec_pos(i,1);
    ry=rec_pos(i,2);
    if(type==1)
        [out(i,1),out(i,2),out(i,3),~,~,~]=gps_1(rx,ry);
    elseif(type==2)
        [out(i,1),out(i,2),out(i,3),~,~,~]=gps_2(rx,ry);
    elseif(type==3)
        [out(i,1),out(i,2),out(i,3),~,~,~]=gps_3(rx,ry);
    end
end

% Store position estimated position
c.car(n).gps=[out(i,1) out(i,2) out(i,3)];

% Increment counters
if(c.car(n).gps_time==c.car(n).gps_time_prev && c.car(n).gps_type==3)
    c.car(n).gps_time_count=c.car(n).gps_time_count+1;
elseif(c.car(n).gps_type==3)
    c.car(n).gps_time_prev=c.car(n).gps_time;
    c.car(n).gps_time_count=0;
end
c.car(n).gps_data(end+1,:)=[out(i,1) out(i,2)];
end

function [gpsx,gpsy,hdop,sat_pos,sat_num,usrxyz] = gps_1(x,y)

if(rand(1)>0.5)
    signal=1;
else
    signal=-1;
end
err=3+signal*rand(1)*2.5;
if(rand(1)>0.5)
    signal=1;
else
    signal=-1;
end
errx=signal*sqrt((err^2)/2);
if(rand(1)>0.5)
    signal=1;
else
    signal=-1;
end
erry=signal*sqrt((err^2)/2);

gpsx=x+errx;
gpsy=y+erry;
hdop=0; sat_pos=0; sat_num=0; usrxyz=0;

end

function [gpsx,gpsy,hdop,sat_pos,sat_num,usrxyz] = gps_2(x,y)

if(rand(1)>0.5)
    signal=1;
else
    signal=-1;
end
err=2.5+signal*rand(1)*2;
if(rand(1)>0.5)
    signal=1;
else
    signal=-1;
end
errx=signal*sqrt((err^2)/2);
if(rand(1)>0.5)
    signal=1;
else
    signal=-1;
end
erry=signal*sqrt((err^2)/2);

gpsx=x+errx;
gpsy=y+erry;
hdop=0; sat_pos=0; sat_num=0; usrxyz=0;

end

function [gpsx,gpsy,hdop,sat_pos,sat_num,usrxyz] = gps_3(x,y)

if(rand(1)>0.5)
    signal=1;
else
    signal=-1;
end
err=0.02+signal*rand(1)*0.015;
if(rand(1)>0.5)
    signal=1;
else
    signal=-1;
end
errx=signal*sqrt((err^2)/2);
if(rand(1)>0.5)
    signal=1;
else
    signal=-1;
end
erry=signal*sqrt((err^2)/2);

gpsx=x+errx;
gpsy=y+erry;
hdop=0; sat_pos=0; sat_num=0; usrxyz=0;

end