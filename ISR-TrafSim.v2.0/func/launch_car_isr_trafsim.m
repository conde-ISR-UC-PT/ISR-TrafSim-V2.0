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
% Launch car in selected road
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

function [id,n] = launch_car_isr_trafsim(r,~)
global s c gps_data

n=datacar_isr_trafsim('addc');

% Verifies and updates the destination tables
dest=check_destination(r);
%dest=23;

% Car and driver setup
id=c.car(n).id;
c.car(n).cmdacc=1;                                      % Trottle
c.car(n).road=r;                                        % Current road
c.car(n).roadori=r;                                     % Origin
c.car(n).roaddes=dest;                                  % Destination
c.car(n).auto=0;                                        % Car id not a autonomous vehicle
c.car(n).type=round(rand(1)+1);                         % Type (1-car, 2-Truck)
c.car(n).setp=1;                                        % Current setpoint
c.car(n).dri.type=round(rand(1)*2+1);                   % Driver type
%deltaspeed=(s.nspeed-(2*s.nspeed)*rand(1))/3.6;         % Noise speed that will be added to the maximum speed in m/s

if(s.cars_same)
    c.car(n).dri.type=2;
    c.car(n).type=1;
end
deltaspeed=0;

if(c.car(n).dri.type==1)
    c.car(n).dri.color=s.cd1;                       % Color of car
    c.car(n).dri.velmax=s.velmax1+deltaspeed;       % Max. speed
elseif(c.car(n).dri.type==2)
    c.car(n).dri.color=s.cd2;
    c.car(n).dri.velmax=s.velmax2+deltaspeed;
elseif(c.car(n).dri.type==3)
    c.car(n).dri.color=s.cd3;
    c.car(n).dri.velmax=s.velmax3+deltaspeed;
end

% Initialize car measures
parameters=car_type_isr_trafsim(c.car(n).type);
para=fieldnames(parameters); npara=size(para,1);
ccarn=c.car(n);
for i=1:npara
    name=para(i);                           % Name of field
    value=getfield(parameters,char(name));  % Value returned by car_type function
    c.car(n).(char(name))=value;            % Assign value to structure
    %c.car(n).Wsbw=c.car(n).length-0.8;  % Space between wheels
end

% Init car dynamic properties
%steering_ratio = 2.8;           % Number of steering turns (Cremalheira, assist./2,8)
minimum_circle_diameter = 11;   % Diameter of turnning (m) 	11
max_stering=asin(c.car(n).length/(minimum_circle_diameter/2-c.car(n).width/2));     % Maximun steering
c.car(n).dynmaxacc=[5 -10];                     % [maxVelAcc   (m/s)    maxVelBrk (m/s) ]
c.car(n).dynmaxste=[180*pi/180 max_stering];    % [VelMaxSteer (rad/s)  MaxSteer (rad/s)]
c.car(n).max_stering_lock2lock=2*max_stering;
c.car(n).NStWteeth=4096;                        % Steering theet's
c.car(n).NWteeth=200;
c.car(n).WheelD=s.wheel_diam;                   % Wheel diameter
c.car(n).WheelR=s.wheel_radius;                 % Wheel radius
c.car(n).WRadii_RR = c.car(n).WheelR;
c.car(n).WRadii_RL = c.car(n).WheelR;
c.car(n).WRadii_FR = c.car(n).WheelR;
c.car(n).WRadii_FL = c.car(n).WheelR;

% Cars without communication V2I
current_rate=s.cars_rate_no_V2I(2)/s.curNcars;
c.car(n).commu=1;
if(current_rate<s.cars_rate_no_V2I(1))
    c.car(n).commu=0;
    c.car(n).dri.color=[0 0 1];
    s.cars_rate_no_V2I(2)=s.cars_rate_no_V2I(2)+1;
end

% Car current speed (m/s)
vel=c.car(n).dri.velmax;
c.car(n).dri.velcur=vel;
c.car(n).dyn(1)=vel;

% Car launch position
pos=launch_pos(r,n);
c.car(n).pos.x=pos(1);
c.car(n).pos.y=pos(2);
c.car(n).pos.t=pos(3);

% Security distance to the front car
c.car(n).time2stop=6+rand(1)*3;
c.car(n).dist2stop=7+rand(1)*3;
c.car(n).dist2light=2+rand(1)*2;

% Initial INS position
if(s.ins>0)
    c.car(n).ins=[c.car(n).pos.x c.car(n).pos.y c.car(n).pos.t];
end

% Initial Odometry position
if(s.odo>0)
    c.car(n).odo=[c.car(n).pos.x c.car(n).pos.y c.car(n).pos.t];
end

% Initial FUS position
if(s.fus>0)
    c.car(n).fus=[c.car(n).pos.x c.car(n).pos.y c.car(n).pos.t];
end

% Initial FUS parameters
if(s.fus3>0)
    fusion3_isr_trafsim('init_parameters',n);
end
% Init GPS parameters
if(s.gps>0 && s.gps_type==1)
    c.car(n).gps_type=gps_data.type;
    c.car(n).gps_native=gps_data.type;
    c.car(n).gps_f1=s.frqgps;
    c.car(n).gps_f2=s.frqrtkgps;
    c.car(n).gps_nrec=gps_data.nreceptors;
    c.car(n).gps_rtk=1;
elseif(s.gps>0 && s.gps_type==2)
    c.car(n).gps_type=3;
    c.car(n).gps_native=3;
    c.car(n).gps_f1=s.frqgps;
    c.car(n).gps_f2=s.frqrtkgps;
    c.car(n).gps_nrec=1;
    c.car(n).gps_rtk=1;
end

% Init magnets detection parameters
c.car(n).mag_rule=s.mag;

% For initialize noise vectors for fusion4 (INOV-CT)
if(s.fus4)
    fusion4_isr_trafsim('init_parameters',n);
    fusion4_isr_trafsim('init_noise',n);
end

% Generate trajectory setpoints
[c.car(n).traj c.car(n).path c.car(n).trajn]=gen_traj_isr_trafsim(c.car(n).roadori,c.car(n).roaddes);

% View road after exit roundabout
c.car(n).rar=check_road_after_roundabout_isr_trafsim(c.car(n).traj);

% View road after exit cross
c.car(n).rac=check_road_after_cross_isr_trafsim(c.car(n).traj);

if(~isempty(c.car(n).rac))
    c.car(n).rbc=check_road_before_cross_isr_trafsim(n);
    c.car(n).lightdirection=view_light(n);
end

% View cross cels intercepted by the trajectory
c.car(n).crosscels=view_cross_cels_isr_trafsim(n);

% Add car to OTA_commu list
if(s.otacommu==1 && s.otacommuemulation==1)
    ota_commu_isr_trafsim('add_node',id);
end

% To simulate previous movement
if(s.run_before==1)
    d=-5;
    pt=c.car(n).pos.t;
    px=c.car(n).pos.x+d*cos(pt);
    py=c.car(n).pos.y+d*sin(pt);
    c.car(n).ins=[px py pt];
    for j=d:s.step_time:-s.step_time
        nx=px+s.step_time*cos(c.car(n).pos.t);
        ny=py+s.step_time*sin(c.car(n).pos.t);
        nt=pt;
        % Get position by GPS
        if(s.gps>0)
            c.car(n).gps_update=1;                      % Used in data fusion
            c.car(n).gps=gps_isr_trafsim(n);
        end
        % Get position by INS
        if(s.ins>0)
            c.car(n).ins=ins_isr_trafsim( [px  py  pt nx  ny  nt 1 1 c.car(n).ins n]);
        end
        % Fusion
        if(s.fus>0)
            c.car(n).fus=fusion_isr_trafsim(n);
        end
        px=nx; py=ny;
    end
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define wich changing direction light the driver will turn on in the cross
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [light] = view_light(n,~)
global c s
incel= s.lighttable(:,1)==c.car(n).rbc;
outcel= s.lighttable(1,:)==c.car(n).rac;
light=s.lighttable(incel,outcel);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Return point of origin as the road to launch
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [pos] = launch_pos(r,~)
global s
%d=10;   % Distance from first setpoint
if(r==1)
    pos(1)=s.ri(r,1);
    pos(2)=s.ri(r,2);
    pos(3)=0;
elseif(r==2)
    pos(1)=s.ri(r,1);
    pos(2)=s.ri(r,2);
    pos(3)=0;
elseif(r==3)
    pos(1)=s.ri(r,1);
    pos(2)=s.ri(r,2);
    pos(3)=pi/2;
elseif(r==4)
    pos(1)=s.ri(r,1);
    pos(2)=s.ri(r,2);
    pos(3)=pi/2;
elseif(r==5)
    pos(1)=s.ri(r,1);
    pos(2)=s.ri(r,2);
    pos(3)=pi/2;
elseif(r==6)
    pos(1)=s.ri(r,1);
    pos(2)=s.ri(r,2);
    pos(3)=pi/2;
elseif(r==7)
    pos(1)=s.ri(r,1);
    pos(2)=s.ri(r,2);
    pos(3)=pi;
elseif(r==8)
    pos(1)=s.ri(r,1);
    pos(2)=s.ri(r,2);
    pos(3)=pi;
elseif(r==9)
    pos(1)=s.ri(r,1);
    pos(2)=s.ri(r,2);
    pos(3)=-pi/2;
elseif(r==10)
    pos(1)=s.ri(r,1);
    pos(2)=s.ri(r,2);
    pos(3)=-pi/2;
elseif(r==11)
    pos(1)=s.ri(r,1);
    pos(2)=s.ri(r,2);
    pos(3)=-pi/2;
elseif(r==12)
    pos(1)=s.ri(r,1);
    pos(2)=s.ri(r,2);
    pos(3)=-pi/2;
elseif(r==15)
    pos(1)=s.ri(r,1);
    pos(2)=s.ri(r,2);
    pos(3)=0;
elseif(r==16)
    pos(1)=s.ri(r,1);
    pos(2)=s.ri(r,2);
    pos(3)=0;
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Check launch tables and update them
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [dest] = check_destination(r)
global s

% Make table
c=0; aux=-1; dest=0;
for i=1:s.rn
    if(s.rp(r,i)~=-1)
        c=c+1;
        aux(1,c)=s.rp(r,i);          % Probability
        aux(2,c)=i;                  % Road
        aux(3,c)=s.rcp(r,i,1);       % Current probability
        aux(4,c)=s.rcp(r,i,2);       % Cars in
    end
end

% Check destination
[~, roads]=size(aux);
tab=randsample(1:roads,roads);
for i=1:roads
    j=tab(i);
    if( round2(aux(3,j),0.0001) <= round2(aux(1,j),0.0001))
        dest=aux(2,j);
        break
    end
end

% Updates table road current probability (s.rcp)
s.rcp(r,dest,2)=s.rcp(r,dest,2)+1;
tot=sum( s.rcp(r,:,2) );
for i=1:roads
    s.rcp(r,aux(2,i),1)=s.rcp(r,aux(2,i),2)/tot;
end

end

