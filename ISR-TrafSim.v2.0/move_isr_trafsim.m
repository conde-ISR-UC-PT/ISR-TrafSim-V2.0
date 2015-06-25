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
% Move the cars in simulator and perform individual tasks for each vehicle
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

function [] = move_isr_trafsim()
global c s

ncars=length(c.listactive);
for n=1:ncars
    
    % Refresh previous positions/dynamic
    c.car(n).posprv.x=c.car(n).pos.x; c.car(n).posprv.y=c.car(n).pos.y; c.car(n).posprv.t=c.car(n).pos.t;
    c.car(n).prvdyn=c.car(n).dyn;
    
    % Read car current position
    x=c.car(n).pos.x; y=c.car(n).pos.y; t=c.car(n).pos.t;
    
    % Read current setpoint position
    rx=c.car(n).traj(1,c.car(n).setp); ry=c.car(n).traj(2,c.car(n).setp);
    
    % Calculate de distance and the angle between the robot and the destiny point
    [dist,dif_ang]=relativepos_isr_trafsim(x,y,t,rx,ry);
    
    % Passes to next set point
    c.car(n).prevsetp=c.car(n).setp;
    while(dist<s.dlockaheadgain*(c.car(n).dri.velmax*s.step_time))
        c.car(n).setp=c.car(n).setp+1;
        c.car(n).roadprev=c.car(n).road;				% Current Road
        c.car(n).road=c.car(n).traj(3,c.car(n).setp);	% Previous Road
        if(c.car(n).setp>=c.car(n).trajn)
            c.car(n).setp=c.car(n).trajn;
            break
        end
        if(c.car(n).setp<=c.car(n).trajn)
            rx=c.car(n).traj(1,c.car(n).setp); ry=c.car(n).traj(2,c.car(n).setp);
            [dist,dif_ang]=relativepos_isr_trafsim(x,y,t,rx,ry);
        else
            break
        end
    end
    
    
    % Print the current setpoint
    if(s.mode==1 && s.printcurrsp==1)
        if(c.car(n).hsetp==-1)
            c.car(n).hsetp=plot(rx,ry,'w*');
        else
            delete(c.car(n).hsetp)
            c.car(n).hsetp=plot(rx,ry,'w*');
        end
    end
    
    % Check if car need turn right/left
    c.car(n).cmdste=turn(dif_ang,dist);
    
    % Kinematic
    [c.car(n).pos.x, c.car(n).pos.y, c.car(n).pos.t, c.car(n).dyn(1), c.car(n).dyn(2), c.car(n).pwr c.car(n).pwf]=dynamic_isr_trafsim(s.step_time,x,y,t,c.car(n).cmdacc,c.car(n).cmdste,c.car(n).dyn(1),c.car(n).dyn(2),c.car(n).dri.velmax,n,c.car(n).traj(3,c.car(n).setp),c.car(n).setp,c.car(n).Wsbw,c.car(n).length);
    c.car(n).TRUEpos(end+1,:)=[c.car(n).pos.x c.car(n).pos.y c.car(n).pos.t c.car(n).dyn(1) c.car(n).dyn(2) c.car(n).cmdacc c.car(n).cmdste dist_isr_trafsim(c.car(n).posprv.x, c.car(n).posprv.y,c.car(n).pos.x,c.car(n).pos.y)];     % Store car trajectory
    
    
    % Calculates the positions of the corners
    [c.car(n).cornX c.car(n).cornY]=pcorners_isr_trafsim(c.car(n).pos.t,c.car(n).length,c.car(n).width);
    
    % Get position by Odometry
    if(s.odo>0)
        if(mod(round2(c.car(n).timeinsim,0.0001),(1/s.frqodo))==0)
            c.car(n).Pwr=c.car(n).Pwr+c.car(n).pwr; c.car(n).Pwf=c.car(n).Pwf+c.car(n).pwf;
            c.car(n).odo=odometry_isr_trafsim(c.car(n).odo(1),c.car(n).odo(2),c.car(n).odo(3),c.car(n).Pwr,c.car(n).Pwf,c.car(n).steerval,c.car(n).type,c.car(n).length);
            c.car(n).Pwr=0; c.car(n).Pwf=0;
        else
            c.car(n).Pwr=c.car(n).Pwr+c.car(n).pwr;
            c.car(n).Pwf=c.car(n).Pwf+c.car(n).pwf;
        end
    end
    
    % Compute lateral errors
    if(s.lateral_errors)
        [err1,err2,err3,err4]=lateral_errors_isr_trafsim(n);
        c.car(n).traj_err=[c.car(n).traj_err  [err1;err2;err3;err4] ];
    end
    
    % Get position by GPS
    if(s.gps>0)
        if(s.gps_type==1)
            gps_isr_trafsim(n);
        elseif(s.gps_type==2)
            gps2_isr_trafsim(n);
        end
    end
    
    % Get position by INS
    if(s.ins>0 && mod(round2(c.car(n).timeinsim,0.0001),(1/s.frqins))==0)
        c.car(n).ins=ins_isr_trafsim( [c.car(n).posprv.x  c.car(n).posprv.y  c.car(n).posprv.t c.car(n).pos.x  c.car(n).pos.y  c.car(n).pos.t c.car(n).dyn(1) c.car(n).prvdyn(1) c.car(n).ins n]);
    end
    
    % Get position by Magnets (out=[distanceInRule MagnetNumber ])
    if(s.mag==1 && mod(round2(c.car(n).timeinsim,0.0001),(1/s.fmag))==0 && c.car(n).mag_rule==1)
        out=magnets_isr_trafsim( [c.car(n).pos.x c.car(n).pos.y c.car(n).pos.t n 0] );
        
        if(~isnan(out(1)))
            c.car(n).mag_detect=[out(2) out(1)];
            %                 c.car(n).mag=mag_pos(out(1),out(2),s.ri(c.car(n).traj(3,c.car(n).setp),7),n);
            c.car(n).mag=mag_pos(out(1),out(2),c.car(n).pos.t,n);
            c.car(n).mag_error=[abs(c.car(n).mag(1)-c.car(n).pos.x) abs(c.car(n).mag(2)-c.car(n).pos.y)];
            
            % Inject position give by magnets on INS estimated position
            if(s.ins>0)
                c.car(n).ins(1)=c.car(n).mag(1);
                c.car(n).ins(2)=c.car(n).mag(2);
            end
            % Reset Sensorial fusion data
            c.car(n).fus=zeros(1,3)*NaN;
            c.car(n).ins_count=0;c.car(n).fusion_init=0;c.car(n).X_pre=0; c.car(n).X_est=0;c.car(n).K=0;c.car(n).P_pre_KF=0;%P_est_KF=0;G=0; W=0;R=0;gps_update=0;
        end
    end
    
    % Fusion
    if(s.fus>0 && mod(round2(c.car(n).timeinsim,0.0001),(1/s.frqfus))==0 && s.time~=0)
        c.car(n).fus=fusion_isr_trafsim(n);
        %disp('FUSION')
    end
    
    % Fusion 3
    if(s.fus3>0 && mod(round2(c.car(n).timeinsim,0.0001),(1/s.frqfus3))==0 && s.time~=0)
        fusion3_isr_trafsim('process',n);
        %disp('FUSION3')
    end
    
    % Fusion 4
    if(s.fus4 && c.car(n).timeinsim>0 && mod(round2(c.car(n).timeinsim,0.0001),(1/s.frqfus4))==0)
        
        % Refresh noise vectors if is necessary
        if(s.time>c.car(n).nv_time-s.step_time)
            fusion4_isr_trafsim('refresh_noise',n);
        end
        
        % Detects slip and modifies flag "slip_state"
        fusion4_isr_trafsim('detect_slip',n);
        
        % Process position correction with inovation
        c.car(n).fus4prev=c.car(n).fus4;
        [c.car(n).fus4]=fusion4_isr_trafsim('process',n);
        
        % Pass to next position in noise vector
        c.car(n).nv_setp=c.car(n).nv_setp+1;
    end
    % Simulate communication procedure for a car
    rxtx_module_isr_trafsim(n);
    
    % Fusion 2
    if(s.fus2>0 && mod(round2(c.car(n).timeinsim,0.0001),(1/s.frqfus2))==0 && s.time~=0)
        fusion2_isr_trafsim(n);
        %            disp('FUSION2')
    end
    
    % Perform data acquisition
    if(s.disp_res)
        if(mod(round2(c.car(n).timeinsim,0.0001),(1/s.fres))==0)
            adq_data(n);
        end
    end
    
    % Refresh time in simulator
    c.car(n).timeinsim=c.car(n).timeinsim+s.step_time;
    
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Data acquisition
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function adq_data(n)
global s c

c.car(n).data= [c.car(n).data [s.time ; c.car(n).timeinsim; ...
    abs(c.car(n).gps(1)-c.car(n).pos.x) ; abs(c.car(n).gps(2)-c.car(n).pos.y); ...                                        % GPS error in X-axis and Y-axis
    c.car(n).mag_error(1) ; c.car(n).mag_error(2); ...                                                                    % Magnets error in X-axis and Y-axis
    abs(c.car(n).ins(1)-c.car(n).pos.x) ; abs(c.car(n).ins(2)-c.car(n).pos.y) ; abs(c.car(n).ins(3)-c.car(n).pos.t); ...  % INS error in X-axis and Y-axis e orientation angle
    abs(c.car(n).fus(1)-c.car(n).pos.x) ; abs(c.car(n).fus(2)-c.car(n).pos.y) ; abs(c.car(n).fus(3)-c.car(n).pos.t); ...  % Fusion error in X-axis and Y-axis e orientation angle
    abs(c.car(n).odo(1)-c.car(n).pos.x) ; abs(c.car(n).odo(2)-c.car(n).pos.y) ; abs(c.car(n).odo(3)-c.car(n).pos.t); ...  % Odometry error in X-axis and Y-axis e orientation angle
    min(c.car(n).laserdist); ...                                                                                          % Minimum distance returnded by laser
    c.car(n).dyn(1); ...                                                                                                  % Car Linear velocity
    c.car(n).traj(3,c.car(n).setp), ...                                                                                   % Current Road
    ]
    ];

c.car(n).mag_error=[NaN NaN];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate car position given by magnets
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [pos] = mag_pos(dist,mag,t,n)
global s c mag_data
%     dbstop at 233 in move
% Calculate the half length of car and the rule length
halflen=c.car(n).length/2;
if(s.rule_length>c.car(n).width)
    rule_length=c.car(n).width;
else
    rule_length=s.rule_length;
end

% Point were is permanent magnet
px=mag_data(mag,1);
py=mag_data(mag,2);

% Distance of magnet to rule center of rule
d=(rule_length/2)-dist;

% Point of the center of rule
xc=px-d*cos(t+pi/2);
yc=py-d*sin(t+pi/2);

% Point of car mass center
pos(1)=xc-halflen*cos(t);
pos(2)=yc-halflen*sin(t);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Turn algoritm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [cmdste] = turn(dif,dist)
global  s

dif = -dif;
laterr= dist * sin(dif);
steer =inference(100+laterr,90+dif*180/pi,0,1)*5;
steer=steer * pi/180;
steer= min(steer,s.dynmaxste(1,2));
steer= max(steer,-s.dynmaxste(1,2));
cmdste=steer/s.dynmaxste(1,2);

end