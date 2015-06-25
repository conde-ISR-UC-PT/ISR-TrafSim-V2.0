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
%%-DESCRIPTION--------------------------------------------------------------
%
% Simulate driver behavior and manage cross traffic gestion - Early Method
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

function [] = cross_gest_isr_trafsim()
global s c
% s.cross_notify_list -> [id ; node ; state; time; posx; posy; inroad; outroad; currentVel; maxVel; carType, CurrentRoad , timeoflastnotify]

% Delete layers if are no longer necessary
delete_layer_isr_trafsim();

% Add car to s.cross_notify_list if car is not in there (no commu)
add_car_notify_cross_isr_trafsim();

% Delete car withot V2I from list s.rg_count_tab2 & s.list_cross_already_reserve
if(s.time>s.time_cross_without_commu && s.time_cross_without_commu~=-1)
    
    id=s.car_cross_without_commu;
    s.car_cross_without_commu=[];
    s.time_cross_without_commu=-1;
    for i=1:size(s.rg_count_tab2,2)
        for p=3:10
            if(isnan(s.rg_count_tab2(p,i)))
                s.rg_count_tab2(p,i)=0;
                s.rg_count_tab2(2,i)=s.rg_count_tab2(2,i)-1;
                break
            end
        end
    end
    for i=2:size(s.list_cross_already_reserve,1)
        if(~isempty(find(s.list_cross_already_reserve(i,:)==id)))
            s.list_cross_already_reserve(i,:)=[];
            break
        end
    end
    if(ismember(id,c.listactive))
        pos=find(c.listactive==id);
        road=find(s.rg_count_tab2(1,:)==c.car(pos).rac);
        for p=3:10
            if(s.rg_count_tab2(p,road)==0)
                s.rg_count_tab2(p,road)=id;
                s.rg_count_tab2(2,road)=s.rg_count_tab2(2,road)+1;
                break
            end
        end
    end
end

% Process Algoritm
Exist_Car_withot_V2I=~isempty(s.list_cross_without_commu);  % Doesn't exist car without communication on the list
Exist_Notification=size(s.cross_notify_list,2)>1;        % Exist notifications

if(s.pause==1)
    s.pause=0;  % to pause only once
    keyboard
end



if(Exist_Notification && ~Exist_Car_withot_V2I)
    aux=2;
    while(aux<=size(s.cross_notify_list,2))
        cond1=s.cross_notify_list(3,aux)==-1;
        cond2=s.time>s.cross_notify_list(4,aux)+2;
        cond3=(dist_isr_trafsim(s.cross_center(1),s.cross_center(2),s.cross_notify_list(5,aux),s.cross_notify_list(6,aux)))<s.control_radius_cross;
        if(cond1 && cond2 && cond3)
            id=s.cross_notify_list(1,aux);                  % Car Id
            n=find(c.listactive==id);                       % Car place on c.listactive
            CarCommu=~isnan(s.cross_notify_list(9,aux));    % If car have communications
            if(isempty(id)||isempty(n))
                keyboard
            end
            if(CarCommu)
                % Generate trajectory
                traj=gen_traj_isr_trafsim(s.cross_notify_list(7,aux),s.cross_notify_list(8,aux));
                
                % View road after crossroads
                rafter=check_road_after_cross_isr_trafsim(traj);
                
                vel_prof=reserve_space_isr_trafsim(round2(s.time,0.001),s.cross_notify_list(5,aux),s.cross_notify_list(6,aux),s.cross_notify_list(9,aux),s.cross_notify_list(10,aux),s.cross_notify_list(11,aux),traj,rafter,n,'cross',s.cross_method);
                
                % Order to send velocity profile
                if(~isnan(vel_prof))
                    send_velocity_profile_isr_trafsim(n,aux,id,vel_prof,rafter,'cross');
                    if(vel_prof(1,end)>s.cross_last_time_used)  % Store last time used by round gest mode
                        s.cross_last_time_used=vel_prof(1,end);
                    end
                    %viewM(s.ocmap2,s.layer,0,s.occup_res2)
                    break
                end
            else
                exist_exit_full=0;
                if(s.cross_notify_list(12,aux)==15 || s.cross_notify_list(12,aux)==16)
                    for i=3:size(s.rg_count_tab2,2)
                        if(s.rg_count_tab2(2,i)>s.rg_car_limit2)
                            exist_exit_full=1;
                        end
                    end
                else
                    for i=1:size(s.rg_count_tab2,2)
                        if(s.rg_count_tab2(2,i)>s.rg_car_limit2)
                            exist_exit_full=1;
                        end
                    end
                end
                previous_withouV2I_exit=s.time_cross_without_commu==-1;
                % To dont send cars when one of exits is already full
                if(~exist_exit_full && previous_withouV2I_exit)
                    
                    % Reserve space
                    [M,L]=reserve_all_paths(s.cross_notify_list(12,aux),n,'wait','cross');
                    
                    if(s.time>s.cross_last_time_used)
                        last=s.time;
                    else
                        last=s.cross_last_time_used;
                    end
                    
                    % Add generated matrix in correct space
                    for i=1:size(M,3)
                        current_layer=round2(last+L(i),0.01);  % Layer to write
                        
                        % Add layer if is necessary
                        exist_current_layer=find(s.layer==current_layer);
                        if(isempty(exist_current_layer))
                            create_layer(round2(current_layer,0.01));
                        end
                        
                        % Write layer in correct position and verify if everything is ok
                        layer_pos=find(s.layer==current_layer);
                        if(nnz(s.ocmap2(:,:,layer_pos)&M(:,:,i))>0)
                            keyboard
                        end
                        s.ocmap2(:,:,layer_pos)=s.ocmap2(:,:,layer_pos)+M(:,:,i);
                    end
                    
                    s.list_cross_without_commu(end+1,:)=[id last+L(1) last+L(end) s.cross_notify_list(12,aux)];
                    s.cross_last_time_used=last+L(end);
                    
                    
                    % Agend time to reserve new cars
                    s.time_cross_without_commu=s.list_cross_without_commu(3);
                    s.car_cross_without_commu=s.list_cross_without_commu(1);
                    %s.list_cross_already_reserve(end+1,:)=NaN;
                    for i=1:size(s.rg_count_tab2,2)
                        for p=3:10
                            if(s.rg_count_tab2(p,i)==0)
                                s.rg_count_tab2(p,i)=NaN;
                                break
                            end
                        end
                    end
                    s.rg_count_tab2(2,:)=s.rg_count_tab2(2,:)+1;
                    
                    % To view the map (help on debug)
                    %                         viewM(s.ocmap2,s.layer,0,s.occup_res2,n,'cross')
                    
                    % Delete notification
                    s.cross_notify_list(:,aux)=[];
                    
                    % Driver ignores traffic rules
                    c.car(n).dri_ignore_trf_cross=1;
                    
                    %disp(['Ignore Cross: ' num2str(id)]);
                    break
                end
            end
        end
        aux=aux+1;
    end
elseif(Exist_Car_withot_V2I)
    
    if(s.time>s.list_cross_without_commu(2)) % s.time>s.list_cross_without_commu(2) to
        % Give permission to car to advance
        n=find(c.listactive==s.list_cross_without_commu(1));
        c.car(n).nostop(end+1)=(s.list_cross_without_commu(4));
        
        
        % Add car to list already reserved
        if(isempty(find(s.list_cross_already_reserve==s.list_cross_without_commu(1))))
            [ll cc]=find(s.list_cross_already_reserve(1,:)==s.list_cross_without_commu(4));
            s.list_cross_already_reserve(end+1,cc)=s.list_cross_without_commu(1);
        end
        
        % Delete car from list
        [ll cc]=find(s.list_cross_without_commu==s.list_cross_without_commu(1));
        s.list_cross_without_commu(ll,:)=[];
    end
end
end
