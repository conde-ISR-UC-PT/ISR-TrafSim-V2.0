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
% Simulate driver behaviour and manage cross traffic lights according
% selected option (normal/intelligent)
%
%-USE----------------------------------------------------------------------
%
% -> Input(s)
%   » n         -   Position of vehicle on the structure
%
% -> Output(s)
%   » acccmd    -   Acceleration command
%   » cause     -   ID of stoping cause
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
%   Sérgio Santos*
%
%  *Institute of Systems and Robotics   - University of Coimbra
% **School of Technology and Management - Polytechnic Institute of Leiria
%--------------------------------------------------------------------------

function [cmdacc cause] = traffic_gestion_isr_trafsim(n)
global c s
cmdacc=1;
cause=0;
danger=0;


% Read current car road
road=c.car(n).traj(3,c.car(n).setp);
carid=c.car(n).id;

%    if(carid==104 && ~isempty(find(5==c.car(n).list)))
%         dbstop at 64 in isr_tfs_traffic_gestion.m
%         a=1;
%end
% Cross gestion
if(s.cross_mode==1)
    cross_gestion_advanced(n);
elseif(s.cross_mode==0)
    cross_gestion_normal(n);
end



% If the car is stopped
if(c.car(n).cmdacc==-999)
    cmdacc=-999;
else
    % If driver not see other cars
    if(isempty(c.car(n).list))
        %cmdacc=1;
    else                 % If driver see other cars
        for cc=1:length( c.car(n).list )
            %dbstop at 84 in isr_tfs_traffic_gestion.m
            % Car close
            ccid=c.car(n).list(cc);
            
            if(ccid<100)
                
                if( ccid==road && isempty(find(c.car(n).nostop==ccid)) )  % The number corresponds to the obstacle
                    
                    % Add new entry to the table
                    if(isempty(find(s.table_gest0(1,:)==carid)) && isempty(find(s.table_gest0(3,:)==road)) && isempty(intersect(ccid,s.crosslist))==0)
                        dd=check_dist_end_road_isr_trafsim(n,road);
                        auxx=[0 0 0];
                        if(dd<25)
                            for i=1:length(c.listactive)    % Search for a car that is still closest he is
                                if(n~=i)
                                    if(c.car(i).traj(3,c.car(i).setp)==road)
                                        ddd=check_dist_end_road_isr_trafsim(i,road);
                                        if(ddd<dd)
                                            auxx=[1 c.car(i).id c.car(i).traj(3,c.car(i).setp)];
                                            dd=ddd;
                                        end
                                    end
                                end
                            end
                            
                            if(auxx(1,1)==1 && isempty(find(s.table_gest0(1,:)==auxx(1,2))))
                                s.table_gest0=[s.table_gest0 [auxx(1,2) ; s.time ; auxx(1,3)] ];
                                %                                     if(auxx(1,3)==15 || auxx(1,3)==16)
                                %                                         col=find(s.rg_count_tab(1,:)==auxx(1,3));
                                %                                         s.rg_count_tab(2,col)=s.rg_count_tab(2,col)-1;
                                %                                     end
                            else
                                s.table_gest0=[s.table_gest0 [carid ; s.time ; road] ];
                                %                                     if(road==15 || road==16)
                                %                                         col=find(s.rg_count_tab(1,:)==road);   s.rg_count_tab(2,col)=s.rg_count_tab(2,col)-1;
                                %                                     end
                            end
                        end
                    end
                    
                    
                    % Approaching the stop
                    dist=check_dist_end_road_isr_trafsim(n,road);
                    if(dist<c.car(n).dist2light)
                        cmdacc=-1; cause=ccid;
                        ind=find(c.car(n).list>=100);
                        if(~isempty(ind))  % If exist vehicles in the detection list
                            aux=0;
                            for car=ind
                                id=c.car(n).list(car);
                                car_pos=find(c.listactive==id);
                                if(c.car(n).road==c.car(car_pos).road)
                                    dist_n=check_dist_end_road_isr_trafsim(n,c.car(n).road);
                                    dist_car=check_dist_end_road_isr_trafsim(car_pos,c.car(car_pos).road);
                                    if(dist_n>dist_car) % if exist car in front
                                        aux=1;
                                    end
                                end
                            end
                            if(aux==0)
                                return
                            end
                        else
                            return;
                        end
                        
                        
                        return;
                        
                    elseif(dist>c.car(n).dist2light)
                        %velmax=4+c.car(n).rand1*dist;
                        velmax=dist;
                        if( (c.car(n).dyn(1)*3.6) > velmax)
                            cmdacc=-1; cause=ccid;
                            % If exist other cars in front, the car doesn't stop because semaphore
                            ind=find(c.car(n).list>=100);
                            if(~isempty(ind))  % If exist vehicles in the detection list
                                aux=0;
                                for car=ind
                                    id=c.car(n).list(car);
                                    car_pos=find(c.listactive==id);
                                    if(c.car(n).road==c.car(car_pos).road)
                                        dist_n=check_dist_end_road_isr_trafsim(n,c.car(n).road);
                                        dist_car=check_dist_end_road_isr_trafsim(car_pos,c.car(car_pos).road);
                                        if(dist_n>dist_car) % if exist car in front (don't stop because semaphore)
                                            aux=1; cause=0;
                                        end
                                    end
                                end
                                if(aux==0)
                                    return
                                end
                            else
                                return;
                            end
                        end
                    end
                end
            elseif(ccid>=100)
                pos=find(c.listactive==ccid);                 % Position of close car in the list
                %                     dbstop at 130 in isr_tfs_traffic_gestion.m
                if(isempty(pos)==0)
                    ccroad=c.car(pos).traj(3,c.car(pos).setp);  % Road of the car that is close
                    nextroad=check_next_road_isr_trafsim(n,road);
                    if(road>=30 && road~=40 && ccroad<30)
                        % Car is inside of roundabout and detects a car outside offside of roundabout
                        danger=1;   % Detected car can obstruct exit road
                        [decision motive]=conflict_decision_isr_trafsim(n,pos,road,ccroad,danger,carid,ccid,0);
                    elseif(road<30 && ccroad>=30 && ccroad~=40)
                        % Car is outside of roundabout and detect car inside of roundabout
                        danger=2; % dbstop at 143 in isr_tfs_traffic_gestion.m
                        [decision motive]=conflict_decision_isr_trafsim(n,pos,road,ccroad,danger,carid,ccid,0);
                    elseif(road>=30 && road~=40 && ccroad>=30 && ccroad~=40)
                        % The 2 cars is inside of roundabout
                        danger=3; % dbstop at 151 in isr_tfs_traffic_gestion.m
                        [decision motive]=conflict_decision_isr_trafsim(n,pos,road,ccroad,danger,carid,ccid,0);
                    elseif(road<30 && road~=40 && ccroad<30 && ccroad~=40 && road~=ccroad)
                        % The car see another car in other straight lane
                        danger=4; % The cars can be in neighboor lanes
                        [decision motive]=conflict_decision_isr_trafsim(n,pos,road,ccroad,danger,carid,ccid,0);
                    elseif(road==40 && ccroad<30)
                        % If car is in crossroad and detects car outside crossroad is not his decision
                    elseif(road<30 && ccroad==40)
                        % If car see car inside of crossroads
                        danger=6;
                        [decision motive]=conflict_decision_isr_trafsim(n,pos,road,ccroad,danger,carid,ccid,0);
                    elseif(road==40 && ccroad==40)
                        % If car is in crossroad and detects car outside crossroad is not his decision
                    elseif(road==ccroad && road<30 && ccroad<30)
                        danger=5; % Cars in same straight lane
                        [decision motive]=conflict_decision_isr_trafsim(n,pos,road,ccroad,danger,carid,ccid,0);
                    elseif(road==40)
                        
                        
                    else
                        disp('traffic_gestion')
                        [carid road]
                        [ccid ccroad]
                        keyboard
                    end
                    % If exist danger the car will brake
                    if(danger>0)
                        if(decision), cmdacc=-1; cause=motive; return; end
                        
                    end
                    
                end
            end
        end
    end
end


end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cross normal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = cross_gestion_normal(n)
global s c

% Turn on the light of changing direction
if(isempty(find(s.table_gest0(1,:)==c.car(n).id))==0)
    
    % Turn on the light of changing direction
    if(c.car(n).hsidelight~=-1)
        delete(c.car(n).hsidelight);
        c.car(n).hsidelight=-1;
    end
    if(c.car(n).lightdirection==1 && s.mode==1)
        %             str=sprintf('R');
        %             c.car(n).hsidelight=text(c.car(n).pos.x,c.car(n).pos.y,str,'color','k','BackgroundColor','w','Fontsize',4);
    elseif(c.car(n).lightdirection==2 && s.mode==1)
        %             str=sprintf('L');
        %             c.car(n).hsidelight=text(c.car(n).pos.x,c.car(n).pos.y,str,'color','k','BackgroundColor','w','Fontsize',4);
    end
end

% Give permission to car advance
if(~isempty(find(s.table_gest0(1,:)==c.car(n).id)) && isempty(find(s.crosscar==c.car(n).id)) )
    road=c.car(n).traj(3,c.car(n).setp);
    if(s.lights(2,road)>0)
        out=check_road_free_isr_trafsim(c.car(n).rac,12);
        if(out)
            aux=0;
            for j=2:length(s.crosscar)
                pos=find(c.listactive==s.crosscar(1,j));
                if( isempty(intersect(c.car(pos).crosscels,c.car(n).crosscels))==0 )
                    aux=1;
                    break
                end
            end
            if(aux==0)
                c.car(n).nostop=[c.car(n).nostop c.car(n).traj(3,c.car(n).setp)];
                s.crosscar=[s.crosscar c.car(n).id];
            end
        end
    end
end

% Delete car from the list
if(isempty(find(s.table_gest0(1,:)==c.car(n).id))==0)
    if(c.car(n).traj(3,c.car(n).setp)~=40 && c.car(n).traj(3,c.car(n).prevsetp)==40)
        pos=find(s.crosscar==c.car(n).id);
        s.crosscar(:,pos)=[];
        pos=find(s.table_gest0(1,:)==c.car(n).id);
        s.table_gest0(:,pos)=[];
        if(c.car(n).hsidelight~=-1)
            delete(c.car(n).hsidelight);
        end
    end
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cross intelligent
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = cross_gestion_advanced(n)
global c s


% Turn on the light of changing direction
if(isempty(find(s.table_gest0(1,:)==c.car(n).id))==0)
    
    % Turn on the light of changing direction
    if(c.car(n).hsidelight~=-1)
        delete(c.car(n).hsidelight);
        c.car(n).hsidelight=-1;
    end
    if(c.car(n).lightdirection==1 && s.mode==1)
        %             str=sprintf('R');
        %             c.car(n).hsidelight=text(c.car(n).pos.x,c.car(n).pos.y,str,'color','k','BackgroundColor','w','Fontsize',4);
    elseif(c.car(n).lightdirection==2 && s.mode==1)
        %             str=sprintf('L');
        %             c.car(n).hsidelight=text(c.car(n).pos.x,c.car(n).pos.y,str,'color','k','BackgroundColor','w','Fontsize',4);
    end
end

% Manage the crossroad
if( isempty(intersect(c.car(n).traj(3,c.car(n).setp),s.crosslist))==0 )
    if(size(s.table_gest0,2)>1)
        if(isempty(find(s.table_gest0(1,:)==c.car(n).id))==0)
            
            % Add car to the cross list if is possible
            if(isempty(find(s.crosscar(1,:)==c.car(n).id)))
                if(size(s.crosscar,2)==1)
                    out=check_road_free_isr_trafsim(c.car(n).rac,12);
                    if(out)
                        
                        % Add nostop to the car list
                        c.car(n).nostop=[c.car(n).nostop c.car(n).traj(3,c.car(n).setp)];
                        s.crosscar=[s.crosscar c.car(n).id];
                        s.lights(2,c.car(n).traj(3,c.car(n).setp))=s.tgreen;
                        s.lights(3,c.car(n).traj(3,c.car(n).setp))=s.tyello;
                        
                    end
                else
                    out=check_road_free_isr_trafsim(c.car(n).rac,12);
                    if(out)
                        aux=0;
                        for j=2:length(s.crosscar)
                            pos=find(c.listactive==s.crosscar(1,j));
                            if( isempty(intersect(c.car(pos).crosscels,c.car(n).crosscels))==0 )
                                aux=1;
                                break
                            end
                        end
                        if(aux==0)
                            
                            % Add nostop to the car list
                            c.car(n).nostop=[c.car(n).nostop c.car(n).traj(3,c.car(n).setp)];
                            s.crosscar=[s.crosscar c.car(n).id];
                            s.lights(2,c.car(n).traj(3,c.car(n).setp))=s.tgreen;
                            s.lights(3,c.car(n).traj(3,c.car(n).setp))=s.tyello;
                            
                        end
                    end
                end
            end
        end
    end
end

% Delete car from the list
if(~isempty(find(s.table_gest0(1,:)==c.car(n).id)))
    if(c.car(n).traj(3,c.car(n).setp)~=40 && c.car(n).traj(3,c.car(n).prevsetp)==40)
        pos=find(s.crosscar==c.car(n).id);
        s.crosscar(:,pos)=[];
        pos=find(s.table_gest0(1,:)==c.car(n).id);
        s.table_gest0(:,pos)=[];
        if(c.car(n).hsidelight~=-1)
            delete(c.car(n).hsidelight);
        end
    end
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% View next road (cross)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [nextroad] = view_road_ahead(n,road)
global c s

dcross=20;
nextroad=0;
if(road>=5 && road<=10 || road==40 || road==15 || road==16)
    caution=0;
    cursetp=c.car(n).setp;
    curdist=c.car(n).traj(4,c.car(n).setp);
    while(curdist<c.car(n).traj(4,c.car(n).setp)+dcross)
        curdist=c.car(n).traj(4,cursetp);
        cursetp=cursetp+1;
        if(cursetp==c.car(n).trajn)
            caution=1;
            break
        end
    end
    if(caution)
        cursetp=c.car(n).trajn;
    end
    nextroad=max(c.car(n).traj(3,[c.car(n).setp:cursetp]));
end
end


