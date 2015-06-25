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
% Drivers decision
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

function [decision motive] = conflict_decision_isr_trafsim(n,pos,road,ccroad,danger,carid,ccid,aux)
global c s
decision=0;
motive=0;

% Danger = 1 -> Car is inside of roundabout and detects a car outside of roundabout
% Danger = 2 -> Car is outside of roundabout and detect car inside of roundabout
% Danger = 3 -> 2 cars is inside of roundabout
% Danger = 4 -> The car see another car in other straight lane
% Danger = 5 -> Cars in same straight lane
% Danger = 6 ->
% Danger = 7 ->
% Danger = 8 ->

% Car is inside of roundabout and detects a car outside of roundabout
if(danger==1)
    %dbstop at 211 in isr_tfs_traffic_gestion.m
    % Only mathers if the car is in the exit lane of current car
    nextroad=check_next_road_isr_trafsim(n,road);
    if(nextroad<30 && nextroad==ccroad) % If car is in the exit lane
        % List all vehicles inside roundabout in exit lane
        list=[];
        for i=1:size(c.car(n).list,2)
            id=c.car(n).list(i);
            if(id>=100) % Only if detects car
                pospos=find(c.listactive==id);
                if(c.car(pospos).traj(3,c.car(pospos).setp)==nextroad)
                    list=[list  [pospos ;check_dist_end_road_isr_trafsim(pospos,nextroad);c.car(pospos).id]];
                end
            end
        end
        %list
        [val col]=max(list(2,:));
        pospos=find(c.listactive==list(3,col));
        cvel=c.car(n).dyn(1); ccvel=c.car(list(1,col)).dyn(1);
        dist_from_start=s.ri(nextroad,6)-list(2,col);
        vellim=3/3.6;
        dist=check_dist_start_road_isr_trafsim(n,nextroad);
        %space=check_road_free(nextroad,8);        % To verify if exist space in exit road to put car
        if(dist_from_start>25)
            % Do nothing
            %disp(['>20 ' num2str(carid) '  ' num2str(c.car(pospos).id)])
        elseif(dist_from_start<=25 && dist_from_start>20)
            %disp(['8>x>20 ' num2str(carid) '  ' num2str(c.car(pospos).id)])
            if(ccvel<vellim)
                if(cvel>vellim)
                    %disp(['# ' num2str(carid) '  ' num2str(c.car(pospos).id)])
                    decision=1;
                    motive=c.car(pospos).id;
                end
            end
        elseif(dist_from_start<20)
            %disp(['<8 ' num2str(carid) '  ' num2str(c.car(pospos).id)])
            if(dist>2.5 && cvel>vellim)          % If car isnt near, cancel decision
                %disp(['## ' num2str(carid) '  ' num2str(c.car(pospos).id)])
                decision=1;
                motive=c.car(pospos).id;
            elseif(dist<2.5)
                %disp(['### ' num2str(carid) '  ' num2str(c.car(pospos).id)])
                decision=1;
                motive=c.car(pospos).id;
            end
            
        end
        
    end
    
    % Car is outside of roundabout and detect car inside of roundabout
elseif(danger==2)
    
    px=c.car(n).pos.x;          py=c.car(n).pos.y;      pt=c.car(n).pos.t;      ps=c.car(n).steerval;
    pxx=c.car(pos).pos.x;       pyy=c.car(pos).pos.y;   ptt=c.car(pos).pos.t;   pss=c.car(pos).steerval;
    ang1=atan2((py-50),(px-50));        ang2=atan2((pyy-50),(pxx-50));
    cvel=c.car(n).dyn(1); ccvel=c.car(pos).dyn(1);
    angdif=dist_ang_isr_trafsim(ang1,ang2);     % If angdif>0 the cc is in front (anti-clockwise)
    nextroad=check_next_road_isr_trafsim(n,road);
    if(nextroad>=30 && nextroad<40)
        if( angdif>-60*(pi/180) && angdif<=10 )   % If detected car is approching
            decision=1;
            motive=ccid;
        elseif( angdif>10 && angdif<20*(pi/180) )
            if( ccvel>=(30/3.6) )
                point=direc_inter_isr_trafsim(px,py,pt,pxx,pyy,ptt,s.sd,s.lsd,ps,pss);
                if(point(1)~=0)  % If exist danger of colision
                    cx=point(2); cy=point(3);                                           % Coordinates of colision point
                    dc=dist_isr_trafsim(px,py,cx,cy); dcc=dist_isr_trafsim(pxx,pyy,cx,cy);      % Distance to colision point
                    ct=dc/cvel; cct=dcc/ccvel;                                          % Time until colision point
                    if(point(1)==2), cct=0; end
                    if(abs(ct-cct)<3)                                               % If time difference < 2s
                        decision=1;
                        motive=ccid;
                    end
                end
            else
                decision=1;
                motive=ccid;
            end
        end
    end
    % Regras que anulam a paragem
    if(decision && dist_isr_trafsim(px,py,pxx,pyy)>10)
        dist=check_dist_start_road_isr_trafsim(n,nextroad);
        if(dist>1 && dist>ccvel)   % If car is not near (5m away from roundabout ) and velocity is bellow 5Km/h
            % Cancel stoping decision
            decision=0;
            motive=0;
            
        end
        roadtab1=check_next_next_road_isr_trafsim(n,road);
        roadtab2=check_next_next_road_isr_trafsim(pos,ccroad);
        auxtab=0;
        for k=1:min(size(roadtab1,2),3)
            for j=1:min(size(roadtab2,2),2)
                if(roadtab1(k)==roadtab2(j)), auxtab=1; end
            end
        end
        if(auxtab==0)
            % Cancel stoping decision
            decision=0;
            motive=0;
        end
    end
    % The 2 cars is inside of roundabout
elseif(danger==3)
    px=c.car(n).pos.x;          py=c.car(n).pos.y;      pt=c.car(n).pos.t;      ps=c.car(n).steerval;
    pxx=c.car(pos).pos.x;       pyy=c.car(pos).pos.y;   ptt=c.car(pos).pos.t;   pss=c.car(pos).steerval;
    ang1=atan2((py-50),(px-50));        ang2=atan2((pyy-50),(pxx-50));
    angdif=dist_ang_isr_trafsim(ang1,ang2);     % If angdif>0 the cc is in front (anti-clockwise)
    cvel=c.car(n).dyn(1); ccvel=c.car(pos).dyn(1);
    %if(mod(road,2)==1), carlane='inside'; elseif(mod(road,2)==0), carlane='outside'; end
    %if(mod(ccroad,2)==1), cclane='inside'; elseif(mod(ccroad,2)==0), cclane='outside'; end
    
    if(angdif<=0)
        % The cc is behind, do nothing
    elseif(angdif>0 && angdif<60*(pi/180) )    % The cc is in front and close 50º, verify danger
        point=direc_inter_isr_trafsim(px,py,pt,pxx,pyy,ptt,s.sd,s.lsd,ps,pss);
        if(point(1)~=0)  % If exist danger of colision
            %                 dbstop at 231 in isr_tfs_traffic_gestion.m
            cx=point(2); cy=point(3);                                           % Coordinates of colision point
            dc=dist_isr_trafsim(px,py,cx,cy); dcc=dist_isr_trafsim(pxx,pyy,cx,cy);      % Distance to colision point
            ct=dc/cvel; cct=dcc/ccvel;                                          % Time until colision point
            if(point(1)==2), cct=0; end
            if(dcc<4), cct=0; end
            if(dc<4), ct=0; end
            if(abs(ct-cct)<4 || dist_isr_trafsim(pxx,pyy,px,py)<10 )                                               % If time difference < 2s
                decision=1;
                motive=ccid;
            end
        end
    end
    % The car see another car in other straight lane
elseif(danger==4)
    if(mod(road,2)==1), carlane='inside'; elseif(mod(road,2)==0), carlane='outside'; end
    %if(mod(ccroad,2)==1), cclane='inside'; elseif(mod(ccroad,2)==0), cclane='outside'; end
    
    % To detect neighbor lanes
    if(strcmp(carlane,'inside') && ccroad==road+1)
        close=1;
    elseif(strcmp(carlane,'outside') && ccroad==road-1)
        close=1;
    else
        close=0;
    end
    
    if(close==1)    % The cars are in neighboor lanes
        carnextroad=check_next_road_isr_trafsim(n,road);
        ccnextroad=check_next_road_isr_trafsim(pos,ccroad);
        % Only detect colision if cars are near of the end of roads
        if(ccnextroad~=0 && carnextroad~=0 && check_dist_start_road_isr_trafsim(n,carnextroad)<10 && check_dist_start_road_isr_trafsim(pos,ccnextroad)<10 && carnextroad~=40 && ccnextroad~=40)
            px=c.car(n).pos.x;          py=c.car(n).pos.y;      pt=c.car(n).pos.t;      ps=c.car(n).steerval;
            pxx=c.car(pos).pos.x;       pyy=c.car(pos).pos.y;   ptt=c.car(pos).pos.t;   pss=c.car(pos).steerval;
            point=direc_inter_isr_trafsim(px,py,pt,pxx,pyy,ptt,s.sd,s.lsd,ps,pss);
            cvel=c.car(n).dyn(1); ccvel=c.car(pos).dyn(1);
            if(point(1)~=0)  % If exist danger of colision
                cx=point(2); cy=point(3);                                           % Coordinates of colision point
                dc=dist_isr_trafsim(px,py,cx,cy); dcc=dist_isr_trafsim(pxx,pyy,cx,cy);      % Distance to colision point
                ct=dc/cvel; cct=dcc/ccvel;                                          % Time until colision point
                if(point(1)==2), cct=0; end
                if(abs(ct-cct)<3)                                               % If time difference < 2s
                    
                    decision=1;
                    motive=ccid;
                    
                end
            end
        end
        
        % Annuls previous decision
        if(decision==1 && ccvel==0) % If close car is sttoped
            decision=0;
            motive=0;
        end
    end
elseif(danger==5) % Cars in same straight lane
    px=c.car(n).pos.x;          py=c.car(n).pos.y;      pt=c.car(n).pos.t;      ps=c.car(n).steerval;   time2stop=c.car(n).time2stop; dist2stop=c.car(n).dist2stop;
    pxx=c.car(pos).pos.x;       pyy=c.car(pos).pos.y;   ptt=c.car(pos).pos.t;   pss=c.car(pos).steerval;
    %point=direc_inter_isr_trafsim(px,py,pt,pxx,pyy,ptt,s.sd,s.lsd,ps,pss);
    cvel=c.car(n).dyn(1); 
    %ccvel=c.car(pos).dyn(1);
    
    %keyboard
    
    if(check_dist_end_road_isr_trafsim(n,road) > check_dist_end_road_isr_trafsim(pos,ccroad))
        
        
        % min_gap=c.car(n).dist2stop;
        % T = 1s
        % deltaV=vf-vi
        % a=s.dynmaxacc(1) \\ b=s.dynmaxacc(2)
        % s*=s0+(v*T+ (v*deltaV)/(2*sqrt(a*b)))
        dist_=c.car(n).dist2stop+( cvel*1+((cvel*(cvel-c.car(n).prvdyn(1)))/(2*sqrt(s.dynmaxacc(1)*s.dynmaxacc(2)))) );
        % dv=a[1 - (v/vp) - (dist_/s)^2] where vp is speed limit and s is
        % the distance between the two vehicles
        dist=dist_isr_trafsim(px,py,pxx,pyy);
        dv=s.dynmaxacc(1)*(1 - (cvel/(50/3.6)) - (dist_/dist)^2);
        
        if(dv<=0)
            decision=1;
            motive=ccid;
        end
    end
elseif(danger==6)
    cond1=~isempty(find(s.crosslist==road)); % If car is in access to crossroads
    [ll, cc]=find(s.rg_count_tab2==ccid);
    if(~isempty(ll) && ~isempty(cc))    % If car spoted have the same access road
        cond2=road==c.car(pos).rbc;
    else
        cond2=0;
    end
    
    if(cond1 && cond2)
        px=c.car(n).pos.x;          py=c.car(n).pos.y;
        pxx=c.car(pos).pos.x;       pyy=c.car(pos).pos.y;
        if(dist_isr_trafsim(px,py,pxx,pyy)<6)
            decision=1;
            motive=ccid;
        end
    end
end

end

