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
% Manage the nodes list, update that list (nodes position) and schedule the
% the sending of packets
%
%-USE----------------------------------------------------------------------
%
% -> Input(s)
%   » 'arg'
%       refresh     -   Actualize number of nodes and nodes position
%       add_node    -   Add a communication node
%       del_node    -   Delete a communication node
%       [senderID, receiverID, type_of_MSG]
%                   -   Send information from one node to other
%
% -> Output(s)
%   » None
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

function [out] = ota_commu_isr_trafsim(arg,aux)
out=0;
[~, c]=size(arg);

if(c==7)                            % 'refresh' positions of the nodes
    commu_refresh();
elseif(c==8 && strcmp(arg,'add_node'))      % 'add_node'
    commu_add_node(aux);
elseif(c==8 && strcmp(arg,'del_node'))      % 'del_node
    commu_del_node(aux);
else
    commu_send(arg,aux);                % send a package arg=3, [senderID, receiverID, type_of_MSG]
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Add a new node
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = commu_add_node(id)
global s
global Nnodes navcommu mac_status mac_queue net_pending net_queue rrep_table retransmit pending_id packet_id backoff_attempt backoff_counter rrep_in_crosslayer rrep_forward_crosslayer rrep_destination_crosslayer bcast_table rreq_out  rreq_in rreq_forward rreq_out_crosslayer rreq_in_crosslayer rreq_forward_crosslayer rrep_out rrep_in rrep_forward rrep_out_crosslayer

% Verify if id is not on the the nodes list (s.otalist)
pos=find(s.otalist==id, 1);
if(isempty(pos))
    s.otalist=[s.otalist id];
else
    disp('OTA Commu ERROR')
end

% Add new space on necessary lists
i=find(s.otalist==id);
navcommu(i).start=0;
navcommu(i).end=0;
mac_queue(i).list=[];
mac_status(i)=0;
net_pending(i).id=[];
net_pending(i).retransmit=[];
net_queue(i).list=[];
rrep_table(i).list=[];
backoff_counter(i,1) = 0;
backoff_attempt(i,1) = 0;
packet_id(i,1) = 0;
pending_id(i,1) = 0;
retransmit(i,1) = 0;
rreq_out(i,1) = 0;
rreq_in(i,1) = 0;
rreq_forward(i,1) = 0;
rreq_out_crosslayer(i,1) = 0;
rreq_in_crosslayer(i,1) = 0;
rreq_forward_crosslayer(i,1) = 0;
rrep_out(i,1) = 0;
rrep_in(i,1) = 0;
rrep_forward(i,1) = 0;
rrep_out_crosslayer(i,1) = 0;
rrep_in_crosslayer(i,1) = 0;
rrep_forward_crosslayer(i,1) = 0;
rrep_destination_crosslayer(i,1) = 0;

% Refresh number of nodes
Nnodes=Nnodes+1;

% Refresh broadcast table
bcast_table = zeros(Nnodes , Nnodes);

% Refresh mobility parameters
%poscommu = zeros(Nnodes, 6);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Delete a node
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = commu_del_node(id)
global s
global Nnodes navcommu mac_status mac_queue net_pending net_queue rrep_table retransmit pending_id packet_id backoff_attempt backoff_counter rrep_in_crosslayer rrep_forward_crosslayer rrep_destination_crosslayer bcast_table rreq_out  rreq_in rreq_forward rreq_out_crosslayer rreq_in_crosslayer rreq_forward_crosslayer rrep_out rrep_in rrep_forward rrep_out_crosslayer

% Verify if id is on the nodes list (s.otalist)
i=find(s.otalist==id);
if(isempty(i))
    disp('OTA Commu ERROR')
end

% Delete space on necessary lists
s.otalist(i)=[];
navcommu(i)=[];
mac_queue(i)=[];
mac_status(i)=[];
net_pending(i)=[];
net_queue(i)=[];
rrep_table(i)=[];
backoff_counter(i,:)=[];
backoff_attempt(i,:)=[];
packet_id(i,:)=[];
pending_id(i,:)=[];
retransmit(i,:)=[];
rreq_out(i,:)=[];
rreq_in(i,:)=[];
rreq_forward(i,:)=[];
rreq_out_crosslayer(i,:)=[];
rreq_in_crosslayer(i,:)=[];
rreq_forward_crosslayer(i,:)=[];
rrep_out(i,:)=[];
rrep_in(i,:)=[];
rrep_forward(i,:)=[];
rrep_out_crosslayer(i,:)=[];
rrep_in_crosslayer(i,:)=[];
rrep_forward_crosslayer(i,:)=[];
rrep_destination_crosslayer(i,:)=[];

% Refresh number of nodes
Nnodes=Nnodes-1;

% Refresh broadcast table
bcast_table = zeros(Nnodes , Nnodes);

% Refresh mobility parameters
%poscommu = zeros(Nnodes, 6);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Send information from one node to other
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = commu_send(arg,aux)
% dbstop at 250 in commu_send
global s c
global Event_list;
global Nnodes ;

if(s.otacommuemulation==0)
    rid=arg(2);
    sid=arg(1);
    info=aux;
    if(strcmp(info,'notify_round'))
        n=find(c.listactive==sid);
        if(~isempty(intersect(c.car(n).traj(3,c.car(n).setp),s.roundlist)) )
            % Inside the Roundabout
            r=dist_isr_trafsim(s.refposxRound,s.refposyRound,c.car(n).pos.x,c.car(n).pos.y);
            if(r<=s.otarangeround)
                if(s.cartxpos==0)       % True Position
                    pos = [c.car(n).pos.x c.car(n).pos.y];
                elseif(s.cartxpos==1)   % Fusion Position
                    pos = [c.car(n).fus(1,1) c.car(n).fus(1,2)];
                elseif(s.cartxpos==2)   % GPS position
                    pos = [c.car(n).gps(1,1) c.car(n).gps(1,2)];
                end
                track = [c.car(n).roadori c.car(n).roaddes];
                curvel = c.car(n).dyn(1);
                maxvel = c.car(n).dri.velmax;
                cartype = c.car(n).type;
                road=c.car(n).traj(3,c.car(n).setp);
                
                
                ppp=find(s.round_notify_list(12,:)==road);
                if(~isempty(find(s.round_notify_list(1,:)==sid, 1)))
                    col=find(s.round_notify_list(1,:)==sid);
                    aux=s.round_notify_list(4,col);
                    s.round_notify_list(:,col)=[];
                    
                    s.round_notify_list=[s.round_notify_list [sid ; 0 ; -1 ; aux ; pos(1) ; pos(2) ; track(1) ; track(2) ; curvel; maxvel; cartype ; road; s.time] ];
                elseif(isempty(ppp))
                    s.round_notify_list=[s.round_notify_list [sid ; 0 ; -1 ; round2(s.time,0.001) ; pos(1) ; pos(2) ; track(1) ; track(2) ; curvel; maxvel; cartype ; road; s.time] ];
                else
                    
                    dist1=dist_isr_trafsim(s.refposxRound,s.refposyRound,pos(1),pos(2));
                    dist2=dist_isr_trafsim(s.refposxRound,s.refposyRound,s.round_notify_list(5,ppp),s.round_notify_list(6,ppp));
                    if(dist1<dist2)
                        s.round_notify_list(:,ppp)=[];
                        s.round_notify_list=[s.round_notify_list [sid ; 0 ; -1 ; round2(s.time,0.001) ; pos(1) ; pos(2) ; track(1) ; track(2) ; curvel; maxvel; cartype ; road; s.time] ];
                    end
                end
                
                if(s.printcommunotify==1 && s.mode==1)
                    paux=find(c.listactive==sid);
                    plot(c.car(paux).pos.x,c.car(paux).pos.y,'w^')
                end
            else
                if(s.printcommunotify==1 && s.mode==1)
                    paux=find(c.listactive==sid);
                    plot(c.car(paux).pos.x,c.car(paux).pos.y,'b^')
                end
            end
        end
    elseif(strcmp(info,'notify_cross'))
        n=find(c.listactive==sid);
        if(~isempty(intersect(c.car(n).traj(3,c.car(n).setp),s.crosslist)))
            % Inside the Crossroad
            r=dist_isr_trafsim(s.refposxCross,s.refposyCross,c.car(n).pos.x,c.car(n).pos.y);
            if(r<=s.otarangecross)
                if(s.cartxpos==0)       % True Position
                    pos = [c.car(n).pos.x c.car(n).pos.y];
                elseif(s.cartxpos==1)   % Fusion Position
                    pos = [c.car(n).fus(1,1) c.car(n).fus(1,2)];
                elseif(s.cartxpos==2)   % GPS position
                    pos = [c.car(n).gps(1,1) c.car(n).gps(1,2)];
                end
                track = [c.car(n).roadori c.car(n).roaddes];
                curvel = c.car(n).dyn(1);
                maxvel = c.car(n).dri.velmax;
                cartype = c.car(n).type;
                road=c.car(n).traj(3,c.car(n).setp);
                
                ppp=find(s.cross_notify_list(12,:)==road);
                
                % Refresh notification list
                if(~isempty(find(s.cross_notify_list(1,:)==sid, 1)))
                    col=find(s.cross_notify_list(1,:)==sid);
                    aux=s.cross_notify_list(4,col);
                    s.cross_notify_list(:,col)=[];
                    
                    s.cross_notify_list=[s.cross_notify_list [sid ; 0 ; -1 ; aux ; pos(1) ; pos(2) ; track(1) ; track(2) ; curvel; maxvel; cartype ; road; s.time] ];
                    % If is not on notification list
                elseif(isempty(ppp))
                    s.cross_notify_list=[s.cross_notify_list [sid ; 0 ; -1 ; round2(s.time,0.001) ; pos(1) ; pos(2) ; track(1) ; track(2) ; curvel; maxvel; cartype ; road; s.time] ];
                else
                    %keyboard
                    dist1=dist_isr_trafsim(s.refposxCross,s.refposyCross,pos(1),pos(2));
                    dist2=dist_isr_trafsim(s.refposxCross,s.refposyCross,s.cross_notify_list(5,ppp),s.cross_notify_list(6,ppp));
                    if(dist1<dist2)
                        s.cross_notify_list(:,ppp)=[];
                        s.cross_notify_list=[s.cross_notify_list [sid ; 0 ; -1 ; round2(s.time,0.001) ; pos(1) ; pos(2) ; track(1) ; track(2) ; curvel; maxvel; cartype ; road; s.time] ];
                    end
                end
                
                if(s.printcommunotify==1 && s.mode==1)
                    paux=find(c.listactive==sid);
                    plot(c.car(paux).pos.x,c.car(paux).pos.y,'w^')
                end
            else
                if(s.printcommunotify==1 && s.mode==1)
                    paux=find(c.listactive==sid);
                    plot(c.car(paux).pos.x,c.car(paux).pos.y,'b^')
                end
            end
        end
        
    elseif(strcmp(info,'gps_info'))
        if(rid==0)
            for n=1:size(c.listactive,2)
                r=dist_isr_trafsim(s.refposxRound,s.refposyRound,c.car(n).pos.x,c.car(n).pos.y);
                if(r<=s.otarangeGPS)
                    c.car(n).gps_time=s.time;
                    if(s.printcommugpsinfo==1 && s.mode==1)
                        plot(c.car(n).pos.x,c.car(n).pos.y,'w+')
                    end
                else
                    if(s.printcommugpsinfo==1 && s.mode==1)
                        plot(c.car(n).pos.x,c.car(n).pos.y,'b+')
                    end
                end
            end
        end
    elseif(strcmp(info,'vel_prof'))
        n=find(c.listactive==rid);
        
        r=dist_isr_trafsim(s.refposxRound,s.refposyRound,c.car(n).pos.x,c.car(n).pos.y);
        if(r<=s.otarangeround)
            c.car(n).commu_vel_prof=1;
            if(s.printcommuvelprof==1 && s.mode==1)
                plot(c.car(n).pos.x,c.car(n).pos.y,'wo')
            end
        else
            if(s.printcommuvelprof==1 && s.mode==1)
                plot(c.car(n).pos.x,c.car(n).pos.y,'bo')
            end
        end
    elseif(strcmp(info,'vel_prof2'))
        n=find(c.listactive==rid);
        
        r=dist_isr_trafsim(s.refposxCross,s.refposyCross,c.car(n).pos.x,c.car(n).pos.y);
        if(r<=s.otarangecross)
            c.car(n).commu_vel_prof2=1;
            if(s.printcommuvelprof==1 && s.mode==1)
                plot(c.car(n).pos.x,c.car(n).pos.y,'wo')
            end
        else
            if(s.printcommuvelprof==1 && s.mode==1)
                plot(c.car(n).pos.x,c.car(n).pos.y,'bo')
            end
        end
    end
    
else
    % sender ID
    snode=find(s.otalist==arg(1));
    
    % receiver ID
    if(arg(2)==0)
        rnode=0;        % Broadcast
    else
        rnode=find(s.otalist==arg(2));
    end
    
    if( isempty(snode) || isempty(rnode) )
        disp('OTA Commu ERROR');
    end
    
    % Type of message
    apptype = 'traffic_simulator';
    msg='send_app';
    info=aux;
    
    k=1+length(Event_list);
    
    Event_list(k).instant = arg(3);
    Event_list(k).type = msg;
    Event_list(k).node = snode;
    Event_list(k).app.type = apptype;
    Event_list(k).app.key = Nnodes+1-k;
    Event_list(k).app.id1 = 0;
    Event_list(k).app.id2 = 0;
    Event_list(k).app.id3 = rnode;
    Event_list(k).app.id4 = snode;
    Event_list(k).app.info = info;
    Event_list(k).app.route = [];
    Event_list(k).app.hopcount = 0;
    Event_list(k).app.castmethod = 'block_broadcast';
    Event_list(k).app.neighbours = 'with_knowledge';
    Event_list(k).net = [];
    Event_list(k).pkt = [];
    
    
    if(strcmp(info,'notify_round'))
        n=find(c.listactive==s.otalist(snode));
        if(s.cartxpos==0)       % True Position
            Event_list(k).app.pos = [c.car(n).pos.x c.car(n).pos.y];
        elseif(s.cartxpos==1)   % Fusion Position
            Event_list(k).app.pos = [c.car(n).fus(1,1) c.car(n).fus(1,2)];
        elseif(s.cartxpos==2)   % GPS position
            Event_list(k).app.pos = [c.car(n).gps(1,1) c.car(n).gps(1,2)];
        end
        Event_list(k).app.track = [c.car(n).roadori c.car(n).roaddes];
        Event_list(k).app.curvel = c.car(n).dyn(1);
        Event_list(k).app.maxvel = c.car(n).dri.velmax;
        Event_list(k).app.cartype = c.car(n).type;
        Event_list(k).app.road = c.car(n).traj(3,c.car(n).setp);
    elseif(strcmp(info,'notify_cross'))
        n=find(c.listactive==s.otalist(snode));
        if(s.cartxpos==0)       % True Position
            Event_list(k).app.pos = [c.car(n).pos.x c.car(n).pos.y];
        elseif(s.cartxpos==1)   % Fusion Position
            Event_list(k).app.pos = [c.car(n).fus(1,1) c.car(n).fus(1,2)];
        elseif(s.cartxpos==2)   % GPS position
            Event_list(k).app.pos = [c.car(n).gps(1,1) c.car(n).gps(1,2)];
        end
        Event_list(k).app.track = [c.car(n).roadori c.car(n).roaddes];
        Event_list(k).app.curvel = c.car(n).dyn(1);
        Event_list(k).app.maxvel = c.car(n).dri.velmax;
        Event_list(k).app.cartype = c.car(n).type;
        Event_list(k).app.road = c.car(n).traj(3,c.car(n).setp);
    elseif(strcmp(info,'gps_info'))
        
        
    elseif(strcmp(info,'vel_prof'))
        
    elseif(strcmp(info,'vel_prof2'))
        
    end
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Actualize number of nodes and nodes position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = commu_refresh()
global c s gps_data Nnodes node;

% Refresh number of nodes
Nnodes=length(s.otalist);

% Reference station
node=[];
for i=1:Nnodes
    id=s.otalist(i);
    if(id==90)
        pos=xyz2enu(gps_data.xyz_origin_ref,s.xyz_origin);
        node(i,:)=[pos(1) pos(2) 0 0 0 0];
    else
        pos=find(c.listactive==id);
        node(i,:)=[c.car(pos).pos.x c.car(pos).pos.y 0 0 0 0];
    end
end

end
