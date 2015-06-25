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

t = event.instant;
j = event.node;
global c s

if bdebug, disp(['recv_app @ node ' num2str(j)]); end
switch event.app.type
    case 'crosslayer_searching'
        if ddebug, disp(['recv_app: time ' num2str(t) ' node ' num2str(j) ' receives the reply of crosslayer searching with route ' num2str(event.net.route)]); end
        fid = fopen(log_file, 'a');
        if fid == -1, error(['Cannot open file', log_file]); end
        % Record traffic_id, topo_id, end_time, hop_count, requesting node, requesting key
        fprintf(fid, '%Dcommu %Dcommu %g %Dcommu %Dcommu %Dcommu \Nnodes', [event.app.id1; event.app.id2; t; length(event.net.route)-1; j; event.app.key]);
        fclose(fid);
    case 'dht_searching'
        tempi = find(event.app.route==j);
        if isempty(tempi) | length(tempi)>1
            if ddebug, disp(['recv_app: dht_searching: node ' num2str(j) ' receives a wrong DHT searching request with route: ' num2str(event.app.route)]); end
            return;
        end
        if tempi == length(event.app.route)
            % I am the destination of this overlay request: send the answer back to the requestor
            % if ddebug, disp(['recv_app: at time ' num2str(t) ' destination node ' num2str(j) ' receives the DHT request from node ' num2str(event.app.route(1))]); end
            newevent = event;
            newevent.type = 'send_net';
            % Make sure the previous ACK at MAC layer is finished
            newevent.instant = t; % already taken care of in MAC layer. + SIFS + ack_tx_time + 2*eps;
            newevent.app.hopcount = newevent.app.hopcount + length(newevent.net.route) - 1;
            newevent.net.src = j;
            newevent.net.dst = newevent.app.route(1);
            newevent.net.size = 100*8;
            NewEvents = [NewEvents; newevent]; clear newevent;
        elseif tempi == 1
            % I am the requester: just received the answer from the destination
            if ddebug, disp(['recv_app: at time ' num2str(t) ' node ' num2str(j) ' receives the DHT reply']); end
            fid = fopen(log_file, 'a');
            if fid == -1, error(['Cannot open file', log_file]); end
            % Record traffic_id, topo_id, start_time, start_hop_count(=0), requesting node, requesting key, overlay route length
            fprintf(fid, '%Dcommu %Dcommu %g %Dcommu %Dcommu %Dcommu %Dcommu \Nnodes', [event.app.id1; event.app.id2; t; event.app.hopcount; j; event.app.key; length(event.app.route)-1]);
            fclose(fid);
        else
            % if ddebug, disp(['recv_app: at time ' num2str(t) ' overlay node ' num2str(j) ' will forward the DHT request to next overlay node ' num2str(event.app.route(tempi + 1))]); end
            % I should send request to the next hop in overlay
            newevent = event;
            newevent.type = 'send_net';
            % Make sure the previous ACK at MAC layer is finished
            newevent.instant = t;   % alread taken care of in MAC layer. + SIFS + ack_tx_time + 2*eps;
            newevent.app.hopcount = event.app.hopcount + length(event.net.route) - 1;
            newevent.net.src = j;
            newevent.net.dst = event.app.route(tempi + 1);
            newevent.net.size = 100*8;
            NewEvents = [NewEvents; newevent]; clear newevent;
        end
    case 'traffic_simulator'
        if(edebug)
            disp(['recv_app: at time ' num2str(t) ' node ' num2str(j) ' receives the traffic  simulator']);
            disp(['Node ' num2str(j) ' receives the ' event.app.info]);
        end
        if(strcmp(event.app.info,'gps_info'))
            
            id=s.otalist(j);
            pos=find(c.listactive==id);
            c.car(pos).commu_gps_info=1;
            c.car(pos).gps_time=s.time;
            %disp('receive RTK')
            if(s.printcommugpsinfo==1 && s.mode==1)
                plot(c.car(pos).pos.x,c.car(pos).pos.y,'w+')
            end
            %disp('rec_gpsinfo'); pause(0.01);
        elseif(strcmp(event.app.info,'vel_prof'))
            
            id=s.otalist(j);
            pos=find(c.listactive==id);
            c.car(pos).commu_vel_prof=1;
            if(s.printcommuvelprof==1 && s.mode==1)
                plot(c.car(pos).pos.x,c.car(pos).pos.y,'wo')
            end
        elseif(strcmp(event.app.info,'vel_prof2'))
            
            id=s.otalist(j);
            pos=find(c.listactive==id);
            c.car(pos).commu_vel_prof2=1;
            if(s.printcommuvelprof==1 && s.mode==1)
                plot(c.car(pos).pos.x,c.car(pos).pos.y,'wo')
            end
            %disp('rec_velprof');
        elseif(strcmp(event.app.info,'notify_round'))
            
            pos=find(s.round_notify_list(12,:)==event.app.road);
            if(~isempty(find(s.round_notify_list(1,:)==s.otalist(event.app.id4))))
                %                 disp(['Actualizes notify ' num2str(s.otalist(event.app.id4))])
                col=find(s.round_notify_list(1,:)==s.otalist(event.app.id4));
                aux=s.round_notify_list(4,col);
                s.round_notify_list(:,col)=[];
                s.round_notify_list=[s.round_notify_list [s.otalist(event.app.id4) ; event.app.id4 ; -1 ; aux ; event.app.pos(1) ; event.app.pos(2) ; event.app.track(1) ; event.app.track(2) ; event.app.curvel; event.app.maxvel; event.app.cartype ; event.app.road; s.time] ];
            elseif(isempty(pos))
                %                 disp(['add notify ' num2str(s.otalist(event.app.id4))])
                s.round_notify_list=[s.round_notify_list [s.otalist(event.app.id4) ; event.app.id4 ; -1 ; round2(t,0.001) ; event.app.pos(1) ; event.app.pos(2) ; event.app.track(1) ; event.app.track(2) ; event.app.curvel; event.app.maxvel; event.app.cartype ; event.app.road ; s.time] ];
            else
                dist1=dist_isr_trafsim(s.round_center(1),s.round_center(2),event.app.pos(1),event.app.pos(2));
                dist2=dist_isr_trafsim(s.round_center(1),s.round_center(2),s.round_notify_list(5,pos),s.round_notify_list(6,pos));
                if(dist1<dist2)
                    %                     disp(['Subs notify ' num2str(s.round_notify_list(1,pos)) ' -> ' num2str(s.otalist(event.app.id4))])
                    s.round_notify_list(:,pos)=[];
                    s.round_notify_list=[s.round_notify_list [s.otalist(event.app.id4) ; event.app.id4 ; -1 ; round2(t,0.001) ; event.app.pos(1) ; event.app.pos(2) ; event.app.track(1) ; event.app.track(2) ; event.app.curvel; event.app.maxvel; event.app.cartype ; event.app.road; s.time] ];
                else
                    %                     disp(['No subs ' num2str(s.round_notify_list(1,pos)) ' -> ' num2str(s.otalist(event.app.id4))])
                end
            end
            
            if(s.printcommunotify==1 && s.mode==1)
                pos=find(c.listactive==s.otalist(event.app.id4));
                plot(c.car(pos).pos.x,c.car(pos).pos.y,'w^')
            end
        elseif(strcmp(event.app.info,'notify_cross'))
            
            pos=find(s.cross_notify_list(12,:)==event.app.road);
            if(~isempty(find(s.cross_notify_list(1,:)==s.otalist(event.app.id4))))
                %                 disp(['Actualizes notify ' num2str(s.otalist(event.app.id4))])
                col=find(s.cross_notify_list(1,:)==s.otalist(event.app.id4));
                aux=s.cross_notify_list(4,col);
                s.cross_notify_list(:,col)=[];
                s.cross_notify_list=[s.cross_notify_list [s.otalist(event.app.id4) ; event.app.id4 ; -1 ; aux ; event.app.pos(1) ; event.app.pos(2) ; event.app.track(1) ; event.app.track(2) ; event.app.curvel; event.app.maxvel; event.app.cartype ; event.app.road; s.time] ];
            elseif(isempty(pos))
                %                 disp(['add notify ' num2str(s.otalist(event.app.id4))])
                s.cross_notify_list=[s.cross_notify_list [s.otalist(event.app.id4) ; event.app.id4 ; -1 ; round2(t,0.001) ; event.app.pos(1) ; event.app.pos(2) ; event.app.track(1) ; event.app.track(2) ; event.app.curvel; event.app.maxvel; event.app.cartype ; event.app.road ; s.time] ];
            else
                dist1=dist_isr_trafsim(s.cross_center(1),s.cross_center(2),event.app.pos(1),event.app.pos(2));
                dist2=dist_isr_trafsim(s.cross_center(1),s.cross_center(2),s.cross_notify_list(5,pos),s.cross_notify_list(6,pos));
                if(dist1<dist2)
                    %                     disp(['Subs notify ' num2str(s.round_notify_list(1,pos)) ' -> ' num2str(s.otalist(event.app.id4))])
                    s.cross_notify_list(:,pos)=[];
                    s.cross_notify_list=[s.cross_notify_list [s.otalist(event.app.id4) ; event.app.id4 ; -1 ; round2(t,0.001) ; event.app.pos(1) ; event.app.pos(2) ; event.app.track(1) ; event.app.track(2) ; event.app.curvel; event.app.maxvel; event.app.cartype ; event.app.road; s.time] ];
                else
                    %                     disp(['No subs ' num2str(s.round_notify_list(1,pos)) ' -> ' num2str(s.otalist(event.app.id4))])
                end
            end
            
            if(s.printcommunotify==1 && s.mode==1)
                pos=find(c.listactive==s.otalist(event.app.id4));
                plot(c.car(pos).pos.x,c.car(pos).pos.y,'w^')
            end
        else
            keyboard
        end
    otherwise
        disp(['recv_app: Undefined application layer type: ' event.app.type]);
end
