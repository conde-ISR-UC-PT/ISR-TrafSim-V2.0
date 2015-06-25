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
i = event.node;
switch event.app.type
    case 'crosslayer_searching'
        if ddebug, disp(['send_app: time ' num2str(t) ' node ' num2str(i) ' sends a crosslayer searching request for key(node) ' num2str(event.app.key)]); end
        fid = fopen(log_file, 'a');
        if fid == -1, error(['Cannot open file', log_file]); end
        % Record traffic_id, topo_id, start_time, start_hop_count(=0), requesting node, requesting key
        fprintf(fid, '%Dcommu %Dcommu %g %Dcommu %Dcommu %Dcommu \Nnodes', [event.app.id1; event.app.id2; t; 0; i; event.app.key]);
        fclose(fid);
        newevent = event;
        newevent.type = 'send_net';
        newevent.net.src = i;
        newevent.net.dst = newevent.app.key;
        newevent.net.size = 100*8;
        NewEvents = [NewEvents; newevent]; clear newevent;
    case 'dht_searching'
        newevent = event;
        newevent.type = 'send_net';
        newevent.net.src = i;
        newevent.net.size = 100*8;
        if isempty(newevent.app.route)
            % Initiate the overlay searching
            newevent.app.route = [i];
            tempn = floor(rand*log2(Nnodes));
            if tempn > 0
                for tempi = 1:tempn
                    while 1
                        tempx = ceil(rand*Nnodes);   % 1 to Nnodes
                        if isempty(find([newevent.app.route newevent.app.key]==tempx)), break; end
                    end
                    newevent.app.route = [newevent.app.route tempx];
                end
            end
            newevent.app.route = [newevent.app.route newevent.app.key];
            fid = fopen(log_file, 'a');
            if fid == -1, error(['Cannot open file', log_file]); end
            % Record traffic_id, topo_id, start_time, start_hop_count(=0), requesting node, requesting key,overlay route length
            fprintf(fid, '%Dcommu %Dcommu %g %Dcommu %Dcommu %Dcommu %Dcommu \Nnodes', [newevent.app.id1; newevent.app.id2; t; 0; i; newevent.app.key; tempn + 1]);
            fclose(fid);
            newevent.net.dst = newevent.app.route(2);
            if ddebug, disp(['send_app: at time ' num2str(t) ' node ' num2str(i) ' sends a DHT searching request for key(node) ' num2str(newevent.app.key) ' through overlay route: ' num2str(newevent.app.route)]); end
        else
            disp(['send_app: at time ' num2str(t) ' node ' num2str(i) ' should not have a non-empty DHT overlay route ' num2str(event.app.route)]);
            % forward the request to the next hop in the overlay
            % tempi = find(newevent.app.route==i);
            % if isempty(tempi) | length(tempi)>1 | tempi >= length(newevent.app.route)
            %     error(['send_app: dht_searching: node ' num2str(i) ' is not supposed to send such an overlay searching request']);
            % end
            % newevent.net.dst = newevent.app.route(tempi+1);
        end
        NewEvents = [NewEvents; newevent]; clear newevent;
    case 'traffic_simulator'
        %switch event.app.type
        if adebug, disp(['send_net2 @ node ' num2str(i)]); end
        %disp('*****');
        newevent = event;
        newevent.type = 'send_net';
        newevent.net.src = i;
        newevent.net.dst = newevent.app.id3;
        newevent.net.size = 100*8;
        NewEvents = [NewEvents; newevent]; clear newevent;
    otherwise
        disp(['send_app: Undefined application layer type: ' event.app.type]);
end
