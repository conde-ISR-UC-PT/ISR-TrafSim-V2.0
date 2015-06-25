% script_send_net2;
t = event.instant;
i = event.node;
j = event.net.dst;
if adebug, disp(['send_net2 @ node ' num2str(i)]); end
% if ddebug, disp(['send_net2: time ' num2str(t) ' node ' num2str(i) ' starts to send a packet to node ' num2str(j)]); end
if j == 0   % broadcast
    newevent = event;
    newevent.instant = t;
    newevent.type = 'send_mac';
    newevent.node = i;
    newevent.net.type = 'data';
    newevent.net.id = new_id(i);
    newevent.net.route = [];
    newevent.net.metric = 0;
    newevent.pkt.tx=i; % or event.net.src
    newevent.pkt.rv=0;
    newevent.pkt.type='data';
    newevent.pkt.size=event.net.size;   % assume no header in network layer
    %             if event.pkt.ttl <= 0, newevent.pkt.ttl = default_ttl; end
    %             if event.pkt.rate <= 0, newevent.pkt.rate=default_rate; end
    %             if event.pkt.power <= 0, newevent.pkt.power=default_power; end
    if event.app.castmethod=='block_broadcast'
        newevent.pkt.ttl = 1;%default_ttl;
    else
        newevent.pkt.ttl = default_ttl;
    end
    newevent.pkt.rate=default_rate;
    newevent.pkt.power=default_power;
    newevent.pkt.id=0;
    newevent.pkt.navcommu=0;
    NewEvents = [NewEvents; newevent]; clear newevent;
else
    if event.app.neighbours == 'with_knowledge'
        newevent = event;
        newevent.instant = t;
        newevent.type = 'send_mac';
        newevent.node = i;
        newevent.net.type = 'data';
        newevent.net.src = i;
        newevent.net.dst = j;
        %define net.route
        newevent.net.route = [i j];
        newevent.net.metric = 0;    % no use for now
        newevent.pkt.tx=i;
        newevent.pkt.rv=j;
        newevent.pkt.type='data';
        % keep net.size
        newevent.pkt.size=event.net.size;
        newevent.pkt.ttl = 1;
        newevent.pkt.rate=default_rate;
        newevent.pkt.power=default_power;
        newevent.pkt.id=0;  % will be updated in 'send_phy'
        newevent.pkt.navcommu=0; % will be updated in lower layer
        
%         aux_tmp=node(i, 3);
%         node(i, 3) = newevent.pkt.power;
%         for k=1:size(node,1)
%             if i==k, continue, end;
%             [pr, snr] = recv_phy(i, k, rmodel);
%             % disp(['recv_phy: node ' num2str(i) ' to node ' num2str(j) ' with snr= ' num2str(snr) ' and distance=' num2str(topo_dist(i, j))]);
%             t1 = rv_threshold_delta;
%             if snr >= (rv_threshold+t1)
%                 probability_receive = 1;
%             elseif snr < (rv_threshold-t1)
%                 probability_receive = 0;
%             elseif rand <= (snr-(rv_threshold-t1))/(t1+t1)
%                 probability_receive = 1;
%             else
%                 probability_receive = 0;
%             end
%             if probability_receive
%                 disp(['neighbour:' num2str(k)] );
%             end
%         end
%         node(i, 3)=aux_tmp;
        NewEvents = [NewEvents; newevent];
        clear newevent;
        
    elseif event.app.neighbours == 'no___knowledge'
        % unicast: find the route by RREP-RREQ
        % assume no neighbor table, find route even dst. is in the neighborhood
        % assume no routing table, RREP will contain whole route
        newevent = event;
        newevent.instant = t;
        newevent.type = 'send_mac';
        newevent.node = i;
        newevent.net.type = 'rreq';
        rreq_out(i) = rreq_out(i) + 1;
        if strcmp(newevent.app.type, 'crosslayer_searching')
            rreq_out_crosslayer(i) = rreq_out_crosslayer(i) + 1;
        end
        % if ddebug, disp(['rreq_out(' num2str(i) ')=' num2str(rreq_out(i))]); end
        newevent.net.id = new_id(i);
        newevent.net.route = [i];
        newevent.net.metric = 0;    % no use for now
        newevent.pkt.tx=i;  % or event.net.src
        newevent.pkt.rv=0;  % broadcast RREQ
        newevent.pkt.type='data';
        newevent.pkt.size=size_rreq;
        %             if event.pkt.ttl <= 0, newevent.pkt.ttl = default_ttl; end
        %             if event.pkt.rate <= 0, newevent.pkt.rate=default_rate; end
        %             if event.pkt.power <= 0, newevent.pkt.power=default_power; end
        newevent.pkt.ttl = default_ttl;
        newevent.pkt.rate=default_rate;
        newevent.pkt.power=default_power;
        newevent.pkt.id=0;
        newevent.pkt.navcommu=0;
        NewEvents = [NewEvents; newevent];
        % set timeout timer for RREQ
        newevent.instant = t + rreq_timeout;   % question: how large should this timeout be?
        newevent.type = 'timeout_rreq';
        NewEvents = [NewEvents; newevent];
        net_pending(i).id = [net_pending(i).id newevent.net.id];   % save the id of pending RREQ
        net_pending(i).retransmit = [net_pending(i).retransmit 0];
        clear newevent;
    end
end
