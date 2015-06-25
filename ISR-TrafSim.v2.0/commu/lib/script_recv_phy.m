% script_recv_phy;
t = event.instant;
i = event.pkt.tx;
j = event.node;
if bdebug, disp(['recv_phy @ node ' num2str(j)]); end
if node(j, 4) ~= 2
    %disp(['recv_phy: node ' num2str(j) ' is not in receive mode'])
    error(['recv_phy: node ' num2str(j) ' is not in receive mode']);
end
node(j, 4) = 0; % receiver switches back to idle mode
if t > navcommu(j).start & t < navcommu(j).end
    % this has already been checked when sending
    % but navcommu may be changed during transmission, so double check
    if ddebug, disp(['recv_phy: packet virtual collision at node ' num2str(j)]); end
    % just drop the packet
else
    [pr, snr] = recv_phy(i, j, rmodel);
    
%     disp(['recv_phy: node ' num2str(i) ' to node ' num2str(j) ' with snr= ' num2str(snr) ' and distance=' num2str(topo_dist(i, j))]);
    t1 = rv_threshold_delta;
    if snr >= (rv_threshold+t1)
        probability_receive = 1;
    elseif snr < (rv_threshold-t1)
        probability_receive = 0;
    elseif rand <= (snr-(rv_threshold-t1))/(t1+t1)
        probability_receive = 1;
    else
        probability_receive = 0;
    end
    if probability_receive
        if event.pkt.rv == 0 | event.pkt.rv == j   % broadcast or unicast to this receiver
            % TTL is taken care of at network layer
            % event.pkt.ttl = event.pkt.ttl - 1;
            % if event.pkt.ttl < 0
            %     if adebug, disp(['recv_phy: TTL from node ' num2str(i) ' to node ' num2str(j) ' is negative, drop the packet']); end
            %     return;
            % end
            % there is already a MAC layer packet waiting for transmission, but this incoming packet should be
            % received first, no futher receiving is possible (see send_phy)
            % if mac_status(j)
            %     if adebug, disp(['recv_phy: node ' num2str(j) ' is waiting to transmit, so cannot receive']); end
            newevent = event;
            newevent.instant = t;
            newevent.type = 'recv_mac';
            newevent.node = j;
            NewEvents = [NewEvents; newevent]; clear newevent;
        elseif event.pkt.navcommu > 0    % this packet is not for me, but use its navcommu
            if navcommu(j).start < t
                navcommu(j).start = t;
            end
            if navcommu(j).end < (t+event.pkt.navcommu)
                % question: debug
                navcommu(j).end = t + event.pkt.navcommu;
            end
        end
    else
        if bdebug, disp(['recv_phy: packet from node ' num2str(i) ' cannot be successfully received at node' num2str(j)]); end
    end
end
