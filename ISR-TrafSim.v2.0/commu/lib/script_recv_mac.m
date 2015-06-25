%script_recv_mac;

t = event.instant;
i = event.pkt.tx;
j = event.node;
if adebug, disp(['recv_mac @ node ' num2str(j)]); end
if event.pkt.rv == 0 & strcmp(event.pkt.type, 'data') == 0
    % broadcast but not data packet
    error(['recv_mac: node ' num2str(j) ' receives a broadcast packet with a wrong type: ' event.pkt.type]);
end
if j == i
    % I myself sent this packet, no action
    return;
end
switch event.pkt.type
    case 'rts'
        % send back a CTS
        newevent = event;
        newevent.instant = t + SIFS;
        newevent.type = 'send_phy';
        newevent.node = j;
        % keep the data size, rate, and id as RTS packet
        newevent.pkt.type = 'cts';
        newevent.pkt.tx=j;
        newevent.pkt.rv=i;
        newevent.pkt.navcommu=event.pkt.navcommu - SIFS - cts_tx_time;
        NewEvents = [NewEvents; newevent]; clear newevent;
    case 'cts'
        % remove pending id for RTS
        if pending_id(j) ~= event.pkt.id
            if ddebug, disp(['the received CTS id ' num2str(event.pkt.id) ' does not match the pending RTS id ' num2str(pending_id(j))]); end
            % probably this CTS is in response to an earlier RTS,
            % but I have retransmitted a new RTS which is replied
            % already or I have retransmitted so many times and given up
            % so we just ignore this CTS.
            return;
        end
        pending_id(j) = 0;
        retransmit(j) = 0;
        % send DATA
        newevent = event;
        newevent.instant = t + SIFS;
        newevent.type = 'send_phy';
        newevent.node = j;
        % keep the data size and rate as before
        % newevent.pkt.ttl = 1;
        newevent.pkt.type = 'data';
        newevent.pkt.tx=j;
        newevent.pkt.rv=i;
        % creat a new id for the data packet
        newevent.pkt.id = new_id(j);
        newevent.pkt.navcommu = 0; % not necessary because RTS already did so
        NewEvents = [NewEvents; newevent]; clear newevent;
    case 'data'
        % should check that this is not a duplicated or out-of-order packet
        if event.pkt.rv ~= 0    % send ACK if not broadcast
            % send back an ACK
            newevent = event;
            newevent.instant = t + SIFS;
            newevent.type = 'send_phy';
            newevent.node = j;
            % keep the data size, rate, and id the same as DATA packet
            newevent.pkt.type = 'ack';
            newevent.pkt.tx=j;
            newevent.pkt.rv=i;
            newevent.pkt.navcommu=0; % not necessary because CTS already did so
            NewEvents = [NewEvents; newevent]; clear newevent;
        end
        % send data up to network layer
        newevent = event;
        % Make sure the ACK is sent out before processing this data packet in
        % the upper layers because the upper layers may immediately
        % send more packets upon receiving this data packet.
        if event.pkt.rv ~= 0,
            newevent.instant = t + SIFS + ack_tx_time + 2*eps;
        else
            newevent.instant = t + 2*eps;
        end
        newevent.type = 'recv_net';
        newevent.node = j;
        NewEvents = [NewEvents; newevent]; clear newevent;
    case 'ack'
        % make sure the acknowledged packet is the just sent DATA packet
        if pending_id(j) ~= event.pkt.id
            if ddebug, disp(['the received ACK id=' num2str(event.pkt.id) ' does not match the pending DATA id=' num2str(pending_id(j))]); end
            % probably this is a duplicated ACK (same reason as the above CTS case)
            return;
        end
        % remove pending id for DATA
        pending_id(j) = 0;
        retransmit(j) = 0;
        if ~isempty(mac_queue(j).list)
            % more packets are waiting to be sent
            % newevent.instant = t + turnaround_time; % switch from receive to transmit
            % if ddebug, disp('recv_mac: after receiving ACK, take the next packet from mac_queue'); end
            mac_status(j) = 1;
            newevent = mac_queue(j).list(1);
            mac_queue(j).list(1) = [];
            newevent.instant = t + cca_time;
            newevent.type = 'wait_for_channel';
            newevent.node = j;
            % the packet setup is already done in 'send_mac'
            NewEvents = [NewEvents; newevent]; clear newevent;
        else
            mac_status(j) = 0;
        end
    otherwise
        disp(['recv_mac: Undefined mac packet type: ' event.pkt.type]);
end
