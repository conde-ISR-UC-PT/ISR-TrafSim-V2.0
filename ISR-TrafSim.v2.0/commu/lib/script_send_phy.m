% script_send_phy
t = event.instant;
i = event.node;
j = event.pkt.rv;
if adebug, disp(['send_phy at time ' num2str(t) ' node ' num2str(i) ' will send a packet to node ' num2str(j)]); end
txtime = tx_time(event.pkt);
if node(i, 4) == 0 & (navcommu(i).start > (t+txtime) | navcommu(i).end < t) % idle and no navcommu
    node(i, 3) = event.pkt.power;
    node(i, 4) = 1; % switch to transmit mode, assume turnaround time is zero
    % set up the receiver
    if j == 0   % broadcast from node i
        for k=1:Nnodes
            % due to broadcast nature in wireless channel, every idle node may capture/sense this transmission
            if node(k, 4)~=0 | k==i, continue; end
            if overlap(t, t+txtime, navcommu(k).start, navcommu(k).end), continue; end
            node(k, 4) = 2; % receiver switches to receiving mode
            newevent = event;
            newevent.instant = t + txtime;
            newevent.type = 'recv_phy';
            newevent.node = k;
            NewEvents = [NewEvents; newevent]; clear newevent;
        end
        % when there is no node to receive this broadcast, still send it.
        % because I do not know if any node will respond, but actually node node will respond.
        %                 if length(NewEvents) <= 0
        %                     if ddebug, disp(['send_phy: node ' num2str(i) ' broadcasts to no nodes']); end
        %                     newevent = event;
        %                     newevent.instant = t;
        %                     newevent.type = 'send_phy_finish';
        %                     newevent.node = i;
        %                     NewEvents = [NewEvents; newevent]; clear newevent;
        %                     return;
        %                 end
    else    % unicast from i to j
        if node(j, 4) ~= 0 | overlap(t, t+txtime, navcommu(j).start, navcommu(j).end)
            if ddebug, disp(['send_phy: receiving node ' num2str(j) ' is not ready to receive from node ' num2str(i)]); end
            % At physical layer, I cannot synchronize with the receiver, so I know I do not need to actually transmit?
            % We sill transmit this packet, but no actual reception.
            %                     newevent = event;
            %                     newevent.instant = t;
            %                     newevent.type = 'send_phy_finish';
            %                     newevent.node = i;
            %                     NewEvents = [NewEvents; newevent]; clear newevent;
            %                     return;
        else
            node(j, 4) = 2; % receiver is switched to receiving mode
            newevent = event;
            newevent.instant = t + txtime;
            newevent.type = 'recv_phy';
            newevent.node = j;
            NewEvents = [NewEvents; newevent]; clear newevent;
        end
        for k=1:Nnodes
            % due to broadcast nature in wireless channel, every idle node may capture/sense this transmission
            if node(k, 4)~=0 | k==i | k==j, continue; end
            if overlap(t, t+txtime, navcommu(k).start, navcommu(k).end), continue; end
            node(k, 4) = 2; % receiver switches to receiving mode
            newevent = event;
            newevent.instant = t + txtime;
            newevent.type = 'recv_phy';
            newevent.node = k;
            NewEvents = [NewEvents; newevent]; clear newevent;
        end
    end
    % setup the transmitter
    newevent = event;
    newevent.instant = t + txtime + eps;
    newevent.type = 'send_phy_finish';
    newevent.node = i;
    NewEvents = [NewEvents; newevent]; clear newevent;
    if strcmp(event.pkt.type, 'rts')
        % set timeout timer for RTS
        newevent = event;
        newevent.instant = t + (txtime + SIFS + cts_tx_time) * 2;   % question: how to choose this timeout limit?
        newevent.type = 'timeout_rts';
        newevent.node = i;
        NewEvents = [NewEvents; newevent]; clear newevent;
        if retransmit(i) <= 0 & pending_id(i) > 0
            %%%%%%%%%%% Modification -> Clear the timoutRTS in the 'Event_list'
            global Event_list
            tot=size(Event_list,2); cont=1;
            while(cont<=tot)
                if(Event_list(cont).pkt.id==pending_id(i) && Event_list(cont).pkt.tx==i)
                    Event_list(cont)=[];
                    tot=tot-1;
                end
                cont=cont+1;
            end
            %%%%%%%%%%% End of modification
            %dbstop at 80 in script_send_phy.m
            %disp(['### -> Pending error at ' num2str(s.time) ' (s)'])
            %pause
            %error(['send_phy: node ' num2str(i) ' there is already a pending packet, cannot send a new RTS packet']);end
        end
        pending_id(i) = event.pkt.id;
    end
    if strcmp(event.pkt.type, 'data') & j ~= 0
        % set timeout timer for DATA
        newevent = event;
        newevent.instant = t + (txtime + SIFS + ack_tx_time) * 2;   % double check
        newevent.type = 'timeout_data';
        newevent.node = i;
        NewEvents = [NewEvents; newevent]; clear newevent;
        if retransmit(i) <= 0 & pending_id(i) > 0
            %error(['send_phy: node ' num2str(i) ' there is already a pending packet, cannot send a new DATA packet']);
        end
        pending_id(i) = event.pkt.id;
    end
else    % radio hardware is not idle or navcommu block
    if adebug, disp(['send_phy at time ' num2str(t) ' node ' num2str(i) ' is not ready to send a packet to node ' num2str(j)]); end
    if adebug, disp(['--- node(i, 4)=' num2str(node(i, 4)) 'navcommu.start=' num2str(navcommu(i).start) 'navcommu.end=' num2str(navcommu(i).end)]); end
    % Since the node status is already checked at MAC layer, it must be due to navcommu virtual carrier sense
    % I am a hiddent node: physical carrier sense is okay, but blocked by virtual carrier sense
    % I should go back to MAC and try later.
    newevent = event;
    newevent.instant = t + cca_time;
    newevent.type = 'wait_for_channel';
    newevent.node = i;
    NewEvents = [NewEvents; newevent]; clear newevent;
    % Drop the packet, and try next MAC packet if any
    %             if ~isempty(mac_queue(i).list)
    %                 % more packets are waiting to be sent
    %                 mac_status(i) = 1;
    %                 newevent = mac_queue(i).list(1);
    %                 mac_queue(i).list(1) = [];
    %                 newevent.instant = t + cca_time;   % question: should cca_time or other be used here?
    %                 newevent.type = 'wait_for_channel';
    %                 newevent.node = i;
    %                 NewEvents = [NewEvents; newevent]; clear newevent;
    %             else
    %                 mac_status(i) = 0;
    %             end
end
