%script_recv_net
t = event.instant;
i = event.net.src;
j = event.node;
if bdebug, disp(['time ' num2str(t) ' recv_net @ node ' num2str(j)]); end
% take care of TTL at network layer
event.pkt.ttl = event.pkt.ttl - 1;
if event.pkt.ttl < 0
    if bdebug, disp(['recv_net: TTL from node ' num2str(i) ' to ' num2str(j) ' is negative, drop the packet']); end
    return;
end
if j == i | j == event.pkt.tx
    % I myself sent this packet, no action
    return;
end
% if cdebug, disp(['time ' num2str(t) 'node ' num2str(event.pkt.tx) ' -> node ' num2str(j) ' with type ' event.net.type]); end
switch event.net.type
    case 'rreq'
        rreq_in(j) = rreq_in(j) + 1;
        if strcmp(event.app.type, 'crosslayer_searching')
            rreq_in_crosslayer(j) = rreq_in_crosslayer(j) + 1;
        end
        if sum(ismember(event.net.route, j))
            % I am already in the found route
            return;
        end
        event.net.route = [event.net.route j];
        if j == event.net.dst
            % I am the destination of this RREQ: send RREP back
            % check if I have already replied to the same RREQ
            % if ~isempty(rrep_table) & sum(ismember(rrep_table, [i event.net.id], 'rows'))
            % we currently use: rrep_table.id, rrep_table.metric, rrep_table.route
            send_rrep = -1;
            if isempty(rrep_table(j).list)
                k = 1;
                send_rrep = 1;
            else
                % rrep_table is not empty
                for k=1:length(rrep_table(j).list)
                    if rrep_table(j).list(k).route(1)==i
                        % find a early saved RREQ from the same src
                        % assume this is the only saved RREQ from the same src
                        if rrep_table(j).list(k).id < event.net.id
                            % I replied to an older RREQ: take the new one and reply
                            send_rrep = 1;
                        elseif rrep_table(j).list(k).id == event.net.id
                            % I replied to the same RREQ: should I reply to again?
                            if event.net.metric < rrep_table(j).list(k).metric
                                % metric: the samller the better
                                % This is a better route, take it and reply
                                send_rrep = 1;
                            else
                                % not a better route: ignore
                                send_rrep = 0;
                            end
                        else
                            % I replied to a newer RREQ: ignore
                            send_rrep = 0;
                        end
                        break;
                    end
                end
            end
            if send_rrep ~= 0
                rrep_out(j) = rrep_out(j) + 1;
                if strcmp(event.app.type, 'crosslayer_searching')
                    rrep_out_crosslayer(j) = rrep_out_crosslayer(j) + 1;
                end
                if send_rrep < 0
                    % no early saved RREQ from this src: add one
                    k = length(rrep_table(j).list) + 1; % same as: k = k + 1;
                end
                rrep_table(j).list(k).id = event.net.id;
                rrep_table(j).list(k).metric = event.net.metric;
                rrep_table(j).list(k).route = event.net.route;
                newevent = event;
                newevent.instant = t;
                newevent.type = 'send_mac';
                newevent.net.type = 'rrep';
                newevent.net.src = j;
                newevent.net.dst = i;
                newevent.pkt.tx = j;
                newevent.pkt.rv = newevent.net.route(length(newevent.net.route)-1); % next hop
                newevent.pkt.type='data';
                newevent.pkt.size=size_rrep;
                newevent.pkt.rate=default_rate;
                newevent.pkt.ttl = default_ttl;   % unicast question: what value for this TTL?
                newevent.pkt.power=default_power;
                newevent.pkt.id=0;  % will be updated in 'send_phy'
                newevent.pkt.navcommu=0; % will be updated in lower layer
                NewEvents = [NewEvents; newevent]; clear newevent;
                if cdebug, disp(['node ' num2str(j) ' will send an RREP with route ' num2str(event.net.route) ' at time ' num2str(t)]); end
            end
            return;
        end
        % I am not the destination of this RREQ: just re-broadcast it
        % maybe one of my previous RREPs contains the route to the
        % requesting source node, but do not worry for now
        if event.pkt.ttl < 0
            % already checked above, no use
            % cannot go further: drop it
        else
            if event.net.id > bcast_table(j, event.net.src)
                % forward this RREQ only if I have not forwarded the
                % same broadcast RREQ from the same source before
                rreq_forward(j) = rreq_forward(j) + 1;
                if strcmp(event.app.type, 'crosslayer_searching')
                    rreq_forward_crosslayer(j) = rreq_forward_crosslayer(j) + 1;
                end
                bcast_table(j, event.net.src) = event.net.id;
                newevent = event;
                newevent.instant = t + rand*slot_time;  % question: random delay before rebroadcasting
                newevent.type = 'send_mac';
                newevent.node = j;
                newevent.pkt.tx=j;
                newevent.pkt.rv=0;
                NewEvents = [NewEvents; newevent]; clear newevent;
            end
        end
    case 'rrep'
        % if cdebug, disp(['time ' num2str(t) ' node ' num2str(j) ' receives a RREP with route: ' num2str(event.net.route)]); end
        rrep_in(j) = rrep_in(j) + 1;
        if strcmp(event.app.type, 'crosslayer_searching')
            rrep_in_crosslayer(j) = rrep_in_crosslayer(j) + 1;
        end
        if isempty(event.net.route)
            warning(['recv_net: node ' num2str(j) ' is receiving a RREP without any route entry']);
            return;
        end
        temp = find(event.net.route == j);
        if length(temp) > 1
            warning(['recv_net: node ' num2str(j) ' appears more than once in a RREP']);
            return;
        end
        if length(temp) <= 0
            warning(['recv_net: node ' num2str(j) ' does not appear in a RREP it receives']);
            return;
        end
        if temp == 1
            % I am the requesting node so this RREP is what I am waiting for
            if cdebug, disp(['time ' num2str(t) ' node ' num2str(j) ' receives a RREP with route: ' num2str(event.net.route)]); end
            temp2 = find(net_pending(j).id == event.net.id);
            if isempty(temp2)
                % no RREQ waiting for this RREP;
                % probably this is an RREP for an earlier timeout RREQ, but I have already received an RREP for the latest RREQ.
                if ddebug, disp(['recv_net: node ' num2str(j) ' receives an RREP without a corresponding pending RREQ']); end
                return;
            end
            if length(temp2) > 1
                error(['recv_net: node ' num2str(j) ' receives an RREP with more than one pending RREQ']);
            end
            % Removes the pending RREQ
            net_pending(j).id(temp2) = [];
            net_pending(j).retransmit(temp2) = [];
            if strcmp(event.app.type, 'crosslayer_searching')
                % cross-layer searching application, no data to transmit
                % send the packet up to the application layer
                rrep_destination_crosslayer(j) = rrep_destination_crosslayer(j) + 1;
                newevent = event;
                newevent.instant = t;
                newevent.node = j;
                newevent.type = 'recv_app';
                NewEvents = [NewEvents; newevent]; clear newevent;
            else    % a regular RREP at network layer received
                % send the following data packet by this route
                newevent = event;
                newevent.instant = t;
                newevent.type = 'send_mac';
                newevent.node = j;
                newevent.net.type = 'data';
                newevent.net.id = new_id(j);
                newevent.net.src = j;
                newevent.net.dst = i;
                % keep net.size, net.route
                newevent.pkt.tx = j;
                newevent.pkt.rv = newevent.net.route(2); % next hop
                newevent.pkt.type='data';
                newevent.pkt.size=newevent.net.size;
                newevent.pkt.rate=default_rate;
                newevent.pkt.ttl = length(newevent.net.route) + 1;
                newevent.pkt.power=default_power;
                newevent.pkt.id=0;  % will be updated in 'send_phy'
                newevent.pkt.navcommu=0; % will be updated in lower layer
                NewEvents = [NewEvents; newevent];
                clear newevent;
            end
            % no ACK at network layer
            % the net_queue is always empty, so no next network layer packet to send
        else
            % I need to forward this RREP back to the next hop towards the source
            rrep_forward(j) = rrep_forward(j) + 1;
            if strcmp(event.app.type, 'crosslayer_searching')
                rrep_forward_crosslayer(j) = rrep_forward_crosslayer(j) + 1;
            end
            newevent = event;
            newevent.instant = t;
            newevent.type = 'send_mac';
            newevent.node = j;
            newevent.net.type = 'rrep';
            newevent.pkt.tx = j;
            newevent.pkt.rv = newevent.net.route(temp - 1); % next hop
            % if cdebug, disp(['time ' num2str(t) ' node ' num2str(j) ' will forward RREP to node ' num2str(newevent.pkt.rv)]); end
            NewEvents = [NewEvents; newevent]; clear newevent;
        end
    case 'data'
        if event.net.dst == 0
            % a network layer broadcast packet
            if event.pkt.rv ~= 0
                warning(['recv_net: node ' num2str(j) ' receives a broadcast at NET, but not at MAC']);
            end
            if event.app.castmethod=='block_broadcast'
                newevent = event;
                newevent.instant = t;
                newevent.type = 'recv_app';
                newevent.node = j;
                NewEvents = [NewEvents; newevent]; clear newevent;
                
                return;
            elseif event.app.castmethod=='allow_broadcast'
                if event.net.id > bcast_table(j, event.net.src)
                    bcast_table(j, event.net.src) = event.net.id;
                    newevent = event;
                    newevent.instant = t + rand*slot_time;
                    newevent.type = 'send_mac';
                    newevent.pkt.tx = j;
                    NewEvents = [NewEvents; newevent]; clear newevent;
                    
                end
            end
            return;
        end
        % receives a unicast data packet at network layer
        if isempty(event.net.route)
            warning(['recv_net: node ' num2str(j) ' is receiving a Net_DATA without any route entry']);
            return;
        end
        temp = find(event.net.route == j);
        if length(temp) > 1
            warning(['recv_net: node ' num2str(j) ' appears more than once in a route for data packet']);
            return;
        end
        if length(temp) <= 0
            warning(['recv_net: node ' num2str(j) ' does not appear in a data packet it receives']);
            return;
        end
        if j == event.net.dst   % or temp == length(event.net.route)
            % I am the destination
            newevent = event;
            newevent.instant = t;
            newevent.type = 'recv_app';
            newevent.node = j;
            NewEvents = [NewEvents; newevent]; clear newevent;
        else
            % I should forward this data packet to the next hop towards the destination
            newevent = event;
            newevent.instant = t;
            newevent.type = 'send_mac';
            newevent.pkt.tx = j;
            newevent.pkt.rv = newevent.net.route(temp + 1); % next hop
            NewEvents = [NewEvents; newevent]; clear newevent;
        end
    otherwise
        disp(['recv_net: Undefined network layer packet type: ' event.net.type]);
end
