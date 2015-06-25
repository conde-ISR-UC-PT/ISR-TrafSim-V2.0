%script_wait_for_channel
t = event.instant;
i = event.node;
j = event.pkt.rv;
if bdebug, disp(['wait_for_channel @ node ' num2str(i)]); end
%         if mac_status(i) == 0
%             % Reset by timeout_rreq after many RREQ retries
%             if ddebug, disp(['wait_for_channel: node ' num2str(i) 'mac_status reset because so many RREQ retries at network layer']); end
%             return;
%         end
if node(i, 4) == 0 & carrier_sense(i) == 0
    % question: I want to transmit, but have to capture from many neighbors
    % the node is idle and the channel is free, can backoff now
    if backoff_counter(i) > 0   % resume the backoff
        newevent = event;
        newevent.instant = t + slot_time;
        newevent.type = 'backoff';
        newevent.node = i;
        NewEvents = [NewEvents; newevent]; clear newevent;
    else                        % start from DIFS first
        newevent = event;
        newevent.instant = t + DIFS;
        newevent.type = 'backoff_start';
        newevent.node = i;
        NewEvents = [NewEvents; newevent]; clear newevent;
    end
else
    % the node is not idle; must be receiving...wait until this receiving is finished
    % or the channel is not free; wait until the channel is free
    newevent = event;
    newevent.instant = t + cca_time;
    newevent.type = 'wait_for_channel';
    newevent.node = i;
    NewEvents = [NewEvents; newevent]; clear newevent;
end
