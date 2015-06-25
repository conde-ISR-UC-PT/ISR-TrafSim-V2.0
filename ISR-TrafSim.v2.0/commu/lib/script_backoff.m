%script_backoff;
t = event.instant;
i = event.node;
j = event.pkt.rv;
if bdebug, disp(['backoff @ node ' num2str(i)]); end
if node(i, 4) == 0 & carrier_sense(i) == 0
    % the node is still idle and the channel is free, continue backoff
    if backoff_counter(i) > 1
        backoff_counter(i) = backoff_counter(i) - 1;
        newevent = event;
        newevent.instant = t + slot_time;
        newevent.type = 'backoff';
        newevent.node = i;
        NewEvents = [NewEvents; newevent]; clear newevent;
    else    % ready to send the packet
        backoff_counter(i) = 0; % reset counter for next use
        newevent = event;
        newevent.instant = t;
        newevent.type = 'send_phy';
        newevent.node = i;
        NewEvents = [NewEvents; newevent];
        % txtime = tx_time(newevent.pkt);
        clear newevent;
        % mac_queue will be taken care of after really send this packet in 'send_phy_finish'
        %                 if j == 0   % broadcast: the real data is sent here
        %                     % we can send the next packet from mac_queue now
        %                     if ~isempty(mac_queue(i).list)
        %                         % more packets are waiting to be sent
        %                         newevent = mac_queue(i).list(1);
        %                         mac_queue(i).list(1) = [];
        %                         newevent.instant = t + txtime + cca_time;   % question: should cca_time or other be used here?
        %                         newevent.type = 'wait_for_channel';
        %                         newevent.node = i;
        %                         NewEvents = [NewEvents; newevent]; clear newevent;
        %                     else
        %                         % will reset in 'send_phy_finish'
        %                         % mac_status(i) = 0;
        %                     end
        %                 else
        %                     % unicast: the RTS is sent here, will wait for CTS or timeout_rts
        %                     % do nothing here
        %                 end
    end
else   % channel becomes busy during backoff count-down
    if backoff_counter(i) > 1
        backoff_counter(i) = backoff_counter(i) - 1;
    else
        % start a new backoff counter when count-down is zero
        backoff_attempt(i) = backoff_attempt(i) + 1;
        temp = min(backoff_attempt(i)+CW_min,CW_max);
        backoff_counter(i) = floor((2^temp-1)*rand);
    end
    newevent = event;
    newevent.instant = t + cca_time;
    newevent.type = 'wait_for_channel';
    newevent.node = i;
    NewEvents = [NewEvents; newevent]; clear newevent;
end
