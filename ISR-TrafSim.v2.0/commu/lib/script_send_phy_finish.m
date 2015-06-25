% script_send_phy_finish
t = event.instant;
i = event.node;
j = event.pkt.rv;
if bdebug, disp(['send_phy_finish @ node ' num2str(i)]); end
if node(i, 4) ~= 1
    error(['send_phy_finish: node ' num2str(i) ' should be in transmission mode']);
end
node(i, 4) = 0; % after all receivings, go back to idle
node(i, 3) = 0;
if j==0 % | strcmp(event.pkt.type, 'ack')
    % finished broadcast % or finished RTS-CTS-DATA-ACK for unicast
    if ~isempty(mac_queue(i).list)
        % more packets are waiting to be sent
        mac_status(i) = 1;
        newevent = mac_queue(i).list(1);
        mac_queue(i).list(1) = [];
        newevent.instant = t + cca_time;   % question: should cca_time or other be used here?
        newevent.type = 'wait_for_channel';
        newevent.node = i;
        NewEvents = [NewEvents; newevent]; clear newevent;
    else
        mac_status(i) = 0;
    end
end
