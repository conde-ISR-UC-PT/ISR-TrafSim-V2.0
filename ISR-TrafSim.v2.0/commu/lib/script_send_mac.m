% script_send_mac;
t = event.instant;
i = event.node;
j = event.pkt.rv;
if bdebug, disp(['send_mac: node ' num2str(i) ' to send to node ' num2str(j) ' isempty(mac_queue)=' num2str(isempty(mac_queue(i).list)) ' mac_status=' num2str(mac_status(i))]); end
event.pkt.id = new_id(i);
event.pkt.type = 'data';    % used in function call of 'tx_time'
% the tx_time should be the same as in 'send_phy'
event.pkt.navcommu = SIFS + cts_tx_time + SIFS + tx_time(event.pkt) + SIFS + ack_tx_time;
% if ddebug, disp(['send_mac node ' num2str(i) ' will reserve navcommu=' num2str(event.pkt.navcommu)]); end
if j ~= 0
    % for unicast, RTS should be sent first
    event.pkt.type = 'rts';
end
% keep the data body size and rate for transmitting data later
if ~isempty(mac_queue(i).list) & ~mac_status(i)
    error(['send_mac: node ' num2str(i) ' channel is free, but there is still packets waiting at MAC...this should not happen']);
end
if ~isempty(mac_queue(i).list) | mac_status(i)
    % old packets are waiting to be sent, just wait behind them
    % or one packet is being transmitted at MAC layer, just wait in the MAC queue
    mac_queue(i).list = [mac_queue(i).list event];
else
    mac_status(i) = 1;
    % newevent.instant = t + turnaround_time; % swith to transmit
    newevent = event;
    newevent.instant = t + cca_time;    % check channel status
    newevent.type = 'wait_for_channel';
    newevent.node = i;
    NewEvents = [NewEvents; newevent]; clear newevent;
end
