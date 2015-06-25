% script_timeout_data
t = event.instant;
i = event.node;
j = event.pkt.rv;
if adebug, disp(['timeout_data @ node ' num2str(i)]); end
%         if pending_id(i) == event.pkt.id % not acknowledged yet
%             if adebug, disp(['timeout_data: node ' num2str(i) ' failed to transmit DATA, go back to transmit RTS']); end
%             % remove the pending id for DATA
%             pending_id(i) = 0;
%             retransmit(i) = 0;
%             % go back to send RTS
%             newevent = event;
%             newevent.instant = t + cca_time;    % check channel status
%             newevent.type = 'wait_for_channel';
%             newevent.node = i;
%             newevent.pkt.type = 'data';
%             newevent.pkt.navcommu = SIFS + cts_tx_time + SIFS + tx_time(newevent.pkt) + SIFS + ack_tx_time;
%             newevent.pkt.type = 'rts';
%             % create a new id for the new RTS
%             newevent.pkt.id = new_id(i);
%             NewEvents = [NewEvents; newevent]; clear newevent;
%         end
if pending_id(i) == event.pkt.id % not acknowledged yet
    if cdebug, disp(['timeout_data: node ' num2str(i) ' pending_id=' num2str(pending_id(i)) ' event_id=' num2str(event.pkt.id)]); end
    retransmit(i) = retransmit(i) + 1;
    if retransmit(i) > max_retries
        % so many retries, drop the data packet
        if cdebug, disp(['timeout_data: node ' num2str(i) ' has retried so many times to transmit DATA']); end
        retransmit(i) = 0;
        pending_id(i) = 0;
        if ~isempty(mac_queue(i).list)
            % more packets are waiting to be sent
            mac_status(i) = 1;
            newevent = mac_queue(i).list(1);
            mac_queue(i).list(1) = [];
            newevent.instant = t + cca_time;    % question: cca_time or other
            newevent.type = 'wait_for_channel';
            newevent.node = i;
            NewEvents = [NewEvents; newevent]; clear newevent;
        else
            % Cannot send DATA successfully, reset MAC layer
            mac_status(i) = 0;
        end
        return;
    end
    if adebug, disp(['timeout_data: node ' num2str(i) ' to retransmit DATA']); end
    % retransmit the DATA
    newevent = event;
    newevent.instant = t + cca_time;    % check channel status
    newevent.type = 'wait_for_channel';
    newevent.node = i;
    % newevent.pkt.type = 'data';
    newevent.pkt.navcommu = SIFS + ack_tx_time; % necessary for retransmission because the initial DATA has navcommu=0
    NewEvents = [NewEvents; newevent]; clear newevent;
end
