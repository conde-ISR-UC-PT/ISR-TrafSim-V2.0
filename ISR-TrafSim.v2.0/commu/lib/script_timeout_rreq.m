% script_timeout_rreq
t = event.instant;
i = event.node;
j = event.net.dst;
if bdebug, disp(['timeout_rreq @ node ' num2str(i)]); end
temp = find(net_pending(i).id == event.net.id);
if isempty(temp)
    % The RREQ is already acknowledged and is not pending anymore, do nothing
    return;
end
if length(temp) > 1
    error(['timeout_rreq: node ' num2str(i) ' has more than one pending RREQs with id=' num2str(event.net.id)]);
end
% The RREQ is not acknowledged yet by an RREP
if ddebug, disp(['timeout_rreq: at time: ' num2str(t) ' node ' num2str(i) ' pending RREQ id=' num2str(net_pending(i).id(temp))]); end
net_pending(i).retransmit(temp) = net_pending(i).retransmit(temp) + 1;
if net_pending(i).retransmit(temp) > net_max_retries
    % so many retries, drop the RREQ
    % An RREP may come later, will just ignore
    if ddebug, disp(['timeout_rreq: node ' num2str(i) ' has retried so many times to transmit RREQ']); end
    net_pending(i).id(temp) = [];
    net_pending(i).retransmit(temp) = [];
    %             Should take care of MAC layer queue
    %             if ~isempty(mac_queue(i).list)
    %                 % more packets are waiting to be sent
    %                 % newevent.instant = t + turnaround_time; % switch from receive to transmit
    %                 mac_status(i) = 1;
    %                 newevent = mac_queue(i).list(1);
    %                 mac_queue(i).list(1) = [];
    %                 newevent.instant = t + cca_time;    % question: cca_time or other
    %                 newevent.type = 'wait_for_channel';
    %                 newevent.node = i;
    %                 % packet setup is already done in 'send_mac' before put into the mac_queue
    %                 NewEvents = [NewEvents; newevent]; clear newevent;
    %             else
    %                 % cannot send RREQ successfully, reset MAC layer
    %                 mac_status(i) = 0;
    %             end
    if ~isempty(net_queue(i).list)
        disp('* Tout rreq')
        % this is redundant, net_queue is always empty
        % more packets are waiting to be sent
        error(['timeout_rreq: node ' num2str(i) ' have a non-empty network layer queue']);
        newevent = net_queue(i).list(1);
        net_queue(i).list(1) = [];
        newevent.instant = t;
        newevent.type = 'send_net2';
        NewEvents = [NewEvents; newevent]; clear newevent;
    end
    return;
end
if adebug, disp(['timeout_rreq: node ' num2str(i) ' to retransmit RREQ']); end
% retransmit the RREQ
newevent = event;
newevent.instant = t;
newevent.type = 'send_mac';
newevent.net.id = new_id(i);  % question: do we need a new id for this retransmission? answer: yes, for bcast_table
net_pending(i).id(temp) = newevent.net.id;
rreq_out(i) = rreq_out(i) + 1;
if strcmp(newevent.app.type, 'crosslayer_searching')
    rreq_out_crosslayer(i) = rreq_out_crosslayer(i) + 1;
end
NewEvents = [NewEvents; newevent];
% set timeout timer for the retransmitted RREQ
newevent.instant = t + rreq_timeout;   % question: same as above
newevent.type = 'timeout_rreq';
NewEvents = [NewEvents; newevent];
net_pending(i).id(temp) = newevent.net.id;   % save the new id of the pending RREQ
clear newevent;
