%script_timeout_rts
%dbstop at 22 in script_timeout_rts.m

t = event.instant;
i = event.node;
j = event.pkt.rv;
if adebug, disp(['timeout_rts @ node ' num2str(i)]); end
if pending_id(i) == event.pkt.id % not acknowledged yet, retransmit
    if cdebug, disp(['timeout_rts: node ' num2str(i) ' pending_id=' num2str(pending_id(i)) ' event_id=' num2str(event.pkt.id)]); end
    retransmit(i) = retransmit(i) + 1;
    if retransmit(i) > max_retries
            %disp('# Tout RTS')
            %[event.node event.app.id3]
            if(strcmp(event.app.info,'gps_info'))
                global c s
                id=s.otalist(event.app.id3);
                pos=find(c.listactive==id);
                if(s.printcommugpsinfo==1 && s.mode==1)
                    plot(c.car(pos).pos.x,c.car(pos).pos.y,'b+')
                end
            elseif(strcmp(event.app.info,'notify'))
                global c s
                id=s.otalist(event.app.id4);
                pos=find(c.listactive==id);
                if(s.printcommunotify==1 && s.mode==1)
                    plot(c.car(pos).pos.x,c.car(pos).pos.y,'b^')
                end
            elseif(strcmp(event.app.info,'vel_prof'))
                global c s
                id=s.otalist(event.app.id3);
                pos=find(c.listactive==id);
                if(s.printcommuvelprof==1 && s.mode==1)
                    plot(c.car(pos).pos.x,c.car(pos).pos.y,'bo')
                end
            end        % so many retries, drop the packet
        if cdebug, disp(['timeout_rts: node ' num2str(i) ' has retried so many times to transmit RTS']); end
        retransmit(i) = 0;
        pending_id(i) = 0;
        % question: what if there are waiting packets in mac_queue?
        % answer: should send them anyway as if the current packet is done.
        % similar to the the operation when ACK is received
        if ~isempty(mac_queue(i).list)
            % more packets are waiting to be sent
            % newevent.instant = t + turnaround_time; % switch from receive to transmit
            mac_status(i) = 1;
            newevent = mac_queue(i).list(1);
            mac_queue(i).list(1) = [];
            newevent.instant = t + cca_time;    % question: cca_time or other
            newevent.type = 'wait_for_channel';
            newevent.node = i;
            % packet setup is already done in 'send_mac' before put into the mac_queue
            NewEvents = [NewEvents; newevent]; clear newevent;
        else

            % cannot send RTS successfully, reset MAC layer
            mac_status(i) = 0;
        end
        return;
    end
    if adebug, disp(['timeout_rts: node ' num2str(i) ' to retransmit RTS']); end
    % retransmit the RTS
    newevent = event;
    newevent.instant = t + cca_time;    % check channel status
    newevent.type = 'wait_for_channel';
    newevent.node = i;
    NewEvents = [NewEvents; newevent]; clear newevent;
else
%     if pending_id(i) ~= 0 %& ddebug
%         disp(['timeout_rts at node ' num2str(i) ' pending id=' num2str(pending_id(i)) ' does not match the waiting RTS id=' num2str(event.pkt.id)]); 
%     end
end
