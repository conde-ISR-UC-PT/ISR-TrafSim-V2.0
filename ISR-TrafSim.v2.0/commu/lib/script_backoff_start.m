%script_backoff_start
t = event.instant;
i = event.node;
j = event.pkt.rv;
if bdebug, disp(['backoff_start @ node ' num2str(i)]); end
if node(i, 4) == 0 & carrier_sense(i) == 0
    % the node is still idle and the channel is free, start backoff
    % question: what if the channel was busy during this DIFS period?
    backoff_attempt(i) = 0;
    temp = min(backoff_attempt(i)+CW_min,CW_max);
    backoff_counter(i) = floor((2^temp-1)*rand);
    newevent = event;
    newevent.instant = t + slot_time;
    newevent.type = 'backoff';
    newevent.node = i;
    NewEvents = [NewEvents; newevent]; clear newevent;
else
    % channel becomes busy during DIFS, wait until the channel is free
    newevent = event;
    newevent.instant = t + cca_time;
    newevent.type = 'wait_for_channel';
    newevent.node = i;
    NewEvents = [NewEvents; newevent]; clear newevent;
end
