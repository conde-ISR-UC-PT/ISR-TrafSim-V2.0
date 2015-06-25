%script_send_net;
% event provides net.dst, net.src, net.size
t = event.instant;
i = event.node;
j = event.net.dst;
if adebug, disp(['send_net @ node ' num2str(i)]); end
% net_queue
% Original
if ~isempty(net_queue(i).list)
    % this is redundant; net_queue is always empty
    % old packets are waiting to be sent, just wait behind them
    % if cdebug, disp(['time ' num2str(t) ' node ' num2str(i) ' queue a packet to node ' num2str(j)]); end
    net_queue(i).list = [net_queue(i).list event];
else
    newevent = event;
    newevent.type = 'send_net2';
    NewEvents = [NewEvents; newevent]; clear newevent;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%mod
% if ~isempty(net_queue)
%     if ~isempty(net_queue(i).list)
%         % this is redundant; net_queue is always empty
%         % old packets are waiting to be sent, just wait behind them
%         % if cdebug, disp(['time ' num2str(t) ' node ' num2str(i) ' queue a packet to node ' num2str(j)]); end
%         net_queue(i).list = [net_queue(i).list event];
%     end
% else
%     newevent = event;
%     newevent.type = 'send_net2';
%     NewEvents = [NewEvents; newevent]; clear newevent;
% end