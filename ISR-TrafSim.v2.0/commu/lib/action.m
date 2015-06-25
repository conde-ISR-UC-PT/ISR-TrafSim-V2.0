function [NewEvents] = action(event, log_file)

% debug:
% TTL is determined at network layer only
% one way transmission received, reverse transmission fails, why?
% mainly because randomness in the 'shadowing' model: solved...same distance generates same random number
% friis model works fine
% tworay model is mostly the same as friis because crossover distance = 100 m
% freq should be 2.4G instead of 2.4M, white_noise_variance is changed correspondingly
% Duplicated packets due to early time out and retransmission: this is ok, just provide a larger timeout
% queue waiting problem: net_queue is actually always empty, all NET packets stack at mac_queue
% mac_queue is accompanied by mac_status
% Question 1: how to decide the priority of broadcast, unicast transmission and receptioin?

% to add virtual carrier sense: done
% to add RTS-CTS: done
% to add ad hoc routing: done
% to add application layer actions:
% to add figures or animations to show network and traffic change: not critical

global Gt Gr freq Lcommu ht hr pathLossExp std_db D0commu rmodel s;
global cs_threshold white_noise_variance rv_threshold rv_threshold_delta;
global slot_time CW_min CW_max turnaround_time max_retries SIFS DIFS cca_time basic_rate default_power;
global max_size_mac_body size_mac_header size_rts size_cts size_ack size_plcp;
global Nnodes node Event_list;
global packet_id retransmit pending_id mac_queue backoff_attmpt backoff_counter backoff_attempt;
global navcommu;
global ack_tx_time cts_tx_time rts_tx_time;
global default_rate default_ttl;
global size_rreq size_rrep;
global rreq_timeout net_queue net_pending net_max_retries;
global rrep_table;  % id, route, metric
global bcast_table;
global mac_status;
global adebug bdebug cdebug ddebug edebug fdebug;
global rreq_out rreq_in rreq_forward;
global rreq_out_crosslayer rreq_in_crosslayer rreq_forward_crosslayer;
global rrep_out rrep_in rrep_forward;
global rrep_out_crosslayer rrep_in_crosslayer rrep_forward_crosslayer rrep_destination_crosslayer;
global size_Event_list

NewEvents = [];
testtime=0.005;
str_aux3='    '; str_aux4='    ';
show=s.otashow;

switch event.type
    case 'send_phy'
        script_send_phy;
        if(show==1)
            if (event.pkt.rv~=0)
                h1=plot([node(event.node,1) node(event.pkt.rv,1)], [node(event.node,2) node(event.pkt.rv,2)]);
                str_aux1=' - P2P       ';
            else
                str_aux1= ' - BROADCAST ';
            end
            str_aux2 = 'send PHY ';
            str=[str_aux2 str_aux1];
            h2=text(node(event.node,1),node(event.node,2)+2,str, 'BackgroundColor',[.5 .9 .5]);
            if isfield(event.pkt,'type')
                str_aux3(1:size(event.pkt.type,2))=event.pkt.type;
            else
                str_aux3='NaN ';
            end
            if isfield(event.net,'type')
                str_aux4(1:size(event.net.type,2))=event.net.type;
            else
                str_aux4='NaN ';
            end
            if edebug, disp([str num2str(event.node, '%02d') ' -> ' num2str(event.app.id3, '%02d') ' PKT: ' str_aux3 ' NET: ' str_aux4 ' SEL: ' num2str(size_Event_list(1),'%03d') ' T: ' num2str(event.instant,'%3.5f')]); end
            pause(testtime*50);
        end
        
    case 'send_phy_finish'
        script_send_phy_finish;
        if(show==1)
            if (event.pkt.rv~=0)
                h1=plot([node(event.node,1) node(event.pkt.rv,1)], [node(event.node,2) node(event.pkt.rv,2)]);
                str_aux1=' - P2P       ';
            else
                str_aux1= ' - BROADCAST ';
            end
            str_aux2 = 'send PhyF';
            str=[str_aux2 str_aux1];
            h2=text(node(event.node,1),node(event.node,2)+2,str, 'BackgroundColor',[.5 .9 .5]);
            if isfield(event.pkt,'type')
                str_aux3(1:size(event.pkt.type,2))=event.pkt.type;
            else
                str_aux3='NaN ';
            end
            if isfield(event.net,'type')
                str_aux4(1:size(event.net.type,2))=event.net.type;
            else
                str_aux4='NaN ';
            end
            if edebug, disp([str num2str(event.node, '%02d') ' -> ' num2str(event.app.id3, '%02d') ' PKT: ' str_aux3 ' NET: ' str_aux4  ' SEL: ' num2str(size_Event_list(1),'%03d')  ' T: ' num2str(event.instant,'%3.5f')]); end
            pause(testtime*50);
        end
    case 'recv_phy'
        script_recv_phy;
        if(show==1)
            if (event.pkt.tx~=0)
                h1=plot([node(event.node,1) node(event.pkt.tx,1)], [node(event.node,2) node(event.pkt.tx,2)]);
                str_aux1=' - P2P       ';
            else
                str_aux1= ' - BROADCAST ';
            end
            str_aux2 = 'recv PHY ';
            str=[str_aux2 str_aux1];
            h2=text(node(event.node,1),node(event.node,2)+2,str, 'BackgroundColor',[.0 1.0 .0]);
            if isfield(event.pkt,'type')
                str_aux3(1:size(event.pkt.type,2))=event.pkt.type;
            else
                str_aux3='NaN ';
            end
            if isfield(event.net,'type')
                str_aux4(1:size(event.net.type,2))=event.net.type;
            else
                str_aux4='NaN ';
            end
            if edebug, disp([str num2str(event.pkt.tx, '%02d') ' -> ' num2str(event.node, '%02d')  ' PKT: ' str_aux3 ' NET: ' str_aux4 ' SEL: ' num2str(size_Event_list(1),'%03d')  ' T: ' num2str(event.instant,'%3.5f')]); end
            pause(testtime*50);
        end
    case 'send_mac'
        %dbstop at 125 in action.m
        script_send_mac;
        if(show==1)
            if (event.pkt.rv~=0)
                h1=plot([node(event.node,1) node(event.pkt.rv,1)], [node(event.node,2) node(event.pkt.rv,2)]);
                str_aux1=' - P2P       ';
            else
                str_aux1= ' - BROADCAST ';
            end
            str_aux2 = 'send MAC ';
            str=[str_aux2 str_aux1];
            h2=text(node(event.node,1),node(event.node,2)+2,str, 'BackgroundColor',[.9 .5 .5]);
            if isfield(event.pkt,'type')
                str_aux3(1:size(event.pkt.type,2))=event.pkt.type;
            else
                str_aux3='NaN ';
            end
            if isfield(event.net,'type')
                str_aux4(1:size(event.net.type,2))=event.net.type;
            else
                str_aux4='NaN ';
            end
            if edebug, disp([str num2str(event.node, '%02d') ' -> ' num2str(event.app.id3, '%02d')  ' PKT: ' str_aux3 ' NET: ' str_aux4 ' SEL: ' num2str(size_Event_list(1),'%03d')  ' T: ' num2str(event.instant,'%3.5f')]); end
            pause(testtime*50);
        end
    case 'wait_for_channel'
        script_wait_for_channel;
        if(show==1)
            str_aux2 = 'WAIT for channel';
            str=[str_aux2 ];
            h2=text(node(event.node,1),node(event.node,2)+2,str, 'BackgroundColor',[.5 .5 .5]);
            pause(testtime);
        end
    case 'backoff_start'  % after DIFS, start backoff
        script_backoff_start;
        if(show==1)
            str_aux2 = 'BACKOFF start';
            str=[str_aux2 ];
            h2=text(node(event.node,1),node(event.node,2)+2,str, 'BackgroundColor',[.5 .5 .5]);
            pause(testtime);
        end
    case 'backoff'
        script_backoff;
        if(show==1)
            str_aux2 = 'BACKOFF';
            str=[str_aux2 ];
            h2=text(node(event.node,1),node(event.node,2)+2,str, 'BackgroundColor',[.5 .5 .5]);
            pause(testtime);
        end
    case 'timeout_rts'
        script_timeout_rts;
        if(show==1)
            if (event.pkt.rv~=0)
                h1=plot([node(event.node,1) node(event.pkt.rv,1)], [node(event.node,2) node(event.pkt.rv,2)]);
                str_aux1=' - P2P       ';
            else
                str_aux1= ' - BROADCAST ';
            end
            str_aux2 = 'Tout RTS ';
            str=[str_aux2 str_aux1];
            h2=text(node(event.node,1),node(event.node,2)+2,str, 'BackgroundColor',[.5 .5 .0]);
            if isfield(event.pkt,'type')
                str_aux3(1:size(event.pkt.type,2))=event.pkt.type;
            else
                str_aux3='NaN ';
            end
            if isfield(event.net,'type')
                str_aux4(1:size(event.net.type,2))=event.net.type;
            else
                str_aux4='NaN ';
            end
            if edebug, disp([str num2str(event.node, '%02d') ' -> ' num2str(event.app.id3, '%02d')  ' PKT: ' str_aux3 ' NET: ' str_aux4 ' SEL: ' num2str(size_Event_list(1),'%03d')  ' T: ' num2str(event.instant,'%3.5f')]); end
            pause(testtime*100);
        end
    case 'timeout_data'
        script_timeout_data;
        if(show==1)
            if (event.pkt.rv~=0)
                h1=plot([node(event.node,1) node(event.pkt.rv,1)], [node(event.node,2) node(event.pkt.rv,2)]);
                str_aux1=' - P2P       ';
            else
                str_aux1= ' - BROADCAST ';
            end
            str_aux2 = 'Tout DATA';
            str=[str_aux2 str_aux1];
            h2=text(node(event.node,1),node(event.node,2)+2,str, 'BackgroundColor',[.5 .5 .0]);
            if isfield(event.pkt,'type')
                str_aux3(1:size(event.pkt.type,2))=event.pkt.type;
            else
                str_aux3='NaN ';
            end
            if isfield(event.net,'type')
                str_aux4(1:size(event.net.type,2))=event.net.type;
            else
                str_aux4='NaN ';
            end
            if edebug, disp([str num2str(event.node, '%02d') ' -> ' num2str(event.node, '%02d')  ' PKT: ' str_aux3 ' NET: ' str_aux4 ' SEL: ' num2str(size_Event_list(1),'%03d')  ' T: ' num2str(event.instant,'%3.5f')]); end
            pause(testtime*100);
        end
    case 'recv_mac'
        script_recv_mac;
        if(show==1)
            if (event.pkt.tx~=0)
                h1=plot([node(event.node,1) node(event.pkt.tx,1)], [node(event.node,2) node(event.pkt.tx,2)]);
                str_aux1=' - P2P       ';
            else
                str_aux1= ' - BROADCAST ';
            end
            str_aux2 = 'Recv MAC ';
            str=[str_aux2 str_aux1];
            h2=text(node(event.node,1),node(event.node,2)+2,str, 'BackgroundColor',[1.0 .0 .0]);
            if isfield(event.pkt,'type')
                str_aux3(1:size(event.pkt.type,2))=event.pkt.type;
            else
                str_aux3='NaN ';
            end
            if isfield(event.net,'type')
                str_aux4(1:size(event.net.type,2))=event.net.type;
            else
                str_aux4='NaN ';
            end
            if edebug, disp([str num2str(event.pkt.tx, '%02d') ' -> ' num2str(event.node, '%02d')  ' PKT: ' str_aux3 ' NET: ' str_aux4 ' SEL: ' num2str(size_Event_list(1),'%03d')  ' T: ' num2str(event.instant,'%3.5f')]); end
            pause(testtime*50);
        end
    case 'send_net'
        script_send_net;
        %         if (event.net.dst~=0)
        %             h1=plot([node(event.node,1) node(event.net.dst,1)], [node(event.node,2) node(event.net.dst,2)]);
        %             str_aux1=' - P2P       ';
        %         else
        %             str_aux1= ' - BROADCAST ';
        %         end
        %         str_aux2 = 'send NET ';
        %         str=[str_aux2 str_aux1];
        %         h2=text(node(event.node,1),node(event.node,2)+2,str, 'BackgroundColor',[.5 .5 .9]);
        %         %             if edebug, disp([str num2str(event.node, '%02d') ' -> ' num2str(event.net.dst, '%02d') ' SEL: ' num2str(size_Event_list(1),'%03d')  ' T: ' num2str(event.instant,'%3.5f')]); end
        %         pause(0.5);
        
    case 'send_net2'
        script_send_net2;
        if(show==1)
            if (event.net.dst~=0)
                h1=plot([node(event.node,1) node(event.net.dst,1)], [node(event.node,2) node(event.net.dst,2)]);
                str_aux1=' - P2P       ';
            else
                str_aux1= ' - BROADCAST ';
            end
            str_aux2 = 'send NET2';
            str=[str_aux2 str_aux1];
            h2=text(node(event.node,1),node(event.node,2)+2,str, 'BackgroundColor',[.5 .5 .9]);
            if isfield(event.pkt,'type')
                str_aux3(1:size(event.pkt.type,2))=event.pkt.type;
            else
                str_aux3='NaN ';
            end
            if isfield(event.net,'type')
                str_aux4(1:size(event.net.type,2))=event.net.type;
            else
                str_aux4='NaN ';
            end
            if edebug, disp([str num2str(event.node, '%02d') ' -> ' num2str(event.net.dst, '%02d')  ' PKT: ' str_aux3 ' NET: ' str_aux4 ' SEL: ' 'NaN'  ' T: ' num2str(event.instant,'%3.5f')]); end
            pause(testtime*50);
        end
    case 'timeout_rreq'
        script_timeout_rreq;
        if(show==1)
            if (event.net.dst~=0)
                h1=plot([node(event.node,1) node(event.net.dst,1)], [node(event.node,2) node(event.net.dst,2)]);
                str_aux1=' - P2P       ';
            else
                str_aux1= ' - BROADCAST ';
            end
            str_aux2 = 'Tout RREQ';
            str=[str_aux2 str_aux1];
            h2=text(node(event.node,1),node(event.node,2)+2,str, 'BackgroundColor',[.5 .5 .0]);
            if isfield(event.pkt,'type')
                str_aux3(1:size(event.pkt.type,2))=event.pkt.type;
            else
                str_aux3='NaN ';
            end
            if isfield(event.net,'type')
                str_aux4(1:size(event.net.type,2))=event.net.type;
            else
                str_aux4='NaN ';
            end
            if edebug, disp([str num2str(event.node, '%02d') ' -> ' num2str(event.net.dst, '%02d')  ' PKT: ' str_aux3 ' NET: ' str_aux4 ' SEL: ' num2str(size_Event_list(1),'%03d')  ' T: ' num2str(event.instant,'%3.5f')]); end
            pause(testtime*100);
        end
    case 'recv_net'
        script_recv_net;
        if(show==1)
            if (event.net.src~=0)
                h1=plot([node(event.node,1) node(event.net.src,1)], [node(event.node,2) node(event.net.src,2)]);
                str_aux1=' - P2P       ';
            else
                str_aux1= ' - BROADCAST ';
            end
            str_aux2 = 'recv NET ';
            str=[str_aux2 str_aux1];
            h2=text(node(event.node,1),node(event.node,2)+2,str, 'BackgroundColor',[.0 .0 1.0]);
            if isfield(event.pkt,'type')
                str_aux3(1:size(event.pkt.type,2))=event.pkt.type;
            else
                str_aux3='NaN ';
            end
            if isfield(event.net,'type')
                str_aux4(1:size(event.net.type,2))=event.net.type;
            else
                str_aux4='NaN ';
            end
            if edebug, disp([str num2str(event.net.src, '%02d') ' -> ' num2str(event.node, '%02d')  ' PKT: ' str_aux3 ' NET: ' str_aux4 ' SEL: ' num2str(size_Event_list(1),'%03d')  ' T: ' num2str(event.instant,'%3.5f')]); end
            pause(testtime*50);
        end
    case 'send_app'
        script_send_app;
        if(show==1)
            str_aux2 = 'send APP';
            str=[str_aux2 ];
            h2=text(node(event.node,1),node(event.node,2)+2,str, 'BackgroundColor',[.7 .0 .7]);
            %             if edebug, disp([str num2str(event.node, '%02d') ' -> ' num2str(event.node, '%02d') ' SEL: ' num2str(size_Event_list(1),'%03d')  ' T: ' num2str(event.instant,'%3.5f')]); end
            pause(testtime*150);
        end
    case 'recv_app'
        script_recv_app;
        if(show==1)
            str_aux2 = 'recv APP';
            str=[str_aux2 ];
            h2=text(node(event.node,1),node(event.node,2)+2,str, 'BackgroundColor',[.7 .0 .7]);
            %             if edebug, disp([str num2str(event.node, '%02d') ' -> ' num2str(event.node, '%02d') ' SEL: ' num2str(size_Event_list(1),'%03d')  ' T: ' num2str(event.instant,'%3.5f')]); end
            pause(testtime*150);
        end
    otherwise
        disp(['action: Undefined event type: ' event.type]);
end;
if exist('h1')
    delete(h1);
    clear h1
end
if exist('h2')
    delete(h2);
    clear h2
end

return;