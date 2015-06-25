function dummy()

% dbstop at 63 in sim1.m
% dbstop at 77 in sim2.m

%   dbstop at 9 in run.m


dbstop at 47 in commu/action.m % case 'send_phy'
dbstop at 72 in commu/action.m % case 'send_phy_finish'
dbstop at 96 in commu/action.m % case 'recv_phy'
dbstop at 120 in commu/action.m % case 'send_mac'
% dbstop at 144  in commu/action.m % case 'wait_for_channel'
% dbstop at 151 in commu/action.m % case 'backoff_start'
% dbstop at 158 in commu/action.m % case 'backoff'
dbstop at 165 in commu/action.m % case 'timeout_rts'
dbstop at 189 in commu/action.m % case 'timeout_data'
dbstop at 213 in commu/action.m % case 'recv_mac'
dbstop at 237 in commu/action.m % case 'send_net'
dbstop at 251 in commu/action.m % case 'send_net2'
dbstop at 275 in commu/action.m % case 'timeout_rreq'
dbstop at 299 in commu/action.m % case 'recv_net'
dbstop at 323 in commu/action.m % case 'send_app'
dbstop at 331 in commu/action.m % case 'recv_app'

% dbstop at 15 in position_update.m
% dbstop at 83 in position_update.m


%      dbstop at 8 in script_send_net2.m
%    dbstop at 5 in script_recv_app.m
%      dbstop at 7 in script_recv_net.m