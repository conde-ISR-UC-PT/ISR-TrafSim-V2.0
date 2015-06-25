function [Pr] = recv_power(tx, rv, rmodel)
% send packet at PHY layer
%dbstop at 4 in recv_power.m
global node Gt Gr freq Lcommu ht hr pathLossExp std_db D0commu;
global cs_threshold;

lambdacommu = 3e8 / freq;
Pt = node(tx, 3);
if Pt <= 0
    disp(['In function recv_power: the transmission power of node ' num2str(tx) ' is zero']);
end
% Pt

% Update the position before calculating distance and received power
%position_update;

Dcommu = sqrt((node(tx, 1)-node(rv, 1))^2+(node(tx, 2)-node(rv, 2))^2);

switch rmodel
    case 'friis'
        Pr = friis(Pt, Gt, Gr, lambdacommu, Lcommu, Dcommu);
    case 'tworay'
        [Pr, crossover_dist] = tworay(Pt, Gt, Gr, lambdacommu, Lcommu, ht, hr, Dcommu);
    case 'shadowing'
        Pr = log_normal_shadowing(Pt, Gt, Gr, lambdacommu, Lcommu, pathLossExp, std_db, D0commu, Dcommu);
end

% if Pr <= cs_threshold
%     Pr = 0;
% end

return;
