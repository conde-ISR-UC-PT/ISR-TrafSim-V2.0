function Pr = log_normal_shadowing(Pt, Gt, Gr, lambdacommu, Lcommu, pathlossExp, std_db, D0commu, Dcommu)
% log normal shadowing radio propagation model:
% Pr0 = friss(D0commu)
% Pr(db) = Pr0(db) - 10*Nnodes*log(Dcommu/D0commu) + X0
% where X0 is a Gaussian random variable with zero mean and a variance in db
%        Pt * Gt * Gr * (lambdacommu^2)   D0commu^passlossExp    (X0/10)
%  Pr = --------------------------*-----------------*10
%        (4 *pi * D0commu)^2 * Lcommu          Dcommu^passlossExp

% calculate receiving power at reference distance
Pr0 = friis(Pt, Gt, Gr, lambdacommu, Lcommu, D0commu);

% calculate average power loss predicted by path loss model
avg_db = -10.0 * pathlossExp * log10(Dcommu/D0commu);

% get power loss by adding a log-normal random variable (shadowing)
% the power loss is relative to that at reference distance D0commu
% question: reset rand does influcence random
rstate = randn('state');
randn('state', Dcommu);
powerLoss_db = avg_db + (randn*std_db+0);  % random('Normal', 0, std_db);
randn('state', rstate);

% calculate the receiving power at distance Dcommu
Pr = Pr0 * 10^(powerLoss_db/10);

return;