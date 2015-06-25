function [Dcommu] = inv_recv_power(rmodel, Pt, Gt, Gr, lambdacommu, Lcommu, ht, hr, pathLossExp, D0commu, Pr)
% Dcommu = inv_recv_power('shadowing', Pt, Gt, Gr, lambdacommu, Lcommu, ht, hr, pathLossExp, D0commu, 10^(SNR/10)*white_noise_variance)
% Given received power Pr, or equally SNR, find the corresponding distance Dcommu.

switch rmodel
    case 'friis'
        %        Pt * Gt * Gr * (lambdacommu^2)
        %  Pr = --------------------------
        %        (4 *pi * Dcommu)^2 * Lcommu
        Dcommu = sqrt(Pt*Gt*Gr*lambdacommu^2/Pr/Lcommu)/4/pi;
    case 'tworay'
        % if Dcommu < crossover_dist, use Friis free space model
        % if Dcommu >= crossover_dist, use two ray model
        % 	     Pt * Gt * Gr * (ht^2 * hr^2)
        %   Pr = ----------------------------
        %            Dcommu^4 * Lcommu
        crossover_dist = (4 * pi * ht * hr) / lambdacommu;
        Dcommu = sqrt(Pt*Gt*Gr*lambdacommu^2/Pr/Lcommu)/4/pi;
        if (Dcommu > crossover_dist)
            Dcommu = (Pt*Gt*Gr*(hr*hr*ht*ht)/Pr/Lcommu)^(1/4);
        end
    case 'shadowing'
        % Pr0 = friss(D0commu)
        % Pr(db) = Pr0(db) - 10*Nnodes*log(Dcommu/D0commu) + X0
        % where X0 is a Gaussian random variable with zero mean and a variance in db
        %        Pt * Gt * Gr * (lambdacommu^2)   D0commu^pathLossExp    (X0/10)
        %  Pr = --------------------------*-----------------*10
        %        (4 *pi * D0commu)^2 * Lcommu          Dcommu^pathLossExp
        % Assume X0=0
        Dcommu = (Pt*Gt*Gr*lambdacommu*lambdacommu/(4*pi*D0commu)^2/Lcommu*D0commu^pathLossExp/Pr)^(1/pathLossExp);
end

return