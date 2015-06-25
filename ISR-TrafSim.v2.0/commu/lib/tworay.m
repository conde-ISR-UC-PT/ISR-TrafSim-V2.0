function [Pr, crossover_dist] = tworay(Pt, Gt, Gr, lambdacommu, Lcommu, ht, hr, Dcommu)
% if Dcommu < crossover_dist, use Friis free space model
% if Dcommu >= crossover_dist, use two ray model
% Two-ray ground reflection model:
% 	     Pt * Gt * Gr * (ht^2 * hr^2)
%   Pr = ----------------------------
%            Dcommu^4 * Lcommu
% The original equation in Rappaport's book assumes Lcommu = 1.
% To be consistant with the free space equation, Lcommu is added here.

crossover_dist = (4 * pi * ht * hr) / lambdacommu;
if (Dcommu < crossover_dist)
	Pr = friis(Pt, Gt, Gr, lambdacommu, Lcommu, Dcommu);
else
	Pr = Pt * Gt * Gr * (hr * hr * ht * ht) / (Dcommu * Dcommu * Dcommu * Dcommu * Lcommu);
end

return;
