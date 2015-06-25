function Pr = friis(Pt, Gt, Gr, lambdacommu, Lcommu, Dcommu)

% Friis free space propagation model:
%        Pt * Gt * Gr * (lambdacommu^2)
%  Pr = --------------------------
%        (4 *pi * Dcommu)^2 * Lcommu

M = lambdacommu / (4 * pi * Dcommu);
Pr = Pt * Gt * Gr * (M * M) / Lcommu;

return;