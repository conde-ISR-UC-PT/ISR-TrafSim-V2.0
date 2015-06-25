function [o] = overlap(a, b, c, Dcommu)

% check if [a, b] and [c, Dcommu] overlap

o = 0;
if (a>c & a<Dcommu), o = 1; return; end
if (b>c & b<Dcommu), o = 1; return; end
if (c>a & c<b), o = 1; return; end
if (Dcommu>a & Dcommu<b), o = 1; return; end

return;    