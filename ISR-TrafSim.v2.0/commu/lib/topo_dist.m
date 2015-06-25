function [Dcommu] = topo_dist(i, j);
% return distance between two nodes i and j

global Nnodes node;

Dcommu = 0;
if i<=0 | i>Nnodes, return; end
if j<=0 | j>Nnodes, return; end
Dcommu = sqrt((node(i, 1) - node(j, 1))^2 + (node(i, 2) - node(j, 2))^2);

return;
