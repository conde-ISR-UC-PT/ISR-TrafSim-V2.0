%--------------------------------------------------------------------------
%                           ISR-TrafSim v2.0
%                        Copyright (C) 2010-2013
%
%--------------------------------------------------------------------------
% This Matlab file is part of the ISR-TrafSim: a Matlab
% library for traffic simulation and pose estimation in Urban environments,
% namely roundabouts and crossroads.
%
% http://www.isr.uc.pt/~conde/isr-trafsim/
%
%-CITATION---------------------------------------------------------------------------
% If you use this software please cite one of the following papers:
% 1) L.C.Bento, R.Parafita, S.Santos and U.Nunes, An Intelligent Traffic Management
% at Intersections legacy mode for vehicles not equipped with V2V and V2I Communications,
% 16th IEEE Int.Conf. Intelligent Transportation Systems, Netherlands, 2013.
% 2) L.C.Bento, R.Parafita and U.Nunes, Inter-vehicle sensor fusion for accurate vehicle
% localization supported by V2V and V2I communications, 15th IEEE Int.Conf. Intelligent
% Transportation Systems, USA, 2012.
% 3) L.C.Bento, R.Parafita and U.Nunes, Intelligent traffic management at intersections
% supported by V2V and V2I communications, 15th IEEE Int.Conf. Intelligent
% Transportation Systems, USA, 2012.
%
%-DESCRIPTION------------------------------------------------------------------------
%
% Update/Analayse occupation grid with vehicles
%
%-USE----------------------------------------------------------------------
%
% isr_tfs_grid(arg)
%
% -> Input(s)
%   » 'arg'     -   Option
%       update      -   Update grid
%       func        -   For each vehicle determining the vehicles
%                       that are close using diferent methods
%       semaphore   -   Write semaphores on grid
%       cells       -   Group the cells by segment
%       compute     -   Checks the cells occuped in each road
%
% -> Output(s)
%   » [x y t newV newW pwr pwf]
%       (x,y,t) -   Pose updated
%       newV    -   Linear velocity updated
%       newW    -   Angular velocity updated
%       pwr     -   Pulses of rear wheel encoder with error
%       pwf     -   Pulses of front wheel encoder with error
%
%-DISCLAIMER-------------------------------------------------------------------------
% This program is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY;
% without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
% You can use this source code without licensing fees only for NON-COMMERCIAL research
% and EDUCATIONAL purposes only.
% You cannot repost this file without prior written permission from the authors.
%
%-AUTHORS------------------------------------------------------------------
%   Urbano Nunes*
%   Luis Conde Bento**
%   Ricardo Parafita*
%   Sergio Santos*
%
%  *Institute of Systems and Robotics   - University of Coimbra
% **School of Technology and Management - Polytechnic Institute of Leiria
%--------------------------------------------------------------------------

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Selects the option desired and call the function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [out] = grid_isr_trafsim(arg)
out=0;
[~, c]=size(arg);
if(c==6)        % 'update'
    out=grid_update();
elseif(c==4)    % 'func'
    grid_analyze();
elseif(c==9)    % 'semaphore'
    grid_sema();
elseif(c==5)    % 'cells'
    grid_init();
elseif(c==7)    % 'compute'
    grid_compute();
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Checks the cells occupied in each road
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = grid_compute()
global s

tot=round2(s.time,0.001);
for i = 1 : s.rn
    
    [~,c,~]=size(s.ocmap_ind);
    count=0;
    for j=1:c
        if(s.ocmap_ind(1,j,i)~=0 && s.ocmap_ind(1,j,i)~=0)
            if(s.ocmap(s.ocmap_ind(1,j,i),s.ocmap_ind(2,j,i))~=0 )
                count=count+1;
            end
        end
    end
    
    tot=[tot;count];
end
s.ocmap_time=[s.ocmap_time tot];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Group the cells by road
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = grid_init()
global s

% Grouping the pixels that is worth checking around
if (size(s.ocmap,1) == 100) && (size(s.ocmap,2)==160) && (s.occup_res==1)
    load group_cell_by_road.mat
else
    cont=0;
    for j = 1 : size(s.ocmap,2)
        for i = 1 : size(s.ocmap,1)
            cont=cont+1;
            pointsX(1,cont)=(j-1)*(1/s.occup_res)+(1/(2*s.occup_res));
            coordiX(1,cont)=j;
            pointsY(1,cont)=(i-1)*(1/s.occup_res)+(1/(2*s.occup_res));
            coordiY(1,cont)=i;
        end
    end
end

% For each road group the indices of cells
s.ocmap_ind=[];
for i = 1 : s.rn
    x1=s.ri(i,1); y1=s.ri(i,2); x2=s.ri(i,3); y2=s.ri(i,4); a=s.ri(i,7); b=s.rw/2; d=(pi/2);
    xbox=[ x1+b*cos(a+d) x1+b*cos(a-d) x2+b*cos(a-d) x2+b*cos(a+d)  x1+b*cos(a+d)];
    ybox=[ y1+b*sin(a+d) y1+b*sin(a-d) y2+b*sin(a-d) y2+b*sin(a+d)  y1+b*sin(a+d)];
    
    % Checks which the pixels that are inside the polygon
    in = inpolygon(pointsX(1,:),pointsY(1,:),xbox,ybox);
    
    % Store the cels wich belongs to the select road
    cels=length(in); aux=[];
    
    for j=find(in(1,:)==1)
        aux=[aux [coordiY(j) ; coordiX(j) ]];
    end
    
    s.ocmap_ind(1,[1:size(aux,2)],i)=aux(1,:);
    s.ocmap_ind(2,[1:size(aux,2)],i)=aux(2,:);
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Determining the cars that are close
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = grid_analyze()
global c s

if(s.zone_type==4)
    ncars=length(c.listactive);
    for i=1:ncars
        c.car(i).list=[];
        for j=1:ncars
            if(i~=j)
                dd=dist_isr_trafsim(c.car(i).pos.x,c.car(i).pos.y,c.car(j).pos.x,c.car(j).pos.y);
                if(dd<s.circle_r)
                    if(isempty(intersect(c.car(i).list,c.car(j).id)))
                        c.car(i).list=[c.car(i).list c.car(j).id];
                    end
                end
            end
        end
        
        for j=1:length(s.stoplist)
            
            dd=dist_isr_trafsim(c.car(i).pos.x,c.car(i).pos.y,s.ri(s.stoplist(j),3),s.ri(s.stoplist(j),4));
            if(dd<s.circle_r)
                if(isempty(intersect(c.car(i).list,s.stoplist(j))))
                    c.car(i).list=[c.car(i).list s.stoplist(j)];
                end
            end
        end
    end
else
    ncars=length(c.listactive);
    
    for n=1:ncars
        
        % Rotate points of area wich will be analyzed
        if(s.zone_type==1 || s.zone_type==3)
            % Rotation matrix
            m_rot=[cos(c.car(n).pos.t) -sin(c.car(n).pos.t) 0; sin(c.car(n).pos.t) cos(c.car(n).pos.t) 0; 0 0 1];
            % Rotate points
            p=[];
            for i=1:s.npoints
                p=[p m_rot*[s.xg(i);s.yg(i);1]];
            end
            % Add offset
            p(1,:)=p(1,:)+c.car(n).pos.x;
            p(2,:)=p(2,:)+c.car(n).pos.y;
        elseif(s.zone_type==2)
            % Add offset
            p(1,:)=s.xg(1,:)+c.car(n).pos.x;
            p(2,:)=s.yg(1,:)+c.car(n).pos.y;
        end
        
        % Pixels of mass center
        % Calculates the pixels which correspond to the points of the mass center
        px0=floor( c.car(n).pos.x*s.occup_res )+1;
        py0=floor( c.car(n).pos.y*s.occup_res )+1;
        
        % To store points and pixels
        cels=s.lmax*s.occup_res; cont=0;
        pointsX=zeros(1,(cels*cels)); coordiX=zeros(1,(cels*cels)); pointsY=zeros(1,(cels*cels)); coordiY=zeros(1,(cels*cels));
        
        % Grouping the pixels that is worth checking around
        for j = round(px0-cels) : round(px0+cels)
            for i = round(py0-cels) : round(py0+cels)
                if(j>0 && j<=s.dimx*s.occup_res && i>0 && i<=s.dimy*s.occup_res)
                    cont=cont+1;
                    pointsX(1,cont)=(j-1)*(1/s.occup_res)+(1/(2*s.occup_res));
                    coordiX(1,cont)=j;
                    pointsY(1,cont)=(i-1)*(1/s.occup_res)+(1/(2*s.occup_res));
                    coordiY(1,cont)=i;
                end
            end
        end
        
        % Checks which the pixels that are within the polygon
        in = inpolygon(pointsX(1,[1:cont]),pointsY(1,[1:cont]),p(1,:),p(2,:));
        
        % Checks for cars within the zone of interest
        cels=length(in);
        list=[];
        for i=1:cels
            if(in(1,i)==1)
                aux=s.ocmap(coordiY(i),coordiX(i));
                if(aux~=0 && aux~=c.car(n).id)
                    if(isempty(find(list==aux, 1)))
                        list=[list aux];
                    end
                end
            end
        end
        c.car(n).list=list;
    end
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Updates the map of occupation with car ID's
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [out] = grid_update()
global s c
out=[];

% To display paths velocity
flag=0;
if(s.disp_res2)
    if(mod(round2(s.time,0.0001),(1/s.fres2))==0)
        flag=1;
        mapaux=zeros(s.dimy*s.occup_res,s.dimx*s.occup_res);
    end
end

% Clear the map
s.ocmap=uint8(zeros(s.dimy*s.occup_res,s.dimx*s.occup_res));

% Update the map
ncars=length(c.listactive);
for n=1:ncars
    % Define polygon points
    xbox=[c.car(n).cornX c.car(n).cornX(1)]+c.car(n).pos.x;
    ybox=[c.car(n).cornY c.car(n).cornY(1)]+c.car(n).pos.y;
    
    mask = poly2mask(xbox*s.occup_res,ybox*s.occup_res,size(s.ocmap,1),size(s.ocmap,2));
    maskid=uint8(mask.*c.car(n).id);
    if(c.car(n).dyn(1)>0)
        maskdyn=double(mask).*c.car(n).dyn(1);
    else
        maskdyn=double(mask).*0.1;
    end
    if(logical(nnz(s.ocmap(:,:) & mask)))   % If exist colision
        disp(['Colision of car ' num2str(c.car(n).id) ' at ' num2str(s.time) ' (s)'])
        keyboard
    end
    
    s.ocmap=s.ocmap+maskid;
    if(flag)
        mapaux=mapaux+maskdyn;
    end
end

% To display paths velocity
if(flag)
    [lin, col, z]=size(s.ocmap4);
    if(lin==0 && col==0)
        s.ocmap4(:,:,1)=mapaux;
    else
        s.ocmap4(:,:,z+1)=mapaux;
    end
end

% To print stop on ocmap
if(isempty(s.stoplist)==0)
    for i=1:length(s.stoplist)
        road=s.stoplist(1,i);
        rx=s.ri(road,3);
        ry=s.ri(road,4);
        
        % Calculates the pixels which correspond to the point of roadend
        px0=floor( rx*s.occup_res )+1;
        if(px0>s.dimx*s.occup_res)
            px0=s.dimx*s.occup_res;
        end
        py0=floor( ry*s.occup_res )+1;
        if(py0>s.dimy*s.occup_res)
            py0=s.dimy*s.occup_res;
        end
        s.ocmap(py0,px0)=road;
    end
end
end