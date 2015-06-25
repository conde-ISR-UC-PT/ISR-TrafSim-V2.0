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
%-DESCRIPTION--------------------------------------------------------------
%
% Print all parts of scenario simulated (lanes,roundabout,etc...)
%
%-DISCLAIMER---------------------------------------------------------------
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

function [] = printset_isr_trafsim()
global s gps_data

roads();        % Define trajectory setpoints
roundabout();   % Define trajectory setpoints and print lanes
proads();       % Print roads setpoints
pcross();       % Print cross

% Add the cross center point
[~, col,~]=size(s.r);
s.r(:,col+1,:)=0;
for i=1:s.rn
    if(s.ri(i,9)==3)
        s.r(i,col+1,1)=116;
        s.r(i,col+1,2)=50;
        s.ri(i,5)=s.ri(i,5)+1;
    end
end

% Divide roundabout setpoints in parts
% Outside lane (rexit 30 32 34 36)
[lin,~]=size(s.r1);
cont=30;
setp=1;
for i=1:lin
    s.r(cont,setp,1)=s.r1(i,1);
    s.r(cont,setp,2)=s.r1(i,2);
    if(mod(s.r1(i,3),90)==0 && s.r1(i,3)~=0)
        cont=cont+2;
        setp=0;
    end
    setp=setp+1;
end

% Inside lane (rexit 31 33 35 37)
[lin ,~]=size(s.r2);
cont=31;
setp=1;
for i=1:lin
    s.r(cont,setp,1)=s.r2(i,1);
    s.r(cont,setp,2)=s.r2(i,2);
    if(mod(s.r2(i,3),90)==0 && s.r2(i,3)~=0)
        cont=cont+2;
        setp=0;
    end
    setp=setp+1;
end

% Plot GPS fixed station
if(s.gps_type==1)
    
    pos=xyz2enu(gps_data.xyz_origin_ref,s.xyz_origin);
    posx=pos(1);
    posy=pos(2);
    s.refposxRound=posx;
    s.refposyRound=posy;
else
    posx=s.round_center(1);
    posy=s.round_center(2);
    s.refposxRound=posx;
    s.refposyRound=posy;
end

if(s.mode==1)
    x=[]; y=[]; for ang=0:0.1:2*pi, x=[x posx+0.5*cos(ang)]; y=[y posy+0.5*sin(ang)];   end
    fill(x,y,'k')
    
    x=[]; y=[]; for ang=0:0.1:2*pi, x=[x posx+1*cos(ang)]; y=[y posy+1*sin(ang)];   end
    plot(x,y,'k')
    
    x=[]; y=[]; for ang=0:0.1:2*pi, x=[x posx+2*cos(ang)]; y=[y posy+2*sin(ang)];   end
    plot(x,y,'k')
    
    x=[]; y=[]; for ang=0:0.1:2*pi, x=[x posx+3*cos(ang)]; y=[y posy+3*sin(ang)];   end
    plot(x,y,'--k')
    
    x=[]; y=[]; for ang=0:0.1:2*pi, x=[x posx+4*cos(ang)]; y=[y posy+4*sin(ang)];   end
    plot(x,y,'--k')
end

% Define and print cross zone if needed
aux=s.distcrossextra;
aa=110-aux;
bb=122+aux;
cc=56+aux;
dd=44-aux;
s.crosszone=[aa bb cc dd];
if(s.printcrosszone==1)
    plot([aa bb bb aa aa],[cc cc dd dd cc],'--g');
end

% Print area controled by gestion system
if(s.round_gest==1 && s.mode==1)
    x=[]; y=[]; for ang=0:0.05:2*pi, x=[x 50+s.control_radius_round*cos(ang)]; y=[y 50+s.control_radius_round*sin(ang)];end
    if(s.make_video==0)
        plot(x,y,'--g')
    end
end

% Print area controled by gestion system
if(s.cross_mode==2 && s.mode==1)
    x=[]; y=[]; for ang=0:0.05:2*pi, x=[x 115+s.control_radius_cross*cos(ang)]; y=[y 50+s.control_radius_cross*sin(ang)];end
    if(s.make_video==0)
        plot(x,y,'--g')
    end
end

% Print oil in the roads
if(s.fus4>=1 && s.printslipplace==1)
    
    for i=1:size(s.slip_list,1)
        x_c=[]; y_c=[];
        for ang=0:0.3:2*pi
            x_c=[x_c s.slip_list(i,1)+s.slip_radius*cos(ang)];
            y_c=[y_c s.slip_list(i,2)+s.slip_radius*sin(ang)];
        end
        fill(x_c,y_c,[0.5 0.25 0])
    end
end
drawnow
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Print cross center
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = pcross()
global  s

% Print center of cross
a=s.distcrossextra;
b=a/2;
if(s.mode==1)
    fill([122+a 122+a 110-a 110-a],[44 56 56 44],'w','EdgeColor','w')
    fill([122 122 110 110],[44-a 56+a 56+a 44-a],'w','EdgeColor','w')
    fill([122 122 122+a],[56 56+a 56],'w','EdgeColor','w')
    fill([122 122 122+a],[44 44-a 44],'w','EdgeColor','w')
    fill([110 110 110-a],[56 56+a 56],'w','EdgeColor','w')
    fill([110 110 110-a],[44 44-a 44],'w','EdgeColor','w')
    %         plot([95 95],[30 70],'g--');
    %         plot([95 135],[70 70],'g--');
    %         plot([135 135],[70 30],'g--');
    %         plot([135 95],[30 30],'g--');
    plot([122+b 110-b],[56+b 56+b],'k--','LineWidth',2);
    plot([122+b 110-b],[44-b 44-b],'k--','LineWidth',2);
    plot([110-b 110-b],[56+b 44-b],'k--','LineWidth',2);
    plot([122+b 122+b],[56+b 44-b],'k--','LineWidth',2);
    plot([122+b 110-b],[56+b 44-b],'k--','LineWidth',1);
    plot([110-b 122+b],[56+b 44-b],'k--','LineWidth',1);
end



end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Print roads trajectory setpoints
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = proads()
global s

% Print Roads
for i=1:s.rn
    % Print road setpoints
    if(s.mode==1)
        % Roads
        for j=1:s.ri(i,5)
            if(s.printrsp==1)
                % Plot road setpoints
                plot(s.r(i,j,1),s.r(i,j,2),'y.')
            end
            % Plot road ID
            if(j==3 && s.printrid==1)
                text(s.r(i,j,1),s.r(i,j,2),num2str(i),'color','k','BackgroundColor','w','Fontsize',6);
            end
        end
    end
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Roads trajectory setpoints
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = roads()
global s
extRoad=50;
i=0;
% Roads  - Store information                ri(1,:)=[xi yi xf yf nsetpoints length orientation intype endtype]
% Entrance in simulator (1-12)
i=i+1; s.ri(i,8)=1; s.ri(i,9)=2; s.ri(i,1:4)=[0 -s.rw/2+50 31 -s.rw/2+50];  s.rn=i;
i=i+1; s.ri(i,8)=1; s.ri(i,9)=2; s.ri(i,1:4)=[0 -s.rw*(s.rw/2)+50 31 -s.rw*(s.rw/2)+50];  s.rn=i;
i=i+1; s.ri(i,8)=1; s.ri(i,9)=2; s.ri(i,1:4)=[s.rw/2+50 0 s.rw/2+50 31];  s.rn=i;
i=i+1; s.ri(i,8)=1; s.ri(i,9)=2; s.ri(i,1:4)=[s.rw*(s.rw/2)+50 0 s.rw*(s.rw/2)+50 31];  s.rn=i;
i=i+1; s.ri(i,8)=1; s.ri(i,9)=3; s.ri(i,1:4)=[s.rw/2+116 0 s.rw/2+116 44];  s.rn=i;
i=i+1; s.ri(i,8)=1; s.ri(i,9)=3; s.ri(i,1:4)=[s.rw*(s.rw/2)+116 0 s.rw*(s.rw/2)+116 44];  s.rn=i;
i=i+1; s.ri(i,8)=1; s.ri(i,9)=3; s.ri(i,1:4)=[160 s.rw/2+50 122 s.rw/2+50];  s.rn=i;
i=i+1; s.ri(i,8)=1; s.ri(i,9)=3; s.ri(i,1:4)=[160 s.rw*(s.rw/2)+50 122 s.rw*(s.rw/2)+50];  s.rn=i;
i=i+1; s.ri(i,8)=1; s.ri(i,9)=3; s.ri(i,1:4)=[-s.rw/2+116 100 -s.rw/2+116 56];  s.rn=i;
i=i+1; s.ri(i,8)=1; s.ri(i,9)=3; s.ri(i,1:4)=[-s.rw*(s.rw/2)+116 100 -s.rw*(s.rw/2)+116 56];  s.rn=i;
i=i+1; s.ri(i,8)=1; s.ri(i,9)=2; s.ri(i,1:4)=[-s.rw/2+50 100 -s.rw/2+50 69];  s.rn=i;
i=i+1; s.ri(i,8)=1; s.ri(i,9)=2; s.ri(i,1:4)=[-s.rw*(s.rw/2)+50 100 -s.rw*(s.rw/2)+50 69];  s.rn=i;

% Middle roads (13-16)
i=i+1; s.ri(i,8)=3; s.ri(i,9)=2; s.ri(i,1:4)=[110 s.rw/2+50 69 s.rw/2+50];  s.rn=i;
i=i+1; s.ri(i,8)=3; s.ri(i,9)=2; s.ri(i,1:4)=[110 s.rw*(s.rw/2)+50 69 s.rw*(s.rw/2)+50];  s.rn=i;
i=i+1; s.ri(i,8)=2; s.ri(i,9)=3; s.ri(i,1:4)=[69 -s.rw/2+50 110 -s.rw/2+50];  s.rn=i;
i=i+1; s.ri(i,8)=2; s.ri(i,9)=3; s.ri(i,1:4)=[69 -s.rw*(s.rw/2)+50 110 -s.rw*(s.rw/2)+50];  s.rn=i;

% Exit Roads (17-28)
i=i+1; s.ri(i,8)=2; s.ri(i,9)=0; s.ri(i,1:4)=[31 s.rw/2+50 0 s.rw/2+50];  s.rn=i;
i=i+1; s.ri(i,8)=2; s.ri(i,9)=0; s.ri(i,1:4)=[31 s.rw*(s.rw/2)+50 0 s.rw*(s.rw/2)+50];  s.rn=i;
i=i+1; s.ri(i,8)=2; s.ri(i,9)=0; s.ri(i,1:4)=[-s.rw/2+50 31 -s.rw/2+50 0];  s.rn=i;
i=i+1; s.ri(i,8)=2; s.ri(i,9)=0; s.ri(i,1:4)=[-s.rw*(s.rw/2)+50 31 -s.rw*(s.rw/2)+50 0];  s.rn=i;
i=i+1; s.ri(i,8)=3; s.ri(i,9)=0; s.ri(i,1:4)=[-s.rw/2+116 44 -s.rw/2+116 0];  s.rn=i;
i=i+1; s.ri(i,8)=3; s.ri(i,9)=0; s.ri(i,1:4)=[-s.rw*(s.rw/2)+116 44 -s.rw*(s.rw/2)+116 0];  s.rn=i;
i=i+1; s.ri(i,8)=3; s.ri(i,9)=0; s.ri(i,1:4)=[122 -s.rw/2+50 160 -s.rw/2+50];  s.rn=i;
i=i+1; s.ri(i,8)=3; s.ri(i,9)=0; s.ri(i,1:4)=[122 -s.rw*(s.rw/2)+50 160 -s.rw*(s.rw/2)+50];  s.rn=i;
i=i+1; s.ri(i,8)=3; s.ri(i,9)=0; s.ri(i,1:4)=[s.rw/2+116 56 s.rw/2+116 100];  s.rn=i;
i=i+1; s.ri(i,8)=3; s.ri(i,9)=0; s.ri(i,1:4)=[s.rw*(s.rw/2)+116 56 s.rw*(s.rw/2)+116 100];  s.rn=i;
i=i+1; s.ri(i,8)=2; s.ri(i,9)=0; s.ri(i,1:4)=[s.rw/2+50 69 s.rw/2+50 100];  s.rn=i;
i=i+1; s.ri(i,8)=2; s.ri(i,9)=0; s.ri(i,1:4)=[s.rw*(s.rw/2)+50 69 s.rw*(s.rw/2)+50 100];  s.rn=i;

% Calculate road setpoints
for i=1:s.rn
    s.ri(i,6)=rlength( s.ri(i,1:4) );           % Calculates the length of road
    s.ri(i,5)=2+floor( s.ri(i,6) / s.rd );      % Calculates the number of setpoints needed
    s.ri(i,7)=rorientation( s.ri(i,1:4) );      % Calculates the road orientation
    gen_set_points( i , s.ri(i,:) );            % Generates the road setpoints and store them
    
    % If the last 2 setpoints are equal delete the last
    aa=round2(s.r(i,s.ri(i,5),1),0.001);    bb=round2(s.r(i,s.ri(i,5)-1,1),0.001);
    cc=round2(s.r(i,s.ri(i,5),2),0.001);    dd=round2(s.r(i,s.ri(i,5)-1,2),0.001);
    if( ( aa == bb ) && ( cc == dd) )
        s.r(i,s.ri(i,5),1)=0;
        s.r(i,s.ri(i,5),2)=0;
        s.ri(i,5)=s.ri(i,5)-1;
    end
end

% Print lanes
for i=1:s.rn
    if(s.mode==1)
        proad(i);   % Print lanes
    end
end

%Store next launch time
s.ri(1:i,10)=0;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Roundabout trajectory setpoints
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = roundabout()
global s

% Roundabout definitions
xc=s.round_center(1); yc=s.round_center(2);           % Center
rI=s.radiusI;           % Radius of the trajectory (inside lane)
rO=s.radiusO;           % Radius of the trajectory (outside lane)

% Print lanes
if(s.mode==1)
    x1=[]; y1=[]; r1=s.radiusI-(s.rrw/2);   % Orange Center
    x2=[]; y2=[]; r2=s.radiusI+(s.rrw/2);   % Line between the two tracks
    x3=[]; y3=[]; r3=s.radiusO+(s.rrw/2);   % Lanes (two)
    for ang=0:0.1:2*pi
        x1=[x1 xc+r1*cos(ang)]; x2=[x2 xc+r2*cos(ang)]; x3=[x3 xc+r3*cos(ang)];
        y1=[y1 yc+r1*sin(ang)]; y2=[y2 yc+r2*sin(ang)]; y3=[y3 yc+r3*sin(ang)];
        
    end
    fill(x3,y3,'w','Edgecolor','w');
    plot(x2,y2,'k--');
    fill(x1,y1,[0.8 0.8 0.8])
end

% Roundabout outside lane setpoints
aux=0;
for t=0:s.rbd(1,1):360
    aux=aux+1;
    s.r1(aux,1)=xc+rO*cos(t*pi/180); s.r1(aux,2)=yc+rO*sin(t*pi/180); s.r1(aux,3)=t;
    if(s.printrsp==1  && s.mode==1)
        plot(s.r1(aux,1),s.r1(aux,2),'w.')
    end
end
s.r1i(1,1)=aux;
%If last setpoint is repeated delete them
if( round2(s.r1(aux,1),0.001) == round2(s.r1(1,1),0.001)  )
    if( round2(s.r1(aux,2),0.001) == round2(s.r1(1,2),0.001) )
        s.r1(aux,:)=[];
        s.r1i(1,1)=aux-1;
    end
end

% Roundabout inside lane setpoints
aux=0;
for t=0:s.rbd(1,2):360      % Inside lane
    aux=aux+1;
    s.r2(aux,1)=xc+rI*cos(t*pi/180); s.r2(aux,2)=yc+rI*sin(t*pi/180); s.r2(aux,3)=t;
    if(s.printrsp==1 && s.mode==1)
        plot(s.r2(aux,1),s.r2(aux,2),'w.')
    end
end
s.r2i(1,1)=aux;
%If last setpoint is repeated delete them
if( round2(s.r2(aux,1),0.001) == round2(s.r2(1,1),0.001)  )
    if( round2(s.r2(aux,2),0.001) == round2(s.r2(1,2),0.001) )
        s.r2(aux,:)=[];
        s.r2i(1,1)=aux-1;
    end
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate setpoints to one road
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = gen_set_points( r , arg )
global s

xi=arg(1); yi=arg(2); xf=arg(3); yf=arg(4); n=arg(5); ang=arg(7);

% Stores the initial point
s.r(r,1,1)=xi;      s.r(r,1,2)=yi;

% Stores the midlle points
for i=2:(n-1)
    s.r(r,i,1)=xi+s.rd*(i-1)*cos(ang);   s.r(r,i,2)=yi+s.rd*(i-1)*sin(ang);
end

% Stores the end point
s.r(r,n,1)=xf; s.r(r,n,2)=yf;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculation the length of road
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [d]=rlength(arg)
xi=arg(1); yi=arg(2); xf=arg(3); yf=arg(4);
d=sqrt( (xf-xi)^2 + (yf-yi)^2 );
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculation the orientation of road
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ang]=rorientation(arg)
xi=arg(1); yi=arg(2); xf=arg(3); yf=arg(4);
ang=atan2((yf-yi) , (xf-xi));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot a single road
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = proad(i)
global s

ang=s.ri(i,7); xi=s.ri(i,1); yi=s.ri(i,2); xf=s.ri(i,3); yf=s.ri(i,4);

% Calculate corner points
c1x=xi+(s.rw/2)*cos(ang+pi/2);  c1y=yi+(s.rw/2)*sin(ang+pi/2);
c2x=xi+(s.rw/2)*cos(ang-pi/2);  c2y=yi+(s.rw/2)*sin(ang-pi/2);
c3x=xf+(s.rw/2)*cos(ang+pi/2);  c3y=yf+(s.rw/2)*sin(ang+pi/2);
c4x=xf+(s.rw/2)*cos(ang-pi/2);  c4y=yf+(s.rw/2)*sin(ang-pi/2);

% Plot road
fill([c1x c2x c4x c3x],[c1y c2y c4y c3y],'w','Facecolor','w','edgecolor','w');

if(intersect(i,[3 4 5 6 25 26 27 28]))
    if(mod(i,2)==0) % Odd
        
        % Plot road side lines
        for n=c2y:5:c4y
            plot([c1x c3x],[n n+2.5],'k');
        end
        
        plot([c2x c4x],[c2y c4y],'k');
    else % Even
        
        % Plot road side lines
        for n=c1y:5:c3y
            plot([c2x c4x],[n n+2.5],'k');
        end
        
        plot([c1x c3x],[c1y c3y],'k');
        
    end
elseif (intersect(i,[9 10 11 12 19 20 21 22]))
    if(mod(i,2)==0) % Odd
        
        plot([c2x c4x],[c2y c4y],'k');
        % Plot road side lines
        for n=c3y:5:c1y
            plot([c1x c3x],[n n-2.5],'k');
        end
        
    else % Even
        
        plot([c1x c3x],[c1y c3y],'k');
        % Plot road side lines
        
        for n=c4y:5:c2y
            plot([c2x c4x],[n n-2.5],'k');
        end
        
    end
    % Horizontal Roads
elseif (intersect(i,[1 2 15 16 23 24]))
    if(mod(i,2)==0) % Odd
        
        % Plot road side lines
        for n=c2x:5:c4x
            plot([n n+2.5],[c1y c3y],'k');
        end
        
        plot([c2x c4x],[c2y c4y],'k');
    else % Even
        
        % Plot road side lines
        for n=c1x:5:c3x
            plot([n n+2.5],[c2y c4y],'k');
        end
        plot([c1x c3x],[c1y c3y],'k');
    end
    
elseif (intersect(i,[7 8 13 14 17 18]))
    if(mod(i,2)==0) % Odd
        
        % Plot road side lines
        for n=c4x:5:c2x
            plot([n n+2.5],[c1y c3y],'k');
        end
        
        plot([c2x c4x],[c2y c4y],'k');
    else % Even
        
        % Plot road side lines
        for n=c3x:5:c1x
            plot([n n+2.5],[c2y c4y],'k');
        end
        plot([c1x c3x],[c1y c3y],'k');
    end
    
else
    % Plot road side lines
    plot([c1x c3x],[c1y c3y],'k');   plot([c2x c4x],[c2y c4y],'k');
end
end
