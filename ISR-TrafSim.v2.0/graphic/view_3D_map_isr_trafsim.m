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
% To view 3D reservation matrix
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

function [ ] = view_3D_map_isr_trafsim( arg )
global s

X=linspace(0,160,size(s.ocmap,2));
Y=linspace(0,100,size(s.ocmap,1));
aux=s.ocmap_3D; aux_=[];
for z=1:size(aux,3)
    aux_(:,:,z)=rot90(aux(:,:,z));
    aux_(:,:,z)=fliplr(aux_(:,:,z));
end
aux=aux_;
for i=1:size(aux,1)
    for j=1:size(aux,2)
        for z=1:size(aux,3)
            if(aux(i,j,z)==0)
                aux(i,j,z)=NaN;
            end
        end
    end
end


s.h_fig_3D=figure('name','3D Reservation');
if(arg==1)
    [a,b,c]=size(aux);
    if(sum(sum(sum((isnan(aux)))))==a*b*c)
        warndlg('Empty reservation matrix','!! ')
    else
        PATCH_3Darray(aux,fliplr(X),fliplr(Y),s.layer,'col')
        axis([0 160 0 100 s.layer(1) s.layer(2)])
    end
elseif(arg==3)
    
    m=aux(1:end/2,:,:);
    [a,b,c]=size(m);
    if(sum(sum(sum((isnan(m)))))==a*b*c)
        warndlg('Empty reservation matrix','!! ')
    else
        PATCH_3Darray(m,fliplr(X(end-(a-1):end)),fliplr(Y),s.layer,'col')
        axis([a 160 0 100 s.layer(1) s.layer(2)])
    end
elseif(arg==2)
    
    m=aux(end/2:end,:,:);
    [a,b,c]=size(m);
    if(sum(sum(sum((isnan(m)))))==a*b*c)
        warndlg('Empty reservation matrix','!! ')
    else
        PATCH_3Darray(m,fliplr(X(1:a)),fliplr(Y),s.layer,'col')
        axis([0 a 0 100 s.layer(1) s.layer(2)])
    end
end
end

