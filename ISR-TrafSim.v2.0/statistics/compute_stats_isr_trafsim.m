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

function [  ] = compute_stats_isr_trafsim(  )
global s
load('savedata\predata.mat')

list=dir('savedata\car');
nc=size(list,1)-2;
carsWithoutV2I=0;
carsWithV2I=0;
nwc=0;
nnwc=0;
if(nc>0)
    for n=1:nc
        load(['savedata\car\car_' num2str(n) '.mat'])
        % c.car(n).TRUEpos(end+1,:)=[c.car(n).pos.x c.car(n).pos.y c.car(n).pos.t c.car(n).dyn(1) c.car(n).dyn(2) c.car(n).cmdacc c.car(n).cmdste dist_isr_trafsim(c.car(n).posprv.x, c.car(n).posprv.y,c.car(n).pos.x,c.car(n).pos.y)];     % Store car trajectory
        d=aux.TRUEpos;
        % All cars
        dist_(n)=sum(d(:,8));                % meters
        mean_vel(n)=mean(d(:,4));           % m/s
        mean_vel_move=[];
        time(n)=size(d,1)*0.01;             % seconds
        time_per_meter=time(n)/dist_(n) ;    % seconds/meter
        timestop(n)=0;
        timemove(n)=0;
        time_vel_less_10(n)=0;
        time_vel_less_20(n)=0;
        time_vel_less_30(n)=0;
        
        time_acc(n)=0;
        time_brk(n)=0;
        time_acc_percent(n)=0;
        time_brk_percent(n)=0;
        mean_vel_move=[];
        for i=1:size(d,1)
            v=d(i,4);
            if(v==0)
                timestop(n)=timestop(n)+0.01;
            end
            if(v>0)
                timemove(n)=timemove(n)+0.01;
                mean_vel_move(end+1)=v;
            end
            if(v<10/3.6)
                time_vel_less_10(n)=time_vel_less_10(n)+0.01;
            end
            if(v<20/3.6)
                time_vel_less_20(n)=time_vel_less_20(n)+0.01;
            end
            if(v<30/3.6)
                time_vel_less_30(n)=time_vel_less_30(n)+0.01;
            end
            if(d(i,6)==1)
                time_acc(n)=time_acc(n)+0.01;
            elseif(d(i,6)==-1)
                time_brk(n)=time_brk(n)+0.01;
            end
        end
        timestop_percent(n)=(timestop(n)/time(n))*100;
        time_vel_less_10_percent(n)=(time_vel_less_10(n)/time(n))*100;
        time_vel_less_20_percent(n)=(time_vel_less_20(n)/time(n))*100;
        time_vel_less_30_percent(n)=(time_vel_less_30(n)/time(n))*100;
        timemove_percent(n)=100-timestop_percent(n);
        timestop_perdist(n)=timestop(n)/dist_(n);
        time_acc_percent(n)=time_acc(n)/time(n);
        time_brk_percent(n)=time_brk(n)/time(n);
        mean_vel_moving(n)=mean(mean_vel_move);
        
        if(aux.commu==1)
            nwc=nwc+1;
            carsWithV2I=carsWithV2I+1;
            dist_wc(nwc)=sum(d(:,8));                % meters
            mean_vel_wc(nwc)=mean(d(:,4));           % m/s
            mean_vel_move_wc=[];
            time_wc(nwc)=size(d,1)*0.01;             % seconds
            time_per_meter_wc=time_wc(nwc)/dist_wc(nwc) ;    % seconds/meter
            timestop_wc(nwc)=0;
            timemove_wc(nwc)=0;
            time_vel_less_10_wc(nwc)=0;
            time_vel_less_20_wc(nwc)=0;
            time_vel_less_30_wc(nwc)=0;
            
            time_acc_wc(nwc)=0;
            time_brk_wc(nwc)=0;
            time_acc_percent_wc(nwc)=0;
            time_brk_percent_wc(nwc)=0;
            mean_vel_move_wc=[];
            for i=1:size(d,1)
                v=d(i,4);
                if(v==0)
                    timestop_wc(nwc)=timestop_wc(nwc)+0.01;
                end
                if(v>0)
                    timemove_wc(nwc)=timemove_wc(nwc)+0.01;
                    mean_vel_move_wc(end+1)=v;
                end
                if(v<10/3.6)
                    time_vel_less_10_wc(nwc)=time_vel_less_10_wc(nwc)+0.01;
                end
                if(v<20/3.6)
                    time_vel_less_20_wc(nwc)=time_vel_less_20_wc(nwc)+0.01;
                end
                if(v<30/3.6)
                    time_vel_less_30_wc(nwc)=time_vel_less_30_wc(nwc)+0.01;
                end
                if(d(i,6)==1)
                    time_acc_wc(nwc)=time_acc_wc(nwc)+0.01;
                elseif(d(i,6)==-1)
                    time_brk_wc(nwc)=time_brk_wc(nwc)+0.01;
                end
            end
            timestop_percent_wc(nwc)=(timestop_wc(nwc)/time_wc(nwc))*100;
            time_vel_less_10_percent_wc(nwc)=(time_vel_less_10_wc(nwc)/time_wc(nwc))*100;
            time_vel_less_20_percent_wc(nwc)=(time_vel_less_20_wc(nwc)/time_wc(nwc))*100;
            time_vel_less_30_percent_wc(nwc)=(time_vel_less_30_wc(nwc)/time_wc(nwc))*100;
            timemove_percent_wc(nwc)=100-timestop_percent_wc(nwc);
            timestop_perdist_wc(nwc)=timestop_wc(nwc)/dist_wc(nwc);
            time_acc_percent_wc(nwc)=time_acc_wc(nwc)/time_wc(nwc);
            time_brk_percent_wc(nwc)=time_brk_wc(nwc)/time_wc(nwc);
            mean_vel_moving_wc(nwc)=mean(mean_vel_move_wc);
        elseif(aux.commu==0)
            carsWithoutV2I=carsWithoutV2I+1;
            nnwc = nnwc + 1;
            dist_nwc(nnwc)=sum(d(:,8));                % meters
            mean_vel_nwc(nnwc)=mean(d(:,4));           % m/s
            mean_vel_move_nwc=[];
            time_nwc(nnwc)=size(d,1)*0.01;             % seconds
            time_per_meter_nwc=time_nwc(nnwc)/dist_nwc(nnwc) ;    % seconds/meter
            timestop_nwc(nnwc)=0;
            timemove_nwc(nnwc)=0;
            time_vel_less_10_nwc(nnwc)=0;
            time_vel_less_20_nwc(nnwc)=0;
            time_vel_less_30_nwc(nnwc)=0;
            
            time_acc_nwc(nnwc)=0;
            time_brk_nwc(nnwc)=0;
            time_acc_percent_nwc(nnwc)=0;
            time_brk_percent_nwc(nnwc)=0;
            mean_vel_move_nwc=[];
            for i=1:size(d,1)
                v=d(i,4);
                if(v==0)
                    timestop_nwc(nnwc)=timestop_nwc(nnwc)+0.01;
                end
                if(v>0)
                    timemove_nwc(nnwc)=timemove_nwc(nnwc)+0.01;
                    mean_vel_move_nwc(end+1)=v;
                end
                if(v<10/3.6)
                    time_vel_less_10_nwc(nnwc)=time_vel_less_10_nwc(nnwc)+0.01;
                end
                if(v<20/3.6)
                    time_vel_less_20_nwc(nnwc)=time_vel_less_20_nwc(nnwc)+0.01;
                end
                if(v<30/3.6)
                    time_vel_less_30_nwc(nnwc)=time_vel_less_30_nwc(nnwc)+0.01;
                end
                if(d(i,6)==1)
                    time_acc_nwc(nnwc)=time_acc_nwc(nnwc)+0.01;
                elseif(d(i,6)==-1)
                    time_brk_nwc(nnwc)=time_brk_nwc(nnwc)+0.01;
                end
            end
            timestop_percent_nwc(nnwc)=(timestop_nwc(nnwc)/time_nwc(nnwc))*100;
            time_vel_less_10_percent_nwc(nnwc)=(time_vel_less_10_nwc(nnwc)/time_nwc(nnwc))*100;
            time_vel_less_20_percent_nwc(nnwc)=(time_vel_less_20_nwc(nnwc)/time_nwc(nnwc))*100;
            time_vel_less_30_percent_nwc(nnwc)=(time_vel_less_30_nwc(nnwc)/time_nwc(nnwc))*100;
            timemove_percent_nwc(nnwc)=100-timestop_percent_nwc(nnwc);
            timestop_perdist_nwc(nnwc)=timestop_nwc(nnwc)/dist_nwc(nnwc);
            time_acc_percent_nwc(nnwc)=time_acc_nwc(nnwc)/time_nwc(nnwc);
            time_brk_percent_nwc(nnwc)=time_brk_nwc(nnwc)/time_nwc(nnwc);
            mean_vel_moving_nwc(nnwc)=mean(mean_vel_move_nwc);
            
        end
        
    end
    
    disp('>>> DISP STATS')
    disp(['Cars created = ' num2str(s.curNcars) ' cars'])
    disp(['Cars with V2I = ' num2str(carsWithV2I) ' cars'])
    disp(['Cars without V2I = ' num2str(carsWithoutV2I) ' cars'])
    disp(['Cars in simulator = ' num2str(size(c.listactive,2)) ' cars'])
    disp(['Cars waiting to enter = ' num2str(sum(s.rl(2,:))) ' cars'])
    disp(['Cars exit from simulator = ' num2str(s.cars_exit) ' cars'])
    disp(' ')
    disp('>>> DISP CAR STATS - All')
    disp(['Distance [m]= ' num2str(mean(dist_)) ]);
    disp(['Time in simulator [s]= ' num2str(mean(time)) ])
    disp(['Time in simulator per dist [s/m]= ' num2str(mean(time_per_meter)) ])
    disp(['Time move [s]= ' num2str(mean(timemove)) ' / ' num2str(mean(timemove_percent)) ' %'])
    disp(['Time stoped [s]= ' num2str(mean(timestop)) ' / ' num2str(mean(timestop_percent)) ' %'])
    disp(['Time stoped per car [s/car]= ' num2str(mean(timestop)/nc) ])
    disp(['Time stoped per dist [s/m]= ' num2str(mean(timestop_perdist)) ])
    disp(['Mean velocity [m/s]/[Km/h]= ' num2str(mean(mean_vel)) '/' num2str(mean(mean_vel)*3.6) ])
    disp(['Mean velocity while moving [m/s]/[Km/h]= ' num2str(mean(mean_vel_moving)) ' / ' num2str(mean(mean_vel_moving)*3.6) ])
    disp(['Time that vel < 10Km/h [s]= ' num2str(mean(time_vel_less_10))  ' / ' num2str(mean(time_vel_less_10_percent)) ' %'])
    disp(['Time that vel < 20Km/h [s]= ' num2str(mean(time_vel_less_20))  ' / ' num2str(mean(time_vel_less_20_percent)) ' %'])
    disp(['Time that vel < 30Km/h [s]= ' num2str(mean(time_vel_less_30))  ' / ' num2str(mean(time_vel_less_30_percent)) ' %'])
    disp(['Time accelarating = ' num2str(mean(time_acc)) ' [s] / ' num2str(mean(time_acc_percent)*100) ' %' ])
    disp(['Time breaking = ' num2str(mean(time_brk)) ' [s] / ' num2str(mean(time_brk_percent)*100) ' %' ])
    disp(' ')
    disp('>>> DISP CAR STATS (V2I)')
    disp(['Distance [m]= ' num2str(mean(dist_wc)) ]);
    disp(['Time in simulator [s]= ' num2str(mean(time_wc)) ])
    disp(['Time in simulator per dist [s/m]= ' num2str(mean(time_per_meter_wc)) ])
    disp(['Time move [s]= ' num2str(mean(timemove_wc)) ' / ' num2str(mean(timemove_percent_wc)) ' %'])
    disp(['Time stoped [s]= ' num2str(mean(timestop_wc)) ' / ' num2str(mean(timestop_percent_wc)) ' %'])
    disp(['Time stoped per car [s/car]= ' num2str(mean(timestop_wc)/carsWithV2I) ])
    disp(['Time stoped per dist [s/m]= ' num2str(mean(timestop_perdist_wc)) ])
    disp(['Mean velocity [m/s]/[Km/h]= ' num2str(mean(mean_vel_wc)) '/' num2str(mean(mean_vel_wc)*3.6) ])
    disp(['Mean velocity while moving [m/s]/[Km/h]= ' num2str(mean(mean_vel_moving_wc)) ' / ' num2str(mean(mean_vel_moving_wc)*3.6) ])
    disp(['Time that vel < 10Km/h [s]= ' num2str(mean(time_vel_less_10_wc))  ' / ' num2str(mean(time_vel_less_10_percent_wc)) ' %'])
    disp(['Time that vel < 20Km/h [s]= ' num2str(mean(time_vel_less_20_wc))  ' / ' num2str(mean(time_vel_less_20_percent_wc)) ' %'])
    disp(['Time that vel < 30Km/h [s]= ' num2str(mean(time_vel_less_30_wc))  ' / ' num2str(mean(time_vel_less_30_percent_wc)) ' %'])
    disp(['Time accelarating = ' num2str(mean(time_acc_wc)) ' [s] / ' num2str(mean(time_acc_percent_wc)*100) ' %' ])
    disp(['Time breaking = ' num2str(mean(time_brk_wc)) ' [s] / ' num2str(mean(time_brk_percent_wc)*100) ' %' ])
    disp(' ')
    if(s.cars_rate_no_V2I(1)~=0)
        disp('>>> DISP CAR STATS (NOT V2I)')
        disp(['Distance [m]= ' num2str(mean(dist_nwc)) ]);
        disp(['Time in simulator [s]= ' num2str(mean(time_nwc)) ])
        disp(['Time in simulator per dist [s/m]= ' num2str(mean(time_per_meter_nwc)) ])
        disp(['Time move [s]= ' num2str(mean(timemove_nwc)) ' / ' num2str(mean(timemove_percent_nwc)) ' %'])
        disp(['Time stoped [s]= ' num2str(mean(timestop_nwc)) ' / ' num2str(mean(timestop_percent_nwc)) ' %'])
        disp(['Time stoped per car [s/car]= ' num2str(mean(timestop_nwc)/carsWithoutV2I) ])
        disp(['Time stoped per dist [s/m]= ' num2str(mean(timestop_perdist_nwc)) ])
        disp(['Mean velocity [m/s]/[Km/h]= ' num2str(mean(mean_vel_nwc)) '/' num2str(mean(mean_vel_nwc)*3.6) ])
        disp(['Mean velocity while moving [m/s]/[Km/h]= ' num2str(mean(mean_vel_moving_nwc)) ' / ' num2str(mean(mean_vel_moving_nwc)*3.6) ])
        disp(['Time that vel < 10Km/h [s]= ' num2str(mean(time_vel_less_10_nwc))  ' / ' num2str(mean(time_vel_less_10_percent_nwc)) ' %'])
        disp(['Time that vel < 20Km/h [s]= ' num2str(mean(time_vel_less_20_nwc))  ' / ' num2str(mean(time_vel_less_20_percent_nwc)) ' %'])
        disp(['Time that vel < 30Km/h [s]= ' num2str(mean(time_vel_less_30_nwc))  ' / ' num2str(mean(time_vel_less_30_percent_nwc)) ' %'])
        disp(['Time accelarating = ' num2str(mean(time_acc_nwc)) ' [s] / ' num2str(mean(time_acc_percent_nwc)*100) ' %' ])
        disp(['Time breaking = ' num2str(mean(time_brk_nwc)) ' [s] / ' num2str(mean(time_brk_percent_nwc)*100) ' %' ])
    end
else
    disp('No cars have exit');
end

%         disp(['Average time cars are stopped = ' num2str(mean(s.cars_time_stop)) ' (s)'])
%         disp(['Percentage of time cars are stopped = ' num2str(mean(s.cars_time_stop_perc)) ' (%)'])
%         disp(['Average time cars are in motion = ' num2str(mean(s.cars_time_run)) ' (s)'])
%         disp(['Percentage of time cars are in motion = ' num2str(mean(s.cars_time_run_perc)) ' (%)'])
%         disp(['Average time of cars in simulator by distance traveled = ' num2str(mean(s.cars_time_insim)) ' (s/m)'])
%         disp(['Average velocity during all route = ' num2str(mean(s.cars_mean_vel)*3.6) ' (Km/h)'])
%         disp(['Average velocity only when cars are in motion = ' num2str(mean(s.cars_mean_run_vel)*3.6) ' (Km/h)'])
%         disp(['Cars created = ' num2str(s.curNcars) ' cars'])
%         disp(['Cars in simulator = ' num2str(size(c.listactive,2)) ' cars'])
%         disp(['Cars waiting to enter = ' num2str(sum(s.rl(2,:))) ' cars'])
%         disp(['Cars exit from simulator = ' num2str(s.cars_exit) ' cars'])
%         disp(['Number of overtakes = ' num2str(s.overtake_n) ' cars'])


