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
% Return a structure with initial configurations
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

function [out] = configsim_isr_trafsim(arg)

if(strcmp(arg,'init'))
    out=sim_init();
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initializes all the variables necessary for the simulator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [out] = sim_init()

out=struct...
    (...
    'mode',1,...                % (1-Online, 0-Offline, 2-C++)
    'time',0,...                % Initial time (not change)
    'step_time',0.01,...        % Step Time for each interaction of simulator (not change)
    ...     % Basic options
    'duration',50,...           % Duration of simulation in seconds
    'tlaunch',3600/100,...      % 3600/20,...     % Time between launchs in every road (seconds). Ex: 3600/150 -> 150 cars/hour
    'tlaunchr',3600/100/12,...  % 3600/20/12,...  % Launch time beetween roads (seconds). Ex: 3600/150/12 -> Divide 150 cars/hour per 12 lanes
    'round_gest',1,...          % 0-System OFF, 1-System ON
    'round_method',2,...        % Select desired algorithm (1-Basic, 2-Advanced)
    'cross_mode',2,...          % 0-Normal mode , 1-Semi-Vision, 2-Spacio-Temporal Reserve
    'cross_method',2,...        % Select desired algorithm (1-Basic, 2-Advanced)
    ...     % Launch Options
    'ncars',Inf,...               % Limit of cars
    'cars_same',1,...             % All cars/drivers be the same type(1-ON,2-OFF)
    'cars_rate_no_V2I',[0 0],...  % 0.1->10% Rate of cars without communications [rate cars_without];
    'curNcars',0,...                    % Current cars launched
    'distlaunch',22,...                 % Distance that has to be free to launch car in simulator
    'nspeed',2,...                      % Noise speed that will be added to the maximum speed in km/h (velmax=velmax +- nspeed)
    'run_before',0,...                  % The cars will use position methods before entry in simulator
    ...     % Aditional Options
    'overtake',0,...            % If cars can overtake
    'overtake_n',0,...          % Number of overtakes
    'make_video',1,...          % Make video
    'makeVideo_Reserve',1,...
    'followcar',0,...           % (0-Off, else the car with ID is followed if exists)
    'nframes',0,...             % Number of frames stored to offline visualization
    'save_data',1,...           % Enable save data in 'file.mat'. file->[year_month_day_hour_minute.mat]
    'save_car',1,...            % Enable save individual data from every car
    'pack_time',100,...         % To avoid out of memory errors
    'pause',0,...               % to pause simulation
    ...     % Display options
    'lateral_errors',1,...      % Compute lateral errors
    'ocmap_accum',1,...         % Plot occupation map accumulator at end of simulation
    'ocmap_accum_graph',1,...   % Plot occupation map graph at the end of simulator
    'disp_res',1,...            % Adquire data to display global results
    'disp_car_res',1,...        % Adquire data to display car results (Position methods errors)
    'fres',10,...                % Frequency of data acquisition
    'disp_res2',1,...           % Adquire data to display paths velocity
    'fres2',10,...              % Frequency of acquisition
    'ocmap4',[],...             % To store path velocity
    'disp_fus2_res',1,...       % To display filter errors (fusion 4)
    'disp_fus4_res',1,...       % To display filter errors (fusion 4)
    'disp_fus3_res',1,...       % To display results from fusion 3
    'view_gps_errors',1,...     % To view gps erros for the 3 types in simulation init
    'view_3D_Matrix',1,...      % To view 3D matrix from crossroads and roundabout
    ...     % Localization methods
    'ins',0,...     % (0-No, 1-Yes, 2-Yes & PrintGhost)
    'fus',0,...     % (0-No, 1-Yes, 2-Yes & PrintGhost)     GPS+INS (+MAG)
    'fus2',0,...    % (0-No, 1-Yes, 2-Yes & PrintGhost)     GPS+LASER
    'fus3',0,...    % (0-No, 1-Yes, 2-Yes & PrintGhost)     GPS+INS+KF
    'fus4',0,...    % (0-No, 1-Yes, 2-Yes & PrintGhost)     ODO w/ EKF (+MAG)
    'odo',0,...     % (0-No, 1-Yes, 2-Yes & PrintGhost)
    'gps',1,...     % (0-No, 1-Yes, 2-Yes & PrintGhost)
    'las',0,...     % (0-No, 1-Yes)
    'mag',0,...     % (0-No, 1-Yes)
    ...     % GPS
    'gps_type',2,...                    % 1-SatNav / 2-gps
    'gps_C',0,...                       % gps variable
    'gps_F',0,...                       % gps variable
    'gps_T_amb',0,...                   % gps variable
    'gps_P_amp',0,...                   % gps variable
    'gps_P_vap',0,...                   % gps variable
    'gps_eph',0,...                     % gps variable
    'gps_iono',0,...                    % gps variable
    'gps_pr',0,...                      % gps variable
    'gps_Beta',0,...                    % gps variable
    'gps_GPS_Time',0,...                % gps variable
    'gps_Alpha',0,...                   % gps variable
    'gps_af',0,...                      % gps variable
    'gps_Toc',0,...                     % gps variable
    'gps_Ttr',0,...                     % gps variable
    'gps_ec',0,...                      % gps variable
    'gps_A',0,...                       % gps variable
    'gps_Tgd',0,...                     % gps variable
    'gps_Rho_0',0,...                   % gps variable
    'gps_Pos_xyz_Mat',0,...             % gps variable
    'gps_orbit_parameter',0,...         % gps variable
    'gps_Pos_SV',0,...                  % gps variable
    'gps_Ec',0,...                      % gps variable
    'gps_Inc',0,...                     % gps variable
    'gps_Omega',0,...                   % gps variable
    'gps_v',0,...                       % gps variable
    'gps_Dim',0,...                     % gps variable
    'gps_n',0,...                       % gps variable
    'gps_trop_error',1,...              % 1-On / 2-off
    'gps_ion_error',1,...               % 1-On / 2-off
    'gps_clock_offset',1,...            % 1-On / 2-off
    'gps_clock_rel',1,...               % 1-On / 2-off
    ...     % Methods frequency [Hz]
    'frqgps',10,...                     % GPS
    'frqrtkgps',100,...                  % RTK-GPS
    'frqins',100,...                    % INS
    'frqfus',50,...                     % Sensorial Fusion
    'frqfus2',100,...                     % Sensorial Fusion 2
    'frqfus3',100,...                     % Sensorial Fusion 3
    'frqfus4',100,...                     % Sensorial Fusion 4
    'frqodo',100,...                    % Odometry
    'fmag',100,...                      % Magnets
    'flas',100,...                      % Laser
    ...     % Map properties
    'dimx',160,...                      % Dimensions of the map in meters
    'dimy',100,...                      % Dimensions of the map in meters
    ...     % OTA Communication properties
    'otacommu',1,...                    % Communication over-the-air (1-On , 0-off)
    'otacommuemulation',0,...           % Emulation of OTA communication
    'otarangeGPS',1000,...              % Range of broadcasting GPS info
    'otarangeround',35,...              % Range used when communication are not emulated - Round
    'otarangecross',25,...              % Range used when communication are not emulated - Cross
    'otagps',1,...                      % RTK-GPS (0-OFF, 1-ON)
    'otafgps',100,...                   % Frequency of transmission of GPS information I2C (infrastructure to car)
    'use_gps_time',0,...                % 1 for use 'gpstime' in rtk-gps
    'otalist',[],...                    % List of cars that are already nodes (90 is the ID of gestion system)
    'radiopropagmodel',1,...            % Radio Propagation model (1-'friis'; 2-'tworay'; 3-'shadowing')
    'default_power',10,...              % Radio defaul power (0.2)
    'commu_norm','a',...                % IEEE 802.11x (b/g/a)
    'max_retries',1,...                 % Maximum retries for RTS
    'otashow',0,...                     % Show information on online simulation
    'snr_threshold',10,...              % Receive threshold is used to determine if a reception with SNR is abov this threshold so that the packet can be correctly received.
    ...     % Gestion Methods
    'f_traffic_gest',10,...                 % Frequency of the method
    'layer',[],...                          % List of layers        Ex:[0.51s 0.52s 0.53s]
    'ocmap2',logical([]),...            % Occupation map (layers) to resevation algorithms
    'ocmap_3D',[],...                   % Occupation map (layers) to display 3D resevation
    'flag_view_3D',0,...                % Occupation map reservation
    'flag_view_3D_',0,...               % Occupation map reservation (help)
    'occup_res2',1,...                  % Resolution of the grid (EX: 2 is a resolution of 0.5m)
    'occup_size2',0,...                 % (0=Auto) Half the size analyzed locally for writing a car in the map (EX: 2 is a square with 4m side)
    'layer_w',2,...                     % Safety margin (width) used in drawing on layered map
    'layer_l',10,...                     % Safety margin (length) used in drawing on layered map
    'max_tries',20,...                  % Number maximum of tries on method 2 to resolve a conflit
    'basic_method',0,...                % Number of times that basic method was used
    'sd',20,...                         % Security distance in front
    'lsd',4,...                         % Lateral security distance
    ...     % Roundabout gestion system
    'control_radius_round',35,...                 % Control radius of system
    'cartxf',10,...                         % Frequency with which the car attempts to notify the roundabout of his presence
    'cartxpos',2,...                        % 0 -> True position ; 1 -> Fusion ; 2 -> GPS
    'round_notify_list',zeros(13,1)*NaN,... % List used by roundabout gestion system to manage the cars [id ; node ; state; time; posx; posy; inroad; outroad; currentVel; maxVel; carType, CurrentRoad , timeoflastnotify]
    'cross_notify_list',zeros(13,1)*NaN,... % List used by crossroad gestion system to manage the cars [id ; node ; state; time; posx; posy; inroad; outroad; currentVel; maxVel; carType, CurrentRoad , timeoflastnotify]
    'rg_count_tab',[],...                   % Table that will count how many cars are sent to a particular output
    'rg_car_limit',3,...                    % Limit of cars that can be routed to a particular output
    'list_ignore_round',[],...              % List ignore cars on roundabout
    'list_round_proximity',[],...       % List of cars ear to the roundabout
    'list_round_already_reserve',[],... % List of cars that reserve velocity profile
    'amplitudeRoundControl',[10 -2],... % Radius control for cars withou communication
    'hsema_round',[],...                % Handler to semaphores from roundabout
    'list_round_without_commu',[],...   % List of cars without commu already reserved and their time
    'time_round_without_commu',-1,...   % Time where car without communication will left the roundabout
    'car_round_without_commu',[],...    % Car ID that leaves roundabout withou communication
    'round_last_time_used',0,...        % Last time layer used for the roundabout
    ...     % Cross Gestion
    'hsema_cross',[],...                % Handler to semaphores from crossroads
    'list_cross_proximity',[],...       % List of cars ear to the crossroads
    'list_cross_already_reserve',[],... % List of cars that reserve velocity profile
    'list_ignore_cross',[],...              % List ignore cars on roundabout
    'amplitudeCrossControl',[10 -2],... % Radius control for cars withou communication
    'list_cross_without_commu',[],...   % List of cars without commu already reserved and their time
    'car_cross_without_commu',[],...    % Car ID that leaves crossroads withou communication
    'time_cross_without_commu',-1,...   % Time where car without communication will left the crossroads
    'distcrossextra',4,...              % Extra distance added to cross zone
    'crosszone',[],...                  % Points of corners of cross zone
    'cross_last_time_used',0,...        % Last time layer used for the crossroads
    ...         -> Normal Mode
    'cross_trigger',[20 0 5 6],...      % Trigger to change lights state on cross
    'ntgreen',15,...                    % Time of normal traffic lights stay green
    'ntyello',5,...                     % Time of normal traffic lights stay yellow
    ...         -> Inteligent Mode
    'table_gest0',zeros(3,1)*NaN,...    % Table for cross manegement [ID; roadincoming; arrivetime; nextroad]
    'crosscar',NaN,...                  % Current car on the cross
    'lights',[],...                     % Table with tyraffic lights (road)[active greentimeleft orangetimeleft]
    'tgreen',2,...                      % Time of intelligent traffic lights stay green
    'tyello',2,...                      % Time of intelligent traffic lights stay yellow
    ...         -> Reservation Mode
    'rg_count_tab2',[],...                   % Table that will count how many cars are sent to a particular output
    'rg_car_limit2',3,...                    % Limit of cars that can be routed to a particular output
    'control_radius_cross',30,...            % Control radius of system
    ...     % Stop system
    'lighttable',[],...                     % Table of changing lights
    'stoplist',[],...                       % List of stops active
    'crosslist',[15 16 5 6 7 8 9 10],...
    'roundlist',[1 2 3 4 13 14 12 11],...
    ...     % Magnets
    'mag_list',[],...                   % List of magnets
    'nmag',2,...                        % Number of magnets by road (min=1)
    'rule_length',1,...                 % Length of rule (m) (NOTE: Maximum length is the car width)
    'mag_threshold',0.1,...             % Distance threshold for magnet can be detected
    'mag_Br',0.38,...                   % Inducao magnetica residual do magneto(T)
    'mag_niu',4*pi*10^(-7),...          % Permeabilidade do vazio
    'mag_dia',0.033,...                 % Diâmetro do magneto (m)
    'mag_hi',0.0155,...                 % Altura do magneto
    ...     % Odometry
    'front_noise',2,...                 % Standart deviation of white noise in front wheel encoder
    'rear_noise',1,...                  % Standart deviation of white noise in rear wheel encoder
    ...     % Collisions
    'collision_detect',1,...
    'collisions',0,...                  % List of active collisions (colision)[car1 car2]
    ...     % Car properties
    'CarDim',[1 2],...                     % car=[widht length]
    'cprx',0,...
    'cpry',0,...
    'TruDim',[1.5 3],...                   % truck=[widht length]
    'tprx',0,...
    'tpry',0,...
    'wheel_diam',((205*0.55*2)+16*25.4)/1000,...    % Wheel diameter
    'wheel_radius',0,...                % Wheel radius of cars
    'pulses_per_rev',500,...            % Pulses per revolution of encoder
    ...     % Driver vision
    'zone_type',4,...                   % Type of area to be examined in the search for other cars
    ...                                 % (1-Elipse, 2-Circle, 3- Rectangle, 4-Fast method(with circle_r))
    'elipse_M',25,...                   % Elipse major axis
    'elipse_m',10,...                   % Elipse minor axis
    'circle_r',30,...                   % Circle radius
    'rect_half_length',4,...            % Half the length of the rectangle
    'rect_half_width',3,...             % Half the width of the rectangle
    'lmax',0,...                        % Half the side of the square wich will be analyzed locally (for reading)
    'xg',0,...                          % Points of area to be examined without rotation
    'yg',0,...                          % Points of area to be examined without rotation
    'npoints',0,...                     % Number of points of area to be examine
    ...     % Driver options
    'change_lane',[10 20],...           % Gap where driver can overtake
    ...     % Prints active (0=Off, 1=On)
    'printrsp',0,...                    % Print road setpoints
    'printcurrsp',0,...                 % Print the setpoint that the car is following
    'printrid',0,...                    % Print road ID
    'printcrosszone',0,...              % Print the cross zone
    'printstop',1,...                   % Prints stop points
    'printlaser',0,...                  % Prints laser of selected car
    'printsateliteposition',0,...       % To print satelites position on navigation time
    'printmag',0,...                    % To print magnets in scenary
    'printtime',0,...                   % Print simulation time
    'printonlineinfo',0,...             % Print online information l«such as accumulator state, etc...
    'printslipplace',0,...              % Print slip place if exist
    'printcommulistround',0,...         % Print the list of cars detected by the roundabout
    'printcommulistcross',0,...         % Print the list of cars detected by the cross
    'printcommugpsinfo',0,...           % Print a white '+' in simulator when a car receives a gps informantion and a blue '+' when car cannot receive gps information
    'printcommuvelprof',0,...           % Print a white 'o' in simulator when a car receives a velocity profile and a blue 'o' when car cannot receive
    'printcommunotify',0,...            % Print a white '^' in simulator when a car notification is received successfully and a blue '^' when notification is not received
    'printkinematic',1,...              % Print kinematics points and orientations for the selected followed car
    'printsecurity',0,...               % Print sucurity vectors of each vehicle
    'printchangelane',1,...             % Print the change lane manouver
    'printinfo',1,...                   % Print some info with the car
    'infotype',18,...                   % Type of the info will be ploted
    ...                                 ( 1-Origin and Destination, 2-Car ID, 3-v(Km/h), 4-(v,w), 5-Steering angle)
    ...                                 ( 6-GPS HDOP, 7-ID (cmdacc) + Cars sighted, 8- Accelaration comand+v(km/h), 9- Accelaration comand  )
    ...                                 ( 10-ID + List Communication, 11- Road, 12- [CmdAcc+ID+list], 13- ID + AccCmd)
    ...                                 ( 14 - Mode + ID (Mode->D/W/M), 15-Mode (D/W/M), 16- node commu, 17- ID (causeID), )
    ...                                 ( 18 - ID/(Mode->D/W/M)/(Mode->D/W/M)/(causeID)/v(Km/h))
    ...     % Trajectory setpoints
    'rw',3,...                          % Lane widht
    'rn',1,...                          % Number of roads
    'rd',4,...                          % Distance between setpoints
    'ri',0,...                          % Roads information (road)[1xi 2yi 3xf 4yf 5nsetp 6length 7ang 8intype 9endtype 10nextlaunch 11countdelay]
    ...                                 % In/EndType (0=End 1-In 2-Roundabout 3-Cross)
    'r',0,...                           % Roads Setpoints  [road setp [x y]]
    ...     % Roundabout setpoints
    'rrw',5,...                         % Lane widht in roundabout
    'rbd',[90/10 90/8],...              % Distance between setpoints
    'radiusI',13,...                    % Radius of inside lane of roundabout
    'radiusO',18,...                    % Radius of outside lane of roundabout
    'r1i',0,...                         % Outside lane information [1nsetp]
    'r1',0,...                          % Outside lane      [setp [x y]]
    'r2i',0,...                         % Inside lane information
    'r2',0,...                          % Inside lane       [setp [x y]]
    ...     % Routing/Probabilities Table
    'rtable',0,...                      % Routing Table
    'rp',0,...                          % Roads probability (road)[1exit 2exit ...]
    'rcp',zeros(28,28,2),...            % Roads current probability (road)[ [curpro1 ncars1] [curpro2 ncars2] ]
    'rl',[],...                         % Roads launch table [road; acumulator; next launch; last id; time between launchs; flag]
    ...     % Figures
    'figmenu',1,...
    'hfigmenu',-1,...
    'figsim',2,...
    'hfigsim',-1,...
    'hsim',-1,...
    'hott',0,...                        % Online text time
    'hoinfo',0,...                      % Online information
    'hbar',-1,...                       % Wait bar if simulation is in mode offline
    'hwaitoff',-1,...                   % Wait box for play simulation offline
    'hcommulistround',-1,...            % Handler to roundabout list
    'hcommulistcross',-1,...            % Handler to cross list
    'hinterface',-1,...                 % Handler to interface
    'hsema',[],...                      % Handler to plot traffic lights
    'hgraph1',[],...                    % Handler to plot lateral errors                        (fig3)
    'hgraph2',[],...                    % Handler to occupation map acumulator                  (fig4)
    'hgraph3',[],...                    % Handler to occupation map acumulator over the time    (fig5)
    'hgraph4',[],...                    % Handler to graph with gps and magnets errors          (fig6)
    'hgraph5',[],...                    % Handler to graph with INS and fusion errors           (fig7)
    'hgraph6',[],...                    % Handler to graph with odometry errors                 (fig8)
    'hgraph7',[],...                    % Handler to graph with Euclidean distance errors       (fig9)
    'hdisplay',[],...                   % Handler to figure with cars stats                     (fig10)
    ...     % Occupation grid properties
    'ocmap',0,...                       % Occupation map (uint8)
    'ocmap3',0,...                      % Used only in path extraction
    'occup_res',1,...                   % Resolution of the grid (EX: 2 is a resolution of 0.5m)
    'occup_size',2,...                  % Half the size analyzed locally for writing a car in the map (EX: 2 is a square with 4m side)
    'occup_map_accu',[],...             % Occupation map accumulator
    'ocmap_ind',[],...                  % Indices of cells by road [x,y,road]
    'ocmap_time',[],...                 % Number of cells ocupied and time [time ncells(road1) ncells(road2) ]
    ...     % Cars stats
    'cars_time_stop',[],...             % Array with stop time of each car
    'cars_time_stop_perc',[],...        % Array with stop time of each car in percentage
    'cars_time_run',[],...              % Array with run time of each car
    'cars_time_run_perc',[],...         % Array with run time of each car in percentage
    'cars_time_insim',[],...            % Average time of cars in simulator by distance traveled
    'cars_mean_vel',[],...              % Array with mean velocity during trajectory
    'cars_mean_run_vel',[],...          % Array with mean velocity (only when car is runing)
    'cars_exit',0,...                   % Cars deleted from simulator
    ...     % Position Errors
    'dla',5,...                         % Look-ahead distance
    'dlockaheadgain',15,...             % Look-ahead gain distance
    ...         % Maximum speeds (m/s)
    'velmax1',30,...                    % Km/h (after converted to m/s)
    'velmax2',40,...                    % Km/h (after converted to m/s)
    'velmax3',50,...                    % Km/h (after converted to m/s)
    ...     % Dynamic of cars
    'dynmaxacc',[5 -20],...                 % [maxVelAcc/s maxVelBrk/s]
    'dynmaxste',[180*pi/180 40*pi/180],...  % [MaxSteer xºRad/s xºRad]
    'velpercent',[0.5 0.4],...              % Percentage of maximum speed at [ roundabout cross ]
    ...     % Colors
    'obtc',[0 0 0],...                   % Background online text color
    'otc',[1 1 1],...                    % Online text color
    'cd1',[0.1 0.9 0.1],...              % Calm driver
    'cd2',[0.9 0.9 0],...                % Normal driver
    'cd3',[0.9 0.1 0.1],...              % Aggressive driver
    ...
    ...     % Fusion 4 Properties
    'fus4_noise_time',10,...            % Duration (in seconds) off noise vectors used in fus4
    'CT_CORR_INOV',1,...
    'CT_CORR',1,...
    'SLIP_WHEEL',1,...
    'CT_threshold',0.5,...
    'ekf_odo_threshold',[1e-2 4e-4 4e-4 4e-4 4e-4],...
    'slip_list',[],...                  % List of places with oil
    'slip_radius',1,...                 % Radius of oil circles
    ...     % Aux
    'aux1',[],...
    'aux2',[],...
    'offline',0,...
    ...     % Localization
    'xyz_origin',0,...                   % Local de origem em XYZ
    'round_center',[50 50],...           % Center of roundabout
    'cross_center',[115 50],...			 % Center of Crossroad
    'refposxRound',0,...                      % Reference station position in simulator (x) - Round
    'refposyRound',0,...                      % Reference station position in simulator (y) - Round
    'refposxCross',115,...                      % Reference station position in simulator (x) - Cross
    'refposyCross',50,...                      % Reference station position in simulator (y) - Cross
    'lt',40.11,...                                      % Latitude  Local -> ISR-UC
    'lg',-8.24 ...                                      % Longitude Loval -> ISR-UC
    );

% Convert speeds from Km/h to m/s
out.velmax1=out.velmax1/3.6;
out.velmax2=out.velmax2/3.6;
out.velmax3=out.velmax3/3.6;

% Local de origem em XYZ
if(out.gps_type==1)
    out.xyz_origin=llh2xyz([out.lt*pi/180 out.lg*pi/180 0]);    % Local de origem em XYZ
end

% Decide the step_time to simulation
if( ~(out.gps==0 && out.ins==0 && out.fus==0) )
    out.step_time=1/max([out.frqgps out.frqins]);
end

% Read necessary tables from xls files
out.rp=xlsread('ProbTable3.xls'); out.rp(:,1)=[]; out.rp(1,:)=[];
out.rtable=xlsread('RoutingTable.xls'); out.rtable(:,1)=[]; out.rtable(1,:)=[];
out.lighttable=xlsread('LightTable.xls');

%     % Entry Flux
%     % All Roads
%     out.rf=out.dlaunch*ones(28,1);

% Determine half the side of the square wich will be analyzed locally
out.lmax=max([out.elipse_M out.elipse_m out.circle_r sqrt(out.rect_half_length^2 + out.rect_half_width^2)]);

% Compute the points of area wich will be analyzed without rotation
out.xg=[]; out.yg=[];
if(out.zone_type==1)
    % Generate elipse points (generic)
    delta=0.3;
    for ang=0:delta:2*pi
        out.xg=[out.xg out.elipse_M*cos(ang)];
        out.yg=[out.yg out.elipse_m*sin(ang)];
    end
    out.xg=[out.xg out.xg(1)];
    out.yg=[out.yg out.yg(1)];
    out.npoints=length(out.xg);
elseif(out.zone_type==2)
    % Generate circle points
    delta=0.3;
    for ang=0:delta:2*pi
        out.xg=[out.xg out.circle_r*cos(ang)];
        out.yg=[out.yg out.circle_r*sin(ang)];
    end
    out.xg=[out.xg out.xg(1)];
    out.yg=[out.yg out.yg(1)];
    out.npoints=length(out.xg);
elseif(out.zone_type==3)
    % Generate rectangle points (generic)
    ll=out.rect_half_length; lw=out.rect_half_width;
    out.xg(1,1)=-ll; out.xg(1,2)=ll;  out.xg(1,3)=ll; out.xg(1,4)=-ll; out.xg(1,5)=-ll;
    out.yg(1,1)=-lw; out.yg(1,2)=-lw; out.yg(1,3)=lw; out.yg(1,4)=lw;  out.yg(1,5)=-lw;
    out.npoints=length(out.xg);
end

% Generate empty occupation map
out.ocmap=uint8(zeros(out.dimy*out.occup_res,out.dimx*out.occup_res));
out.ocmap3=double(zeros(out.dimy*out.occup_res,out.dimx*out.occup_res));
out.occup_map_accu=zeros(out.dimy*out.occup_res,out.dimx*out.occup_res);

% Half the size analyzed locally for writing a car in the map
if(out.occup_size2==0)
    aux1=out.TruDim(1)+out.layer_w; aux2=out.TruDim(2)+out.layer_l;
    out.occup_size2=ceil(sqrt(aux1^2+aux2^2));
end

% Table that will count how many cars are sent to a particular output
out.rg_count_tab(1:10,8)=0;
out.rg_count_tab(1,1:8)=[15 16 17 18 19 20 27 28];    % Exit from roundabout
out.rg_count_tab2(1:10,8)=0;
out.rg_count_tab2(1,1:8)=[13 14 22 21 23 24 25 26];            % Exit from crossroads

% Wheel parameters
out.wheel_radius=out.wheel_diam;

end
