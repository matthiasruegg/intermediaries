%% Two chamber moving wall experiment
diary on
diary('logBox71.txt')

%% Load PI MATLAB Driver GCS2
addpath('C:\Users\Public\PI\PI_MATLAB_Driver_GCS2'); 

if (~exist('Controller', 'var') || ~isa(Controller, 'PI_GCS_Controller'))
    Controller = PI_GCS_Controller ();
end;

%% Connect PI controller
boolPIdeviceConnected = false;
if (exist ('PIdevice', 'var'))
    if (PIdevice.IsConnected)
        boolPIdeviceConnected = true; 
    end
end

if (~(boolPIdeviceConnected))
    controllerSerialNumber = '117066693';
    PIdevice = Controller.ConnectUSB(controllerSerialNumber);
end

% initialize PIdevice object
PIdevice = PIdevice.InitializeController();

%% Show connected stages
availableAxes = PIdevice.qSAI_ALL;

% get Name of the stage connected to axis 1
PIdevice.qCST('1');

% Show for all axes: which stage is connected to which axis
for idx = 1 : length(availableAxes)
    stageName = PIdevice.qCST(availableAxes{idx});
    disp (['Axis ', availableAxes{idx}, ': ', stageName ]);
end

% Startup Stage
axis = '1';

% switch servo on for axis
PIdevice.SVO(axis, 1);

% reference axis: this will move the axis along the entire range!
velocity = 20.0000;
PIdevice.VEL(axis, velocity);

PIdevice.FRF(axis);
bReferencing = 1;                           
disp('Stage 1 is referencing')
while(0 ~= PIdevice.qFRF(axis) == 0)                        
    pause(0.1);           
    fprintf('.');
end

%% Start experiment
% determine the allowed travel range of the stage
minimumPosition = PIdevice.qTMN(axis);
maximumPosition = 68; % Wall is about 70mm long
travelRange = maximumPosition - minimumPosition;

velocity = 1.0000; % [mm/sec]
PIdevice.VEL(axis, velocity);
Position = 0; % [mm]
PIdevice.MOV(axis, Position);

fileID = fopen('PosTimeBox71.txt','w');

% method 2
duration_1way = 3600*24*2; % [sec]
movStep = 1; % [mm]
nbMvt = maximumPosition / movStep;

StartTime = datetime('now');
EpochStartTime = posixtime(StartTime);

n = 1; % number of go-and-returns
format long g

pause(3600*24*6)

% go and return loop
for N = 1:n
    
    k = 0;
    while Position <= maximumPosition - movStep
        PIdevice.MVR(axis, movStep)
    
        time = datetime('now');
        Epochtime = posixtime(datetime('now'));
        Position = PIdevice.qPOS (axis);
    
        pause(duration_1way /  nbMvt);
        k = k+1;
    
        x = [Position Epochtime];
        fprintf(fileID,'%d %4.4f\n',x);
    end

    LongestPathTime = datetime('now');
    EpochLongestPathTime = posixtime(LongestPathTime);
    pause(3600*24*6) % pause at Lmax
    
    for R = 1:k
        PIdevice.MVR(axis, -movStep)
    
        time = datetime('now');
        Epochtime = posixtime(datetime('now'));
        Position = PIdevice.qPOS(axis);
    
        pause(duration_1way /  nbMvt)
    
        x = [Position Epochtime];
        fprintf(fileID,'%d %4.4f\n',x);
    end
    
 pause(3600*24*6)   
end

positonReached = PIdevice.qPOS(axis);

EndTime = datetime('now');
EpochEndTime = posixtime(EndTime);

%% Close controller connection and cleanup
PIdevice.CloseConnection ();

Controller.Destroy ();
clear Controller;
clear PIdevice;

diary off