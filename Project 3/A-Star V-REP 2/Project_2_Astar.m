%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% David A Smart
% University of Maryland, College Park
% ENPM 661 - Planning for Autonomous Systems
% Project #2 - Path Planning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% search or simulate?
dosearch = 0;
dovrepsim = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if dosearch %% A* Search 
    % set resolution
    ress = 50; % percent (for expanding nodes)
    res = ress2res(ress)*0.95; % m (for eliminating "duplicates")
    pause(1)

    % ***select start and goal****************************
    StartNode = [6.6028,4.1492,0, 0,0,0, 0,0]; % [x,y,theta, dx,dy,dtheta, ul, ur]
    StartNode = [-6,1,-90, 0,0,0, 0,0];
    % % generate random end point
    % Crash_Goal = 1; dxy = 0;
    % while Crash_Goal || dxy < 5
    %     GoalNode = [randi([-7,7]),randi([-5,5])];
    %     dx = abs(StartNode(1)-GoalNode(1));
    %     dy = abs(StartNode(2)-GoalNode(2));
    %     dxy = sqrt(dx^2+dy^2);
    %     % check if in an obstical
    %     Crash_Goal = EvalCrash(GoalNode);
    % end
    % 
    % clear Crash_Start Crash_Goal inObs dx dy dxy

    GoalNode = [-1,-2,0, 0,0,0, 0,0];

    % set angles
    GoalNode(3) = 0;
    StartNode(3) = 180;

    % search entire space for optimal path
    [xytheta,dxdydtheta,vlvr] = Astar_Search (StartNode, GoalNode, res, ress);

    % save text-file
    % diffrent format

    % % world frame velocities
    % ldx = dxdydtheta(:,1);
    % ldy = dxdydtheta(:,2);
    % adz = dxdydtheta(:,3)*(pi/180);

    % robot frame velocities
    r = (72/2)/1000; % m (wheel radius)
    L = 235/1000; % m (distance between wheels)
    % wheel angular velocities
    ul = vlvr(:,1); % rad/s
    ur = vlvr(:,2); % rad/s
    % wheel linear velocities
    vl = ul*r; % m/s
    vr = ur*r; % m/s
    ldx = (vr+vl)/2; % linear forward (m/s)
    ldy = zeros(length(dxdydtheta),1);
    adz = (vr-vl)/L; % angular (rad/s)
    % adz = adz*60/2/pi; % angular (rpm: 2*pi rad / 60 s)

    ldz = zeros(length(dxdydtheta),1);
    adx = zeros(length(dxdydtheta),1);
    ady = zeros(length(dxdydtheta),1);

    la = [ldx,ldy,ldz,adx,ady,adz*1.25]; % file format
    save('Output.mat','xytheta','dxdydtheta','vlvr','la'); % save for later

    % write file
    delete *.txt
    fid = fopen('Output.txt','w');
    fprintf(fid,'linear.x = %f, linear.y = %f, linear.z = %f, angular.x = %f, angular.y = %f, angular.z = %f \r\n',la');
    fclose(fid);
end

if dovrepsim %% simulate in V-REP by sending wheel velocities
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    load('Output.mat'); % load in rather than recomputing

    % set velocity values
    left_targetVelocity = vlvr(:,1);
    right_targetVelocity = vlvr(:,2);
    
    % connenct to V-REP
    vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID = vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
    if (clientID > -1)

        fprintf('Connected to V-REP!\n');

        % get the wheel motor handles
        [~,left_wheelmotorhandle] = vrep.simxGetObjectHandle(clientID,'wheel_left_joint',vrep.simx_opmode_blocking);
        [~,right_wheelmotorhandle] = vrep.simxGetObjectHandle(clientID,'wheel_right_joint',vrep.simx_opmode_blocking);

        i = 0; % initilize counter
        
        % set up a timer
        % tic-toc does not work correctly...b/c "mod" has to be exact
        % i tested it and what should take 30 s takes 75 s w/o v-rep commands! 
        t = timer;
        t.Period = 0.5;
        t.ExecutionMode = 'fixedRate';
        t.TasksToExecute = length(vlvr);
        t.TimerFcn = '[i] = sendvrepcommand(left_targetVelocity,right_targetVelocity,i,vrep,clientID,left_wheelmotorhandle,right_wheelmotorhandle);';
        % {@sendvrepcommand,left_targetVelocity,right_targetVelocity,i,...
        %    vrep,clientID,left_wheelmotorhandle,right_wheelmotorhandle};
        t.BusyMode = 'queue';
        
        %tic
        start(t); % execute!!!
        wait(t)
        stop(t);
        %disp(toc)
        delete(t);

        % make sure to make the robot stop...
        % otherwise it keeps exicuting the last sent command!
        % set wheel motor speeds
        vrep.simxSetJointTargetVelocity(clientID,left_wheelmotorhandle,0,vrep.simx_opmode_blocking);
        vrep.simxSetJointTargetVelocity(clientID,right_wheelmotorhandle,0,vrep.simx_opmode_blocking);

    else
        disp('Failed to connect to V-REP');
    end
    % disconect from V-REP
    vrep.simxFinish(clientID); % close the line if still open
    vrep.delete(); % call the destructor!
    disp('Program ended');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
