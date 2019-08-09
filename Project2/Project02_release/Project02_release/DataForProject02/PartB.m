%Author: Po-Heng Chen , Z5014392
%Program: Solution for RD, S1.2018, Project01.PartB
% =========================================================================
function main()
    load('Speed_dataC');
    load('IMU_dataC.mat');

    times = double(Vel.times)/10000;
    times = times - times(1);
    %.....................................
    speed = Vel.speeds;
    N     = Vel.N ;
    %.....................................
    %ii = find(times < 20) ;
    ii = times < 20 ;
    bias = mean(IMU.DATAf(6,ii)) ;
    yaw_rate = IMU.DATAf(6,:) - bias ;
    %.....................................
    Pose = zeros(3,N-1);

    X0 = [0;0;pi/2] ; %Initial Condition (x, y, heading), in(meters, meters, radians)
    X = X0;
    for i = 1:N-1 %iterations / Loop
        X = PredictVehiclePose(X, yaw_rate(i), speed(i) ,times(i+1)-times(i));
        Pose(:,i) = X ;
    end;

    figure(1) ; clf() ; grid on ;
    plot(Pose(1,:),Pose(2,:)) ; xlabel('X (m)'); ylabel('Y (m)'); title('Position');
    %figure(2) ; clf ; plot(Pose(3,:)) ; %heading
return
end

% =============== Kinematic Model based on Angular Rate ===================
function X = PredictVehiclePose(X0,yaw_rate,speed,dt)
    % Remember: state vector X = [x; y; heading]
    LCar = 0.46 ; %in this example L = 0.46meters
    X=X0 ;
    dL = dt*speed ;
    X(3) = X0(3) + dt * (-yaw_rate);  % yawA = -yawB
    %X(3) = X0(3)+ tan(yaw_rate)*dL/LCar ;
    X(1:2) = X0(1:2)+dL*[ cos(X0(3));sin(X0(3))] ;
return ;
end
%.......................................................................%
