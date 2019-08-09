% Example using the function PredictVehiclePose().
% It implements a dead-reckoning prediction (i.e. kinematic model based on current speed
% and steering).
%.......................................................................
function main()
    Dt=0.05 ; % Step, for Euler approximation: 50ms (expressed in seconds)
    duration = 100 ; % Simulation horizon
    times = [0:Dt:duration];
    N=length(times) ;
    %.....................................
    speed = ones(N,1);
%     steering = (times)*pi/180; % (behavior of certain crazy driver)
    steering = (2+ 20*cos(times*2*pi/20))*pi/180; % (behavior of certain crazy driver)
    % In the arrays “speed[]” and “steering[]”
    % you can define the time evolution of the speed & steering inputs (it is a simulation!)
    %.....................................
    % buffer for storing simulated positions
    Pose=zeros(3,N);
    X0 = [0;0;0] ; %Initial Condition (x, y, heading), in(meters, meters, radians)
    X=X0;
    for i=1:N, %iterations / Loop
        X = PredictVehiclePose(X, steering(i), speed(i) ,Dt);
        Pose(:,i)=X ;
    end
    % Show the results.
    figure(1) ; clf() ;
    plot(Pose(1,:),Pose(2,:)) ; xlabel('X (m)'); ylabel('Y (m)'); title('Position');
    figure(2) ; clf ; 
    plot(Pose(3,:)) ; %heading
return;
end 
%......................................................................% "X=PredictVehiclePose(x0, steering, speed,dt)"
% X0 current state ( [x; y; heading] )
% X estimated next state ( [x; y; heading] at time t+dt)
% speed : current speed (m/s)
% steering : current steering angle (at time t)(in radians)
% dt is the "integration" horizon (should be a fraction of second)
% Jose Guivant - For AAS
% Tricycle / Ackermann model, discrete version
function X = PredictVehiclePose(X0,steering,speed,dt)
    % Remember: state vector X = [x; y; heading]
    LCar=3 ; %in this example L=3meters
    X=X0 ;
    X(3) = X0(3)+ dt*(speed/LCar)*tan(steering) ;
    X(1:2) = X0(1:2)+dt*speed*[ cos(X0(3));sin(X0(3))] ;
return ;
end
