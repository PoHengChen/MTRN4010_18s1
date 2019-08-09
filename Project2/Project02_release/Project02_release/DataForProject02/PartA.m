%Author: Po-Heng Chen , Z5014392
%Program: Solution for RD, S1.2018, Project02.PartA
%..................................................................
function yaw = PartA(file)
    if ~exist('file','var'), file = 'IMU_dataC.mat' ; end;
    load(file) ;
    times = double(IMU.times)/10000; % (1 count = 0.1 ms) => (10000 counts = 1 s)
    times = times - times(1) ;  % just, in order to refer to t0=0 (not necessary).
    %rates_yaw = IMU.DATAf(6,:);
    k = 180/pi;
    
    ii = times < 20 ;
    bias = mean(IMU.DATAf(6,ii)) ;
    IMU.DATAf(6,:) = IMU.DATAf(6,:) - bias ;
    
    Attitude = zeros(3,length(times)-1);
    Attitude(:,1) = [0 0 pi/2];
    for i = 1:IMU.N - 1
        Attitude(:,i+1) = IntegrateOneStepOfAttitude( IMU.DATAf(4:6,i), times(i+1)-times(i), Attitude(:,i) ) ;
    end;
    
    figure(1) ;  clf() ; hold on ; grid on ; zoom on ;
    plot(times , IMU.DATAf(6,:)* k , 'b');
    title('yaw rate');
    xlabel('time (in seconds)'); ylabel('angular rates (degrees/sec)');

    figure(2) ;  clf() ; hold on ; grid on ; zoom on ;
    plot(times , Attitude(3,:)*k , 'b');
    title('Yaw (in Axis Convention B)');
    xlabel('time (in seconds)'); ylabel('yaw (degrees)');
    
    yaw = Attitude(3,:)*k;

end

function NewAttitude  = IntegrateOneStepOfAttitude( gyros, dt, CurrentAttitude ) 

    ang = CurrentAttitude ;  % current global Roll, Pitch, Yaw  (at time t) 
    wx = 0;   %local roll rate 
    wy = 0;   %local pitch rate 
    wz = gyros(3);   %local yaw rate

    %----------------------------------
    cosang1=cos(ang(1)) ; 
    cosang2=cos(ang(2)) ; 
    sinang1=sin(ang(1)) ; 

    roll    = ang(1) + dt * (wx + (wy*sinang1 + wz*cosang1)*tan(ang(2))) ; %(*) 
    % pitch	= ang(2) + dt * (wy*cosang1 - wz*sinang1) ; % as wy = 0, the cummulative sign can be both positive or negative
    pitch	= ang(2) - dt * (wy*cosang1 - wz*sinang1) ; 
    yaw     = ang(3) - dt * ((wy*sinang1 + wz*cosang1)/cosang2)    ; %(*)
     
    %----------------------------------

    NewAttitude= [roll,pitch,yaw]';  % new global Roll, Pitch, Yaw (at time t+dt) 
return;
end