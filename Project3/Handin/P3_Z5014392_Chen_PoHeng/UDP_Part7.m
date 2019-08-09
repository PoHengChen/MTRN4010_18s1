

function main()
  clc;clear;
  fprintf('Running an example UPD reader, for MTRN41010\n');  
  
  x = instrfind('type','udp'); delete(x);  %delete all UDP instances.
  % you may try to delete just those having conflicting input port
    
 PortRx = 1112;    
 uRx = udp('127.0.0.1','LocalPort',PortRx);
 set(uRx,'InputBufferSize',10000,'Timeout',1.2,'DatagramTerminateMode','on'); 
 fopen(uRx); 
 warning('off', 'instrument:fread:unsuccessfulRead');
    
cx1=0; t1=0; cx2=0;
tic();
initial();
initialPlot();
while 1
    xr=uint8(fread(uRx,10000,'uint8'));
    if (numel(xr)>0)     %number of array element > 0
        t1=0;  cx1=cx1+1; 
        ProcessData(xr);         %<----------- you process each message, here.
    else
        t1=t1+1.2;
        if (t1>4), fprintf('Too much time not having activity...\n');   break ; end; %timeout
    end;    
    
    if (toc()>1)   % print, at ~1HZ
        tic();  cx2=cx2+cx1; 
        if (cx1>0) 
            fprintf('Messages: new=[%d],total=[%d]\n',cx1,cx2);
        else
            fprintf('Messages: NO new arrivals.\n');
        end;
        
        cx1=0;
    end;    
    
end;    

fprintf('Closing UDP  port; BYE\n');
fclose(uRx) ; delete(uRx);
return;

end


function ProcessData(x)
    global cur;
    cur.UDP_DATA_ID = double(uint8(x(2)));
    
    time = double(typecast(uint8(x(9:12)),'uint32')) / 10000;
    if cur.UDP_DATA_ID == 31       % Dead-Reckoning (DR)(Longitudinal velocity and GyroZ).
        cur.poseFlag = 1;
        cur.speed = double(typecast((x(13:14)),'int16'))/1000;
        cur.speed_array = [cur.speed_array,cur.speed];
        cur.yaw_rate = double(typecast((x(15:16)),'int16'))*0.02*(pi/180);
        cur.yaw_rate1 = cur.yaw_rate - cur.bias; %
        if cur.checkTimeFlag == 0
            cur.initialTime = time;
            cur.times_imu = time;
            cur.checkTimeFlag = 1;
        end
        cur.Dt = time - cur.times_imu;
        cur.times_imu = time;
    elseif cur.UDP_DATA_ID == 26   % LIDAR
        cur.laserFlag = 1;
        scan = [];
        for i = 1:361
           if isempty(scan)
               scan(1,:) = typecast(uint8(x(15:16)),'uint16');
           else
               scan = [scan(1,:),typecast(uint8(x(13+2*i:14+2*i)),'uint16')];
           end
        end
        cur.scan = scan';
        cur.times_laser = time;
    else
    end
	Part7();
end

function initialPlot()
figure(8) ; clf() ; hold on ; grid on ; zoom on ;
    xlabel('X (m)') ; ylabel('Y (m)') ; axis([-10 7 -1 8]);%axis([-10,10,0,20]) ;%axis([-10 7 -1 8]) ;
    global MyGUIHandle;
    
    MyGUIHandle.handle0 = plot(0,0,'r.','MarkerSize',1);
%     MyGUIHandle.handle1 = quiver3(0,0,0,0,'Color','r','LineWidth',2); % robot position
    MyGUIHandle.handle2 = plot(0,0,'r*','MarkerSize',6); % obstacle

    MyGUIHandle.handle87 = plot(0,0,'b.','MarkerSize',1);
%     MyGUIHandle.handle89 = quiver3(0,0,0,0,'Color','b','LineWidth',2); % robot position
    MyGUIHandle.handle88 = plot(0,0,'b*','MarkerSize',6); % obstacle

   
    MyGUIHandle.handle3 = plot(0,0,'k+','MarkerSize',15); % landmark

    MyGUIHandle.handle6 = title('');      % empty title (update synchronizly with pole update)
    MyGUIHandle.handle7 = text(0,0,'');
    MyGUIHandle.handle8 = text(0,0,'');
    MyGUIHandle.handle9 = text(0,0,'');
    MyGUIHandle.handle10 = text(0,0,'');
    MyGUIHandle.handle11 = text(0,0,'');
    
figure(55) ; clf() ; 
    xlabel('i'); ylabel('radian');title('Bias');
    MyGUIHandle.handle65 = plot(0,0);
figure(77) ; clf() ;  % estimated speed plot
    xlabel('i'); ylabel('radian');title('Speed');
    MyGUIHandle.handle77 = plot(0,0,'b');
figure(99) ; clf() ;  % sensor speed plot
    MyGUIHandle.handle99 = plot(0,0,'r.','MarkerSize',15);
%     MyGUIHandle.handle787 = plot(0,0,'r.');
return
end

function initial()
    global cur;
    cur.Xdr = [0; 0; pi/2;0;0];
    cur.Xdr_History = cur.Xdr;
    cur.Xe  = [0; 0; pi/2;0;0];
    cur.Xe_History  = cur.Xe;
    
    cur.P = zeros(5,5);
    cur.P(4,4) = (2*pi/180)^2; 
    cur.P(5,5) = 0;
    cur.stdDevGyro = 1.5*pi/180 ;        
    cur.stdDevSpeed = 0.05 ; 
    cur.sdev_rangeMeasurement = 0.15 ; 
    cur.sdev_BearingMeasurment = 1*pi/180;
    cur.Q_Process_Model = diag( [ (0.1)^2 ,(0.1)^2 , (1*pi/180)^2,0,(cur.Dt*1.5)^2]) ; % Q_model 
    cur.Q_Input = diag([ cur.stdDevSpeed*cur.stdDevSpeed , cur.stdDevGyro*cur.stdDevGyro]);
    
    cur.bias = -pi/180;
    cur.landmarkSetFlag = 0;
    cur.checkTimeFlag = 0;
    cur.i = 0;
    cur.N_imu = 44578;
    cur.estimated_speed = 0;
    cur.speed_array = 0;
    
    return
end

