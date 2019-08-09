% Basic example, for reading UDP messages
% for MTRN4010.

% read document "StructureMessages-UDP_4010[2018].pdf" for details
% about the message format.

% Run "PossumSimulator09_UDP.exe", if you want to play data back, for
% testing your program.



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
    cur.UDP_data_ID = double(x(2));
    if cur.UDP_data_ID == 31
       time = double(typecast(uint8(x(9:12)),'uint32')) / 10000 ;
       if cur.checkTimeFlag == 0
           cur.time = time;
           cur.initialtime = cur.time;
           cur.checkTimeFlag = 1; 
       end
       cur.Dt = time - cur.time;
       cur.time = time;
       cur.speed = double(typecast((x(13:14)),'int16'))/1000;
       cur.yaw_rate = double(typecast((x(15:16)),'int16'))*0.02*(pi/180);
       cur.yaw_rate = cur.yaw_rate + pi/180; %- cur.bias;
       kk();
    end
    

end

function initialPlot()
    figure(8) ; clf() ; hold on ; grid on ; zoom on ;
    xlabel('X (m)') ; ylabel('Y (m)') ; axis([-3.5 1.5 -1 5]);%axis([-10,10,0,20]) ;%axis([-10 7 -1 8]) ;
    global MyGUIHandles;
    MyGUIHandles.handle1 = plot(0,0,'b.'); % blue pixel
    MyGUIHandles.handle2 = plot(0,0,'r*'); % bright pixel
    MyGUIHandles.handle3 = plot(0,0,'g+'); % center of cluster
    MyGUIHandles.handle4 = plot(0,0,'k.'); % vehicle
return
end

function initial()
    global cur;
    cur.Xdr = [0; 0; pi/2];
    cur.Xdr_History = cur.Xdr;
    cur.bias = -0.0169;
    cur.checkTimeFlag = 0;

return
end
