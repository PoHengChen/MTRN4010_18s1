% Basic example, for reading UDP messages
% for MTRN4010.

% read document "StructureMessages-UDP_4010[2018].pdf" for details
% about the message format.

% Run "PossumSimulator09_UDP.exe", if you want to play data back, for
% testing your program.



function main()
  clc;
  fprintf('Running an example UPD reader, for MTRN41010\n');  
  
  x = instrfind('type','udp'); delete(x);  %delete all UDP instances.
  % you may try to delete just those having conflicting input port
    
 PortRx = 1112;    
 uRx = udp('127.0.0.1','LocalPort',PortRx);
 set(uRx,'InputBufferSize',10000,'Timeout',1.2,'DatagramTerminateMode','on'); 
 fopen(uRx); 
 warning('off', 'instrument:fread:unsuccessfulRead');
    
cx1=0;t1=0;cx2=0;
tic();
while 1,
   xr=uint8(fread(uRx,10000,'uint8'));
   if (numel(xr)>0),  
       t1=0;  cx1=cx1+1; 
       ProcessData(xr);         %<----------- you process each message, here.
   else
      t1=t1+1.2;
      if (t1>4), fprintf('Too much time not having activity...\n');   break ; end; %timeout
    end;    
    
    if (toc()>1),   % print, at ~1HZ
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
    % your part...



end
