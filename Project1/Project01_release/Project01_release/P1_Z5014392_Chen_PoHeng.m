% Edited by - Po-Heng Chen , Z5014392   
% 21/03/2018   Wednesday    Week4

% Program: Solution for AAS, S1.2018, Project01.Part1a and Part1b

function MyProgram(DataFileName)

clc(); close all;
if ~exist('DataFileName','var'), DataFileName ='Laser__2.mat'; end;
load(DataFileName);                                      

figure(7); clf(); hold on; zoom on; axis([-10,10,0,20]);
title('Cartesian Plotting'); xlabel(''); ylabel('');

MyGUIHandles.handle1 = plot(0,0,'b.');
MyGUIHandles.handle2 = plot(0,0,'r*');
MyGUIHandles.handle3 = plot(0,0,'g+');

N = dataL.N;  
disp('Dynamic plotting');

for i=1:N,     % i representing which scan it is          
    scan_i = dataL.Scans(:,i);
    %t =  double(dataL.times(i)-dataL.times(1))/10000;
    ProcessScan(scan_i,MyGUIHandles,i);
    
    s=sprintf('(Dynamic plotting...)\nShowing scan #[%d]/[%d]\r',i,N);
    title(s);
    
    pause(0.0025) ;                   % wait for ~10ms
end;
disp('Done. Bye.');

return;
end

function ProcessScan(scan,mh,i)

% Declare two Mask to extract ranges and intensities
mask1FFF = uint16(2^13-1);
maskE000 = bitshift(uint16(7),13)  ; % shift 0111 to left 13 bits

intensities = bitand(scan,maskE000);
ranges    = single(bitand(scan,mask1FFF))*0.01; 

% Transfer Polar form to Cartesian form
angles = [0:360]'*0.5* pi/180 ;       
X = cos(angles).*ranges;
Y = sin(angles).*ranges;   

set(mh.handle1,'xdata',X,'ydata',Y);
ii = find(intensities~=0);
set(mh.handle2,'xdata',X(ii),'ydata',Y(ii));

% Initialize variabels
count = 0;                  % number of pixel in each cluster
j = 0;                      % number of cluster in each scan
Cluster  = zeros(4,361);    % preallocate memory for Cluster matrix (4 * 10)
for i = 1:length(ranges)-1
    
    % Check if next pixel is in the same cluster
    if abs(ranges(i)-ranges(i+1)) > 0.1  
        OOIs = ExtractOOIs(ranges(i-count:i),intensities(i-count:i),X(i-count:i),Y(i-count:i)) ;
        j = j + 1;
        Cluster(1,j) = OOIs.X;
        Cluster(2,j) = OOIs.Y;
        Cluster(3,j) = OOIs.N;
        Cluster(4,j) = OOIs.Sizes;
        count = 0;
    else
        count = count + 1;
    end
end
% Plot the cluster which are brilliant
brilliant = find(Cluster(3,:)~=0);
set(mh.handle3,'xdata',Cluster(1,brilliant) ,'ydata',Cluster(2,brilliant) );

return;
end

 function r = ExtractOOIs(ranges,intensities,X,Y)
 
     % Brightness
     r.N = 0;
     if sum(intensities)~=0, r.N = 1; end
     
     % Center
     r.X = sum(X)/length(X);
     r.Y = sum(Y)/length(Y);
     r.Centers = [r.X r.Y];
     
     % Diameter
     radius = zeros(1,10000);
     for i = 1:length(X)
         radius(i) = sqrt((r.X-X(i))^2+(r.Y-Y(i))^2) ;
     end
     r.radius = sum(radius)/length(X);
     r.Sizes   = r.radius*2;
     
 return;
 end
 