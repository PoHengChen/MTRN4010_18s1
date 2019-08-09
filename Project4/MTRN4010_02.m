function [] = MTRN4010_02()
% fuzzy control
% sim car movement to target, not guaranteed

clc; clear all; close all; dbstop if error; warning off;
set(0,'defaultaxesfontname','times new roman');

field.range = 50;
time.dt = 1; time.T = 800;

[fig] = FigureNew(field);
[car] = CarNew(field);
[car] = CarNow(car,time,0,0);
[car] = CarShow(fig,car,0);
[target] = TargetNew(field);

fis_vel = readfis('MTRN4010_vel.fis');
fis_ang = readfis('MTRN4010_ang.fis');

for t=0:time.dt:time.T,
  [ds] = FindDistance(car,target);
  vel = evalfis(ds,fis_vel);
  [dq] = FindAngular(car,target);
  ang = evalfis(dq,fis_ang);
  [car]=CarNow(car,time,vel,ang);
  [car]=CarShow(fig,car,t);
end;

function [target] = TargetNew(field)
target.x=20; target.y=40; target.q=pi/2;
% target.trace=[target.x; target.y; target.q];
target.shape=[ 2 0; 1 1; -1 1; -1 -1; 1 -1; 2 0]';
target.hdL.shape=plot(target.shape(1,:),target.shape(2,:),'color','r','linewidth',2);
% target.hdL.trace=plot(target.trace(1,:),target.trace(2,:),'color',[0 0.66 0]);
Rz=[  cos(target.q) -sin(target.q); 
      sin(target.q)  cos(target.q)];
shape=Rz*target.shape+repmat([target.x;target.y],1,6);
set(target.hdL.shape,'xdata',shape(1,:),'ydata',shape(2,:)); 

% function [ds] = FindDistance(car,target)
% ds = sqrt((car.x-target.x)^2+(car.y-target.y)^2);
% % ds = pdist([car.x,car.y;target.x,target.y]);
% 
% function [dq] = FindAngular(car,target)
% dq = atan2(target.y-car.y,target.x-car.x) - car.q;
% % dq = abs(target.q - car.q);
function [ds]=FindDistance(car,target)
dx=target.x-car.x;
dy=target.y-car.y;
ds=sqrt(dx^2+dy^2);

function [dq]=FindAngular(car,target)
dx=target.x-car.x;
dy=target.y-car.y;
q=atan2(dy,dx)-car.q;
dq=AngleWrap(q);

function [q]=AngleWrap(q)
while q<pi,
    q=q+2*pi;
end;
while q>pi,
    q=q-2*pi;
end;

function [fig] = FigureNew(field)
fig.fig = figure('units','normalized','position',[0.3 0.2 0.5 0.5]);
axis([-1 1 -1 1]*field.range); hold on; grid on; axis equal;
xlabel('x-direction'); ylabel('y-direction');
fig.ax = axis;

function [car] = CarNew(field)
car.x=0; car.y=0; car.q=0;
car.trace=[car.x; car.y; car.q];
car.shape=[ 2 0; 1 1; -1 1; -1 -1; 1 -1; 2 0]';
car.hdL.shape=plot(car.shape(1,:),car.shape(2,:),'color','b','linewidth',2);
car.hdL.trace=plot(car.trace(1,:),car.trace(2,:),'color',[0 0.66 0]);

function [car] = CarNow(car,time,v,w)
car.x=car.x+time.dt*v*cos(car.q);
car.y=car.y+time.dt*v*sin(car.q);
car.q=car.q+time.dt*w;
car.trace(:,end+1)=[car.x; car.y; car.q];

function [car] = CarShow(fig,car,t)
ax=axis;
Rz=[  cos(car.q) -sin(car.q); 
      sin(car.q)  cos(car.q)];
shape=Rz*car.shape+repmat([car.x;car.y],1,6);
set(car.hdL.shape,'xdata',shape(1,:),'ydata',shape(2,:)); 
set(car.hdL.trace,'xdata',car.trace(1,:),'ydata',car.trace(2,:));
axis(ax); title(sprintf('Time %d',t)); pause(0.001);


