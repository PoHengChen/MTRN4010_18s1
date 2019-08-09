function [] = MTRN4010_91()
% sim car movement to target
% fuzzy control on velocity/angle
% PSO optimize zero membership function parameters
% modify vel.fis output membership ZERO upper bound
% modify ang.fis output membership ZERO lower/upper bound
% obective function is square rooted sum of pose error squares
% because of many simulations, the programm take a longer time to finish

% program management
clc; clear all; close all; dbstop if error;
set(0,'defaultaxesfontname','times new roman');

% car motion filed, sim time
field.range=50;
time.dt=1; time.T=500;

% car variables
[fig]=FigureNew(field);
[car]=CarNew(field);
[car]=CarNow(car,time,0,0);
[car]=CarShow(fig,car,0);
[target]=TargetNew(field);

% import fuzzy system
fis_vel=readfis('MTRN4010_vel.fis');
fis_ang=readfis('MTRN4010_ang.fis');

% PSO parameters
PSO.VLB=0.1; PSO.VUB=2;% car velocity lower/upper bound
PSO.ALB=0.1*pi; PSO.AUB=0.8*pi;% car angle lower/upper bound
PSO.D=2; PSO.G=5; PSO.N=30;% particle dimension generations, particles
PSO.V=rand(PSO.D,PSO.N);% initial PSO particle velocity
PSO.Gbest=[]; PSO.gbest=realmax;% PSO gbest
PSO.Pbest=[]; PSO.pbest=ones(1,PSO.N)*realmax;% PSO pbest
PSO.w1=0.9; PSO.w2=0.4; PSO.dw=PSO.w1-PSO.w2;% init and final inertia weight
PSO.cg=2; PSO.cp=2;% social, congnitive factor
PSO.X=rand(PSO.D,PSO.N); PSO.BND=[PSO.VLB PSO.VUB; PSO.ALB PSO.AUB];% PSO bound
for d=1:2,% particles for car vel/angle Z-MF
PSO.X(d,:)=PSO.BND(d,1)+PSO.X(d,:)*diff(PSO.BND(d,:));
end;

% PSO main loop
carInit=car;% initial car pose
for g=1:PSO.G,% PSO generation
    disp(PSO.X);% particles
    car=carInit;% restore init car pose
    for n=1:PSO.N,% PSO particles
        v=PSO.X(1,n); a=PSO.X(2,n);% velocity and angular bounds
        fis_vel.output.mf(1).params(3)=v;% modify vel output Z-MF
        fis_ang.output.mf(2).params(3)=a;% modify ang output Z-MF
        fis_ang.output.mf(2).params(1)=-a;
        
        % sim car movement
        for t=0:time.dt:time.T,
            [ds]=FindDistance(car,target);
            vel=evalfis(ds,fis_vel);
            [dq]=FindAngular(car,target);
            ang=evalfis(dq,fis_ang);
            [car]=CarNow(car,time,vel,ang);
        end;
        
        err=[car.x-target.x car.y-target.y AngleWrap(car.q-target.q)];% pose error
        fit(n)=sqrt(sum(err.^2));% objective function
        
        % find global best
        if fit(n)<PSO.gbest,
            PSO.gbest=fit(n);
            PSO.Gbest=PSO.X(:,n);
        end;
        
        % find particle best
        if fit(n)<PSO.pbest(n),
            PSO.pbest(n)=fit(n);
            PSO.Pbest(:,n)=PSO.X(:,n);
        end;
    end;
    disp(sprintf('Generation %d Gbest %5.3f %5.3f gbest %5.3f',...
    g,PSO.Gbest.',PSO.gbest));% currently best results
    % PSO update
    w=PSO.w2+(1-g/PSO.G)*PSO.dw;% decreasing inertia weight
    PSO.V=w*rand(PSO.D,PSO.N).*PSO.V+...
    PSO.cp*rand(PSO.D,PSO.N).*(PSO.Pbest-PSO.X)+...
    PSO.cg*rand(PSO.D,PSO.N).*(repmat(PSO.Gbest,[1,PSO.N])-PSO.X);
    PSO.X=PSO.X+PSO.V;
    % repair particles
    for d=1:PSO.D,
        z=find(PSO.X(d,:)<PSO.BND(d,1));
        PSO.X(d,z)=PSO.BND(d,1)+rand(1,length(z))*diff(PSO.BND(d,:));
        z=find(PSO.X(d,:)>PSO.BND(d,2));
        PSO.X(d,z)=PSO.BND(d,1)+rand(1,length(z))*diff(PSO.BND(d,:));
    end;
end;

% show result with optimized controller
fis_vel.output.mf(1).params(3)=PSO.Gbest(1);% modify vel output Z-MF
fis_ang.output.mf(2).params(3)= PSO.Gbest(2);% modify ang output Z-MF
fis_ang.output.mf(2).params(1)=-PSO.Gbest(2);
car=carInit;% restore initial car pose
for t=0:time.dt:time.T,
    [ds]=FindDistance(car,target);
    vel=evalfis(ds,fis_vel);
    [dq]=FindAngular(car,target);
    ang=evalfis(dq,fis_ang);
    [car]=CarNow(car,time,vel,ang);
    [car]=CarShow(fig,car,t);
end;

function [fig]=FigureNew(field)
fig.fig=figure('units','normalized','position',[0.3 0.2 0.5 0.5]);
axis([-1 1 -1 1]*field.range); hold on; grid on; axis equal;
xlabel('x-direction'); ylabel('y-direction');
fig.ax=axis;

function [target]=TargetNew(field)
target.x=(rand-0.5)*field.range;
target.y=(rand-0.5)*field.range;
target.q=AngleWrap(rand*2*pi);
target.shape=[ 2 0; 1 1; -1 1; -1 -1; 1 -1; 2 0]';
target.hdL.shape=plot(target.shape(1,:),target.shape(2,:),...
    'color','r','linewidth',2);
Rz=[ cos(target.q) -sin(target.q);
      sin(target.q) cos(target.q)];
shape=Rz*target.shape+repmat([target.x;target.y],1,6);
set(target.hdL.shape,'xdata',shape(1,:),'ydata',shape(2,:));

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

function [car]=CarNew(field)
car.x=(rand-0.5)*field.range*3;
car.y=(rand-0.5)*field.range*3;
car.q=AngleWrap(randn*2*pi);
car.trace=[car.x; car.y; car.q];
car.shape=[ 2 0; 1 1; -1 1; -1 -1; 1 -1; 2 0]';
car.hdL.shape=plot(car.shape(1,:),car.shape(2,:),'color','b','linewidth',2);
car.hdL.trace=plot(car.trace(1,:),car.trace(2,:),'color',[0 0.66 0]);

function [car]=CarNow(car,time,v,w)
car.x=car.x+time.dt*v*cos(car.q);
car.y=car.y+time.dt*v*sin(car.q);
car.q=car.q+time.dt*w; car.q=AngleWrap(car.q);
car.trace(:,end+1)=[car.x; car.y; car.q];

function [car]=CarShow(fig,car,t)
Rz=[ cos(car.q) -sin(car.q);
sin(car.q) cos(car.q)];
shape=Rz*car.shape+repmat([car.x;car.y],1,6);
set(car.hdL.shape,'xdata',shape(1,:),'ydata',shape(2,:));
set(car.hdL.trace,'xdata',car.trace(1,:),'ydata',car.trace(2,:));
axis(fig.ax); title(sprintf('Time %d',t)); pause(0.001);
