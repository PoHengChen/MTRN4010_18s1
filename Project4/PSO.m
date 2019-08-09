function []=PSO_test()

clc; clear all; close all; dbstop if error;

% find min of function;
% Beale
% f(x,y) = (1.5-x+xy)^2+(2.25-x+xy^2)^2+(2.625x+xy^3)^3
% -5 <= x,y <=5; f*(3,0.5)=0
% Booth
% f(x,y)=(x+2y-7)^2+(2x+y-5)^2
% -10 <= x,y, <=10;
% Ackley
% f(x,y)=-20exp-0.2sqrt(0.5(X^2+Y^2)))-exp(0.5(cos(2pi_X)+cos(2)pi_y))))+20
% -10 <= x,y, <= 10; f*(0,0)=0

% Select bechmark test function
FunList={'Beale','Booth','Ackley'};
FunTest=listdlg('promptstring','Benchmark Function',...
    'SelectionMode','single',...
    'ListString',FunList);

%PSS parameters
PSO.D=2; PSO.B=[-10 10; -10 10];
PSO.G=200; PSO.N=30;
PSO.V=rand(PSO.D,PSO.N);
PSO.cg=2; PSO.cp=2;
PSO.Gebst=[]; PSO.gbest=realmax;
PSO.Pbset=[]; PSO.pbest=ones(1,PSO.N)*realmax;
PSO.X=NewPSOXs(PSO); PSO.gbest_=[];

WgtList={'Time-decreasing','Random','Gaussian','Non-linear'};
WgtStrategy=listdlg('promptString','Inertia weight strategy',...
    'SelectionMode','single',...
    'ListString',WgtList);
switch WgtStrategy,
    case 1,
        PSO.Winit=0.9; PSO.Wfinal=0.4;
    case 4,
        PSO.Winit=0.9; PSO.Wfinal=0.4;
        nL=cell2mat(inputdlg('Non-linear factor','Non-linear factor',1,{'2'}));
end;

%show objective function
figure; d=0.1;
[x,y]=meshgrid(PSO.B(1,1):d:PSO.B(1,2),PSO.B(2,1):d:PSO.B(2,2));
switch FunTest,
    case 1,
        f=reshape(Beale([x(:)';y(:)']),size(x));
    case 2,
        f=reshape(Booth([x(:)';y(:)']),size(x));
    case 3,
        f=reshape(Ackley([x(:)';y(:)']),size(x));
end;
surf(x,y,f,'edgecolor','none'); hold on; grid on;
xlabel('x');ylabel('y'); zlabel('Fit');
hdX=plot3(PSO.X(1,:),PSO.X(2,:),zeros(1,PSO.N),'b.'); alpha(0.3);
hdG=plot3(0,0,0,'ro','markerfacecolor','r','markersize',5);
ax=axis; drawnow;

% PSO main loop
for g=1:PSO.G,
    PSO.X=RepairPSOX(PSO);
    switch FunTest,
        case 1,
            fit=Beale(PSO.X);
        case 2,
            fit=Booth(PSO.X);
        case 3,
            fit=Ackley(PSO.x);
    end;
    for n=1:PSO.N,
        if fit(n)<PSO.gbest,
            PSO.Gbest=PSO.X(:,n);
            PSO.gbest=fit(n);
            set(hdG,'xdata',PSO.Gbest(1),'ydata',PSO.Gbest(2),'zdata',PSO.gbest);
        end;
        if fit(n)<PSO.pbest(n),
            PSO.Pbest(:,n)=PSO.X(:,n);
            PSO.pbest(n)=fit(n);
        end;
    end;
    % show intermediate graphics
    set(hdX,'xdata',PSO.X(1,:),'ydata',PSO.X(2,:),'zdata',fit);
    title(sprintf('Gen=%d gbest=%8.6f',g,PSO.gbest));
    axis(ax); pause(0.05);
    PSO.gbest_=[PSO.gbest_; PSO.gbest];
    % select inertia weiht strategy
    switch WgtStrategy,
        case 1,
            w=(PSO.Winit-PSO.Wfinal)*(PSO.G-g)/PSO.G+PSO.Wfinal;
        case 2,
            w=(0.5+rand)/2;
        case 3,
            w=abs(randn)/2;
        case 4,
            w=((PSO.G-g)/PSO.G)^nL*(PSO.Winit-PSO.Wfinal)+PSO.Wfinal;
    end;
    % PSO updata equation
    PSO.V=w*PSO.V+...
        PSO.cp*rand(PSO.D,PSO.N).*(PSO.Pbest-PSO.X)+...
        PSO.cg*rand(PSO.D,PSO.N).*(repmat(PSO.Gbest,[1,PSO.N])-PSO.X);
    PSO.X=PSO.X+PSO.V;
    % show intermediate result
    disp(sprintf('gen=%d Gbest=%5.3f gbest=%5.3f,%5.3f',...
        g,PSO.gbest,PSO.Gbest(1),PSO.Gbest(2)));
end;
% trace of fitness
figure; plot(PSO.gbest_); grid on;
xlabel('Iteration'); ylabel('Fitness');
    
    
function [X]=NewPSOXs(PSO)
for n=1:PSO.N,
    X(:,n)=NewPSOX(PSO);
end;

function [X]=NewPSOX(PSO)
for d=1:PSO.D,
    mi=PSO.B(d,1); mx=PSO.B(d,2);
    X=mi+rand(PSO.D,1)*(mx-mi);
end;

function [X]=RepairPSOX(PSO)
X=PSO.X;
for d=1:PSO.D,
    mi=PSO.B(d,1); mx=PSO.B(d,2);
    zmi=find(PSO.X(d,:)<mi); zmiL=length(zmi);
    X(d,zmi)=mi+rand(1,zmiL)*(mx-mi);
    zmx=find(PSO.X(d,:)>mx); zmxL=length(zmx);
    X(d,zmx)=mi+rand(1,zmxL)*(mx-mi);
end;

function [f]=Beale(X)
x=X(1,:); y=X(2,:);
f1=(1.5-x+x.*y).^2;
f2=(2.25-x+x.*(y.^2)).^2;
f3=(2.625-x+x.*(y.^3)).^2;
