function [] = mtrn4010_11()

% GA based path planning

clc; clear all; close all; dbstop if error;

field.range = 50;
field.LMs = 12;

fig = FigureNew(field);
[field] = NewLandmark(field);

GA.N = 100 ; GA.G = 50;
GA.Pc = 0.9 ; GA.Pm =0.3 ;
GA.mindis = realmax ; GA.mingen = 1;
GA.min = zeros(1,field.LMs);
[GA] = NewPopulation(GA,field);

% main GA loop
for g = 1:GA.G,
    title(sprintf('Gerneration %d/%d Distance %5.2f at %d',...
        g, GA.G, GA.mindis, GA.mingen)); drawnow;
    for n = 1:GA.N, % find travel distance as fitness
        dis(n) = FindDistance(GA.X(n,:),field);
    end;
    [mi,ix] = min(dis); % min travel distance
    if mi < GA.mindis, % fitter than before
        GA.mingen = g; % store
        GA.mindis = mi;
        GA.minX = GA.X(ix,:);
        P = field.LM(:,GA.X(ix,:)); P = [field.LMstart P field.LMstop]; % show path
        set(field.hdL.path, 'xdata',P(1,:),'ydata',P(2,:)); drawnow;
    end;
    GA.mindis_(g)=GA.mindis;
    % convert to minimiation problem
    fit = 1-dis/sum(dis); fit=fit/sum(fit); cumfit=cumsum(fit);
    for n=1:GA.N, % do selection
        r=rand; p = min(find(r<cumfit));
        GA.Y(n,:) = GA.X(p,:);
    end;
    
    % do crossover
    for n = 1:GA.N,
        if rand<GA.Pc,
            p1=ceil(rand*GA.N); p2=ceil(rand*GA.N); c=ceil(rand*field.LMs);
            if rand < 0.5,
                GA.X(n,:) = [fliplr(GA.Y(p1,1:c)) (GA.Y(p2,c+1:end))];
            else
                GA.X(n,:) = [(GA.Y(p1,1:c)) fliplr(GA.Y(p2,c+1:end))];
            end;
        end;
    end;
    
    % do mutation                
    for n=1:GA.N,
        if rand<GA.Pm,
            p=ceil(rand*GA.N); r1=ceil(rand*field.LMs); r2=ceil(rand*field.LMs);
            tmp=GA.X(p,r1); GA.X(p,r1)=GA.X(p,r2); GA.X(p,r2)=tmp;
        end;
    end;
    % do elitism
    p=ceil(rand*GA.N);
    GA.X(p,:)=GA.minX;
    % remove landmark duplications
    for n=1:GA.N,
        for a=1:field.LMs,
            b=find(GA.X(n,:)==a);
            if length(b)>1,
                for c=1:field.LMs,
                    d=find(GA.X(n,:)==c);
                    if length(d)==0,
                        GA.X(n,b(1))=c;
                    end;
                end;
            end;
        end;
    end;
end;
figure('units',normalized',...
'position',[rand*0.3+0.1 rand+0.2+0.1 0.35 0.35]);
plot(GA.mindis_); grid on;
xlabel('Generation'); ylabel('Shortest distance obtained');
%81
function [fig] = FigureNew(field)
fig.fig = figure('units','normalized',...
    'position',[rand*0.3+0.1 rand*0.2+0.1 0.35 0.35]);
axis([-1 1 -1 1]*field.range); hold on; grid on; axis equal;
xlabel('x-direction'); ylabel('y-direction');
fig.ax = axis;
%88
function [field]=NewLandmark(field)
field.LM=2*(rand(2,field.LMs)-0.5)*field.range;
field.hdL.LM=plot(field.LM(1,:),field.LM(2,:),'b*');
hold on; grid on; axis equal;
field.LMstart=2*(rand(2,1)-0.5)*field.range;
field.LMstop=2*(rand(2,1)-0.5)*field.range;
field.hdL.LMstart=plot(field.LMstart(1),field.LMstart(2),'ro',...
    'markerfacecolor','r');
text(field.LMstart(1),field.LMstart(2),' Start','color',[0.8 0 0]);
field.hdL.LMstart=plot(field.LMstop(1),field.LMstop(2), 'go', ...
    'markerfacecolor','g');
text(field.LMstop(1),field.LMstop(2),'Stop','color',[0.3 0 0]);
field.hdL.path=plot([0 0],[0 0]);
%102
function [GA]=NewPopulation(GA,field)
for n=1:GA.N,
    GA.X(n,:)=randperm(field.LMs);
end;
%107
function [dis]=FindDistance(X,field)
dis=0;
for z=1:length(X)-1,
    dx=field.LM(1,X(z+1))-field.LM(1,X(z));
    dy=field.LM(2,X(z+1))-field.LM(2,X(z));
    dxy=sqrt(dx^2+dy^2);
    dis=dis+dxy;
end;
dx=field.LM(1,X(1))-field.LMstart(1);
dy=field.LM(2,X(1))-field.LMstart(2);
dis=dis+sqrt(dx^2+dy^2);
dx=field.LM(1,X(end))-field.LMstop(1);
dy=field.LM(2,X(end))-field.LMstop(2);
dis=dis+sqrt(dx^2+dy^2);