%close all;
clear all;clc;

% initializing parameters
filename='sotano_1300_1300_3.jpg';drate = 39;showflag=1;
%filename='testmap_500_500.PNG';drate = 19;showflag=1;
% create the grid graph
G=CreateGridGraph(filename,drate,showflag);

% get the source and destination nodes from inputs, 4 points in this example 
if (~showflag)
        imshow(G.mapshow);
end

[uu] = Pwaypoint(filename,drate,showflag);
[vv] = PPPwaypoint(filename,drate,showflag);

n=1;
for (i=1:length(vv.points(1,:)))
    if(vv.points(1,i)~=0)
        v(n)=i;
        n=n+1;
    else
    end
end
n=1;
for (i=1:length(uu.points(1,:)))
    if(uu.points(1,i)~=0)
        u(n)=i;
        n=n+1;
    else
    end
end

% read the input image and convert to a binary map
map=imread(filename);
mapshow=map;
if (showflag)
    figure,
    imshow(im2bw(map));
end

map=im2bw(map);
map=logical(map);
cubsize=floor(drate/2);
imgcol=size(map,2);
imgrow=size(map,1);
row=round(imgrow/drate); 
col=round(imgcol/drate);

    for m=1:length(v)
        Ir=0;punto=[-1;-1];flagg=0;
        punto2=[-1;-1];punto3=[-1;-1];punto4=[-1;-1];punto5=[-1;-1];punto6=[-1;-1];punto7=[-1;-1];punto8=[-1;-1];punto9=[-1;-1];
        for n=1:length(u)
            ux=uu.points(1,u(n))+1;uy=uu.points(2,u(n))+1;vx=vv.points(1,v(m));vy=vv.points(2,v(m));
            acu=1;xx=[0];yy=[0];
            while ((ux~=vx && uy~=vy) | ux~=vx | uy~=vy)
                if vy<uy
                    vy=vy+1;
                    yy(acu)=vy;
                elseif vy>uy
                    vy=vy-1;
                    yy(acu)=vy;
                else
                    yy(acu)=vy;
                end
                if vx>ux
                    vx=vx-1;
                    xx(acu)=vx;
                elseif vx<ux
                    vx=vx+1;
                    xx(acu)=vx;
                else
                    xx(acu)=vx;
                end
                acu=acu+1;
            end
            for i=1:length(xx)
                %¿hay obstáculo en ese punto que une u y v?
                xi=xx(i)-1;yi=yy(i)-1;
                if(Checkobs(map,xi,yi,cubsize,drate)==0)
                    punto=[vx-1;vy-1];
                    break
                end
            end

            if(uu.points(:,u(n))==punto)
                Ir_uu=0;
                Ir_u(n)=0;
                tt(n)=0;
                t_n(n)=0;
                Ir=Ir+Ir_uu;
                flagg=1;
            else
                x=(vv.points(1,v(m))-uu.points(1,u(n))-1)*3*imgrow/2/row;
                y=(vv.points(2,v(m))-uu.points(2,u(n))-1)*3*imgcol/2/col;
                distancia=abs(sqrt((x)^2+(y)^2));
                angulo=x/distancia;
                %Ir_uu=55*cos(angulo)/(distancia^2+90.9^2);
                Ir_uu=100*10^4/(2*pi*distancia*90.9);
                Ir_u(n)=100*10^4/(2*pi*distancia*90.9);
                %t_n(n)=0.0037/Ir_u(n);
                %Ir_uu=55/(2*pi*distancia*90.9);
                %Ir_u(n)=55/(2*pi*distancia*90.9);
                Ir=Ir+Ir_uu;
                tt(n)=1;
            end
            lb(n)=0;%ub(n)=50;
        end

    %tiempo(m,:)=t_n;
    tuv(m,:)=tt;
    Ir_uuu(m,1:length(Ir_u))=Ir_u;
    Irr(m,1)=Ir;
    t_ir(m)=1;

    end

k=1;
for (i=1:length(Irr))
    if (Irr(i)==0)
    else
        Irradiacion(k,:)=Ir_uuu(i,:);
        irrad2(k,1)=Irr(i);
        tiempo2(k,:)=tuv(i,:);
        t_irr(k)=1;
        k=k+1;
    end
end

for (n=1:length(u))
    xu=uu.points(1,u(n));yu=uu.points(2,u(n));
            if (Checkobs(map,xu-1,yu-1,cubsize,drate)==0)
                t_u(n)=1;
            elseif (Checkobs(map,xu-1,yu,cubsize,drate)==0)
                t_u(n)=1;
            elseif (Checkobs(map,xu-1,yu+1,cubsize,drate)==0)
                t_u(n)=1;
            elseif (Checkobs(map,xu,yu+1,cubsize,drate)==0)
                t_u(n)=1;
            elseif (Checkobs(map,xu+1,yu+1,cubsize,drate)==0)
                t_u(n)=1;
            elseif (Checkobs(map,xu+1,yu,cubsize,drate)==0)
                t_u(n)=1;
            elseif (Checkobs(map,xu+1,yu-1,cubsize,drate)==0)
                t_u(n)=1;
            elseif (Checkobs(map,xu,yu-1,cubsize,drate)==0)
                t_u(n)=1;
            else
                t_u(n)=0;
            end
end

%El secreto está en los 1 y 0 del t_u
% si yo no quiero que se pare ahí entonces ese t_u que corresponde a dicha
% posición debe ser 1, y si quiero que se pare ahí debe ser 0
%tttt=[1 1 1 1 1 1 1 1 1 1 1 1 1     1 0 0 1 1 1 1 0 1 1 0 0 1   1 0 0 1 1 0 1 1 1 1 0 0 1   1 1 1 1 1 0 1 1 1 1 1 1 1     1 0 1 1 1    1 0 1 1 1    1 1 1 1 1 1 1 0 1 1 1 1 1      1 0 0 0 0 0 0 0 1 1 0 0 1      1 0 0 0 0 0 0 0 1 1 0 0 1      1 0 0 0 0 0 0 0 1 1 0 0 1       1 0 1 1 1 1 1 1 1 1 1 1 1      1 0 1 1 1      1 0 1 1 1    1 0 0 0 0 0 0 0 1 1 1 1 0 0 1      1 1 1 1 1 1 1 1 1 1 1 1 1 1 1];
funcion=t_u;   
A=[diag(t_u*-1);-Irradiacion.*tiempo2];b=[0*t_u -3.7*10^3*t_irr];

[t,fval,exitflag,output,lambda]=linprog(funcion,A,b,[],[],t_u*0);

k=1;
for (i=1:length(t))
    if (t(i)==0)
    else
        waypoints(k)=u(i);
        k=k+1;
    end
end

p1=waypoints(1,1:(length(waypoints)-1));
p2=waypoints(1,2:length(waypoints));
%p1=[39,40,41,42,44,50,61,62,67,78,81];
%p2=[40,41,42,44,50,61,62,67,78,81,100];
% compute shortest path between input points 
path=ComputePath(G,p1,p2);

% show the path between points
ShowPath(G,path);
