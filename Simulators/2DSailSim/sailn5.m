function sail3
%This function calls the function 'input.m' and uses an euler integration
%that makes use of the 'rhs.m' function, where the dynamics of the simulation takes place.
%Outputs:
%   -Matrix with time points determining row,
%   and columns=[[x,y]position , [x,y]velocity,  boat angle,  boat angular velocity, rudder angle, sail angle]
%
%   -Figure 1 is a graph of the location of the boat over time, w/ wind direction/magnitude indicated.
%   Also includes sail and rudder locations over time.
%   -Figure 2 shows the complete trajectory of boat.
clear all; close all; hold on
tspan=[0 150]; n=300;  t=linspace(tspan(1), tspan(2), n+1);

%Call input parameters
[rho, p, v, x0, I, th, wp1, omega]=inputsIndivid;
v0=v.boat; theta=th.b;  omega=omega;
z0=[x0 v0 theta omega th.r th.s]';
%Call Integrator
[t, zarray]=eulermethod(tspan, t, z0,n, p,rho, v, I, th, x0, wp1); 
%disp(zarray)
%Label Outputs
xfin=zarray(end,1:2);
vfin=zarray(end,3:4);
thfin=zarray(end,5);
omfin=zarray(end,6);
vfnorm=vfin/norm(vfin);
%Define Figure 1
figure(1);
xlabel('Distance (m)'); ylabel('Distance (m)');
for k=1:length(t)
    cla;
    for waypnt=1:2:length(wp1)
        pointx(waypnt)=wp1(waypnt);
        pointy(waypnt)=wp1(waypnt+1);
    end
    scatter(pointx, pointy, '*');
    xx=zarray(k,1); yy=zarray(k,2); tt=zarray(k,5); ttr=zarray(k,7); tts=zarray(k,8);
    axis([-4+xx 4+xx -4+yy 4+yy]);
    tip=([xx yy]+.5*p.length.h.*[cosd(tt) sind(tt)]);
    tail=([xx yy]-.5*p.length.h.*[cosd(tt) sind(tt)]);
    port=(tail+.5*p.width.h.*[cosd(tt+90) sind(tt+90)]);
    star=(tail+.5*p.width.h.*[cosd(tt-90) sind(tt-90)]);
    plot([tail(1), port(1), tip(1), star(1), tail(1)], [tail(2), port(2), tip(2), star(2), tail(2)],'-g');
    
    cmr=[xx yy]+(-.22)*[cosd(tt+tts),sind(tt+tts)];
    tiprud=cmr+(.5*p.length.r)*[cosd(tt+ttr),sind(tt+ttr)]; tailrud=cmr-(.5*p.length.r)*[cosd(tt+ttr),sind(tt+ttr)];
    line([tailrud(1), tiprud(1)], [tailrud(2), tiprud(2)], 'Color','k','LineWidth', 2.5);
    tipsail=[xx yy]+.5*p.length.s*[cosd(tt+tts) sind(tt+tts)]; tailsail=[xx yy]-.5*p.length.s*[cosd(tt+tts), sind(tt+tts)];
    line([tailsail(1), tipsail(1)], [tailsail(2), tipsail(2)],'Color', 'm','LineWidth',2.5);
    line([tailsail(1),cmr(1)],[tailsail(2),cmr(2)], 'Color','k');
    legend('Boat', 'Rudder', 'Sail', 'Sail-Rudder Connector');
    plot(zarray(1:k,1),zarray(1:k,2), '.k', 'MarkerSize', .25)
    fw1=[xx yy]+[2*p.length.h 2*p.length.h];
    fw2=fw1+v.wind;
    arrow(fw1, fw2,'EdgeColor','b','FaceColor','b')
    text(fw1(1), fw1(2)-.2,'Wind');
    xlabel('Distance (m)'); ylabel('Distance (m)');

    pause(0.15)
end
%Define Figure 2
figure(2)
cla; hold on
hold on
plot(zarray(1:end,1),zarray(1:end,2), '-k', 'MarkerSize', 4.5)
fw1=xfin+[2*p.length.h 2*p.length.h];
fw2=fw1+v.wind;
arrow(fw1, fw2,'EdgeColor','b','FaceColor','b')
text(fw1(1), fw1(2)-.2,'Wind');
xlabel('Distance (m)'); ylabel('Distance (m)');

end
function [t zarray]=eulermethod(tspan, t,z0, n, p,rho, v, I, th, x0, wp1)
time=t(2)-t(1);
zarray= zeros(n+1,length(z0)); zarray(1,:)=z0';
w=0;
aa=0;
tackpointx=0;
tackpointy=0;
waypoint=0;
prevNormr=0;
timeAway=0;
waypointsize=4;
magWind= -norm(v.wind);
for i=1:n;
    ti=t(i);
    currpos=zarray(i,1:2);
    currth=zarray(i,5); currvel= v.wind; % -zarray(i, 3:4);
    z=zarray(i,:)';
    %if i<50;
    rwind=wrapTo360((atan2(currvel(2), currvel(1))*180/pi-180));%-180))
    %else
        %rwind=wrapTo360((atan2(currvel(2), currvel(1))*180/pi-200))
     %   rwind=0;  
    %end
    %%%%%%%%%%%%%% NAV CODE: uncomment below %%%%%%%%%%%%%%%%
    if w==2;  %set time interval to call nav code(e.g. w->5 is every 5s
        out=nav_v2_primitive(wp1,currpos, rwind, currth, waypoint, waypointsize, aa,tackpointx, tackpointy);
    aa=out(4);
    tackpointx=out(5);
    tackpointy=out(6);
    waypoint=out(7); 
    %prevNormr=out(8);
    %timeAway=out(9);
    th.r=double(out(1)); th.s=double(out(2));
    new_err=double(out(3)); new_err2=double(out(4));
    w=0;
    v.wind=[magWind*cosd(rwind), magWind*sind(rwind)];
    %disp(v.wind);
    else
    w=w+1;
    end
    zzz=rhs(ti,z, p,rho, v, I, th, x0, wp1); otherz=z(1:6)+time*zzz(1:6);
    znew=[otherz; zzz(7:8)];
    zarray(i+1,:)=znew';
end
end
function zdot=rhs(t,z,p,rho,v, I, th, x0, wp1)
xdot=z(3:4)';
thetadot=z(6)';
thetaboat=z(5); %degrees
vr=v.wind;%-xdot+v.wind;%HERE
vrn=vr/norm(vr);
if norm(xdot)==0
    xnorm=[0,0];
else
    xnorm=xdot/norm(xdot);
end
thetaruddot=th.r;
thetasaildot=th.s;
alph=compute_alpha3(th, vr, xdot, thetaboat);

%Read data file with alpha, CL, CD
[data]=all_alphas3();
alpha=data(:,1);
CL=data(:,2);
CD=data(:,3);
%interpolate data for exact alpha
CLr=ppval(pchip(alpha,CL),(abs(alph.r)));
CDr=ppval(pchip(alpha,CD),(abs(alph.r)));
CLs=ppval(pchip(alpha,CL),(abs(alph.s)));
CDs=ppval(pchip(alpha,CD),(abs(alph.s)));
CLk=ppval(pchip(alpha,CL),(abs(alph.k)));
CDk=ppval(pchip(alpha,CD),(abs(alph.k)));
CDh=6.5; %based on Jesse Miller's experiments
Cdamph=2; %based on Jesse Miller's experiments

vv=[vrn 0]; kk=[0 0 1]; L=cross(vv,kk); %Define direction of lift (air)
vk=[xnorm 0]; K=cross(vk,kk);           %Define direction of lift (water)
%Force calculation (NOTE: water->xdot(boat velocity), air->vr(boat and wind velocity)
f.lr=CLr*p.a.r*.5*rho.air*(norm(vr))^2.*L; f.lr=f.lr(:,1:2);
f.lk=-CLk*p.a.k*.5*rho.water*(norm(xdot))^2.*K; f.lk=f.lk(:,1:2);
f.ls=CLs*p.a.s*.5*rho.air*(norm(vr))^2.*L; f.ls=f.ls(:,1:2);
f.ds=CDs*p.a.s*.5*rho.air*(norm(vr))^2.*vrn;
f.dk=-CDk*p.a.k*.5*rho.water*(norm(xdot))^2.*xnorm;
f.dr=CDr*p.a.r*.5*rho.air*(norm(vr))^2.*vrn;
f.dh=-CDh*(norm(xdot))^2*(xdot);
%assign velocity derivative
vdot=1/p.mass * (f.ls+f.lk+f.lr+f.ds+f.dr+f.dk+f.dh);

%Moment Calculation
r.r=[cosd(thetaboat+th.s)*(-.2286), sind(thetaboat+th.s)*(-.2286)];
r.k=[0, 0];
r.s=[0, 0];
m.r=cross([r.r 0],[(f.lr+f.dr) 0]);
m.k=cross([r.k 0],[(f.lk+f.dk) 0]);
m.s=cross([r.s 0],[(f.ls+f.ds) 0]);
m.h=-Cdamph.*[0 0 thetadot*pi/180];
sumM=(m.r+m.k+m.s+m.h);
sum=sumM(:,3);
omegadot= 180/pi*(1/I*sum); %in degrees/second
zdot=[xdot vdot thetadot omegadot thetaruddot thetasaildot]';
end