function [rho, p, v, x0, I, theta, wp1, omega]=inputs3(wpnumber, windnumber, bdirnum)
%Basic Parameters(SI units [kg, m, s])
rho.air=1.225; rho.water=1000; 
p.mass=10; %weight of entire boat(~22bs)
p.length.h=1; p.length.s=.2032; p.length.k=.076; p.length.r=.15;
p.width.h=.3; p.width.s=.1; p.width.k=.05; p.width.r=.05;
%largest x-section area 
p.a.s=(.889*.2); 
p.a.k=(.33*.07); 
p.a.r=(.533*.1524); 
%hull: width=12", length=36" height=8"
%rudder: length=21" width=6"
%keel: length=13" width=3" (actually 2, but massive weight on end..thicker--cm at 12" down)
%sail: length=35" width=8"

%Inertia of boat
I=p.mass*(p.length.h^2+p.width.h^2)/20;

%wayPx=[10, 5, 0, -10, -5];
%wayPy=[10, 5, 0, -10, -5];
windDir=[5, 0, -5];
%n=1;
%wayP=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
%for i=1:5
%    for m=1:5
%       wayP(n)=[wayPx(i), wayPy(m)];
%     n=n+1;
%   end
%end
wayP=[10, 10, 10, 5, 10, 0, 10, -10, 10, -5, 5, 10, 5, 5, 5, 0, 5, -10, 5, -5, 0, 10, 0, 5, 0, 0, 0, -10, 0, -5, -10, 10, -10, 5, -10, 0, -10, -10, -10, -5, -5, 10, -5, 5, -5, 0, -5, -10, -5, -5];
%n=1;
windArray=[5, 5, 5, 0, 5, -5, 0, 5, 0, -5, -5, 5, -5, 0, -5, -5];
%for i=1:3
    %for m=1:3
   %     windArray(n)=[windDir(i), windDir(m)];
  %     n=n+1;
 %   end
%end
boatDirection=[0, 90, 180, 270];


%Current location of com of boat
x0=[0,0]; wp1=[wayP(wpnumber), wayP(wpnumber+1)]; 
theta.b=boatDirection(bdirnum); %Boat angle w/r/t global x axis

%Set Wind and Boat Speeds
v.boat=[0,0]; 
v.wind=[windArray(windnumber), windArray(windnumber+1)]; v.wn=v.wind/norm(v.wind); 
v.r=v.wind-v.boat; v.rn=v.r/norm(v.r);
theta.vrn=180-180/pi*atan2(v.r(2),v.r(1));
%%%%%%%%%%%%%%%% NAVIGATION CODE  %%%%%%%%%%%%%%%%%%%%%%%%
%Instructions: uncomment two lines below, and comment theta.r, theta.s below.  
%out=nShort3(wp1,x0, theta.vrn, theta.b, 0, 0);
%theta.r=double(out(1)); theta.s=double(out(2)); new_err=double(out(3)); new_err2=double(out(4));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Set Angles of Rudder and Sail w/r/t boat centerline(ccw is +)
theta.s=0;
theta.r=0;
omega=0; % omega in degrees/s 
end