function h=animateRT(t,stateArray,p,h)
%%%Animate boat's motion in real time

%unpack variables
hull_l=p.hull.length; hull_r=p.hull.radius;
sail_w=p.sail.width; sail_l=p.sail.length;
keel_w=p.keel.width; keel_l=p.keel.length;
rudder_w=p.rudder.width; rudder_l=p.rudder.length;
buffer=0.8*hull_l;

%extract variables
x=stateArray(end,1);
y=stateArray(end,2);
z=stateArray(end,3);
phi=stateArray(end,4); %roll
theta=stateArray(end,5); %pitch
psi=stateArray(end,6); %yaw
v_boat=stateArray(end,7:9);

%plot trajectory so far
set(h.trajectory,'xdata',stateArray(1:end,1),'ydata',stateArray(1:end,2),'zdata',stateArray(1:end,3));

%plot > rotate > translate hull
H=euler2Hom([phi,theta,psi],[x,y,z]);
H_hull=H*p.hull.H;
x_hull=[-0.5*hull_l,0.05*hull_l,0.5*hull_l,0.05*hull_l,-0.5*hull_l,-0.5*hull_l];
y_hull=[-.08*hull_l,-.15*hull_l,0,.15*hull_l,.08*hull_l,-.08*hull_l];
z_hull=zeros(size(x_hull));
p0_hull=[x_hull;y_hull;z_hull;zeros(size(x_hull))+1];
p_hull=H_hull*p0_hull;
set(h.hull1_xy,'xdata',p_hull(1,:),'ydata',p_hull(2,:),'zdata',p_hull(3,:));

x_hull=[-0.5*hull_l,0.05*hull_l,0.05*hull_l,-0.5*hull_l,-0.5*hull_l];
y_hull=[.08*hull_l,.15*hull_l,.15*hull_l,.08*hull_l,.08*hull_l];
z_hull=[0,0,-0.7*hull_r,-0.5*hull_r,0];
p0_hull=[x_hull;y_hull;z_hull;zeros(size(x_hull))+1];
p_hull=H_hull*p0_hull;
set(h.hull2_xy,'xdata',p_hull(1,:),'ydata',p_hull(2,:),'zdata',p_hull(3,:));

x_hull=[-0.5*hull_l,-0.5*hull_l,-0.5*hull_l,-0.5*hull_l,-0.5*hull_l];
y_hull=[-.08*hull_l,.08*hull_l,.08*hull_l,-.08*hull_l,-.08*hull_l];
z_hull=[0,0,-0.5*hull_r,-0.5*hull_r,0];
p0_hull=[x_hull;y_hull;z_hull;zeros(size(x_hull))+1];
p_hull=H_hull*p0_hull;
set(h.hull3_xy,'xdata',p_hull(1,:),'ydata',p_hull(2,:),'zdata',p_hull(3,:));

x_hull=[0.05*hull_l,0.5*hull_l,0.05*hull_l,0.05*hull_l];
y_hull=[.15*hull_l,0,.15*hull_l,.15*hull_l];
z_hull=[0,0,-0.7*hull_r,0];
p0_hull=[x_hull;y_hull;z_hull;zeros(size(x_hull))+1];
p_hull=H_hull*p0_hull;
set(h.hull4_xy,'xdata',p_hull(1,:),'ydata',p_hull(2,:),'zdata',p_hull(3,:));

x_hull=[-0.5*hull_l,0.05*hull_l,0.05*hull_l,-0.5*hull_l,-0.5*hull_l];
y_hull=[-.08*hull_l,-.15*hull_l,-.15*hull_l,-.08*hull_l,-.08*hull_l];
z_hull=[0,0,-0.7*hull_r,-0.5*hull_r,0];
p0_hull=[x_hull;y_hull;z_hull;zeros(size(x_hull))+1];
p_hull=H_hull*p0_hull;
set(h.hull5_xy,'xdata',p_hull(1,:),'ydata',p_hull(2,:),'zdata',p_hull(3,:));

x_hull=[0.05*hull_l,0.5*hull_l,0.05*hull_l,0.05*hull_l];
y_hull=[-.15*hull_l,0,-.15*hull_l,-.15*hull_l];
z_hull=[0,0,-0.7*hull_r,0];
p0_hull=[x_hull;y_hull;z_hull;zeros(size(x_hull))+1];
p_hull=H_hull*p0_hull;
set(h.hull6_xy,'xdata',p_hull(1,:),'ydata',p_hull(2,:),'zdata',p_hull(3,:));

%H_sailRelBody=euler2Hom([0,0,p.sail.angle_relBody],p.sail.origin);
H_sail=H*p.sail.H;
%plot > rotate > translate hull
x_sail=[-sail_w/2,sail_w/2,sail_w/2,-sail_w/2,-sail_w/2];
y_sail=zeros(size(x_sail));
z_sail=[0,0,sail_l,sail_l,0]*p.sail.direction;
p0_sail=[x_sail;y_sail;z_sail;zeros(size(x_sail))+1];
p_sail=H_sail*p0_sail;
set(h.sail_xy,'xdata',p_sail(1,:),'ydata',p_sail(2,:),'zdata',p_sail(3,:));

H_keel=H*p.keel.H;
%plot > rotate > translate hull
x_keel=[-keel_w/2,keel_w/2,keel_w/2,-keel_w/2,-keel_w/2];
y_keel=zeros(size(x_keel));
z_keel=[0,0,keel_l,keel_l,0]*p.keel.direction;
p0_keel=[x_keel;y_keel;z_keel;zeros(size(x_keel))+1];
p_keel=H_keel*p0_keel;
set(h.keel_xy,'xdata',p_keel(1,:),'ydata',p_keel(2,:),'zdata',p_keel(3,:));

if p.rudder.type==1
    H_rudder=H*p.rudder.H;
elseif p.rudder.type==2
    H_rudder=H*p.sail.H*p.rudder.H_relSail;
end
%plot > rotate > translate hull
x_rudder=[-rudder_w/2,rudder_w/2,rudder_w/2,-rudder_w/2,-rudder_w/2];
y_rudder=zeros(size(x_rudder));
z_rudder=[0,0,rudder_l,rudder_l,0]*p.rudder.direction;
p0_rudder=[x_rudder;y_rudder;z_rudder;zeros(size(x_rudder))+1];
p_rudder=H_rudder*p0_rudder;
set(h.rudder_xy,'xdata',p_rudder(1,:),'ydata',p_rudder(2,:),'zdata',p_rudder(3,:));

if (x-buffer)-h.limx(1)<0
    Lx=(x-buffer)-h.limx(1);
elseif (x+buffer)-h.limx(2)>0
    Lx=(x+buffer)-h.limx(2);
else
    Lx=0;
end
if (y-buffer)-h.limy(1)<0
    Ly=(y-buffer)-h.limy(1);
elseif (y+buffer)-h.limy(2)>0
    Ly=(y+buffer)-h.limy(2);
else
    Ly=0;
end
h.limx=h.limx+Lx; h.limy=h.limy+Ly;
axis([h.limx,h.limy,h.limz]);

set(h.water_xy,'xdata',[h.limx,h.limx(2:-1:1),h.limx(1)],'ydata',[h.limy(1),h.limy(1),h.limy(2),h.limy(2)]);

windArrow(1,1:3)=[h.limx(1),mean(h.limy),mean(h.limz)+0.5*(h.limz(2)-mean(h.limz))];
wind_norm=p.v_air_relFixed'/norm(p.v_air_relFixed);
windArrow(2,:)=windArrow(1,:)+wind_norm*hull_l*0.5;
set(h.trueWind,'position',windArrow(1,:),'string',sprintf('True Wind = %0.0f m/s',norm(p.v_air_relFixed)))

v_madeGood=dot(-v_boat,wind_norm);
angle_relWind=180-acosd(dot(v_boat,wind_norm)/norm(v_boat));
lt=length(t);

if mod(lt,10) == 0
    h.fps=1/mean(t(lt-8:lt)-t(lt-9:lt-1));
end
textPos=[h.limx(2),mean(h.limy),0];
set(h.boatStats,'position',textPos,'string',sprintf(...
    'time=%0.0f sec\n\nV_{boat}=%0.2f m/s\n\nV_{madeGood}=%0.2f m/s\n\n\\Theta_{/wind}=%0.0f^{\\circ}\n\nFPS=%0.0f',...
     t(end),norm(v_boat),v_madeGood,angle_relWind,h.fps))

set(h.trueWindArrow,'xdata',windArrow(:,1),'ydata',windArrow(:,2),...
    'zdata',windArrow(:,3));

set(h.trueWindArrowStart,'xdata',windArrow(1,1),'ydata',windArrow(1,2),...
    'zdata',windArrow(1,3))