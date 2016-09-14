function [zForce_hull_relFixed,xMoment_hull_relBody]=hullForce_yz(R,Rt,hull,v_boat_relFixed,boat_height,p)
%%%Calculates hull resistance and rotational damping

zdot=v_boat_relFixed(3);
hull_COV_relBody=R*hull.origin;
hull_height=hull_COV_relBody(3)+boat_height;

if hull_height<-p.hull.radius
    hull_height=-p.hull.radius;
elseif hull_height>p.hull.radius
    hull_height=p.hull.radius;
end

if hull_height>0
    bp=hull.radius*sqrt(1-(hull_height/p.hull.radius)^2);
    ap=hull.length*sqrt(1-(hull_height/p.hull.radius)^2);
else
    bp=hull.radius;
    ap=hull.length;
end
A=pi*ap*bp;

Cd_sphere=0.5;
F_d=[0,0,-0.5*hull.density_fluid*Cd_sphere*A*abs(zdot)*zdot]';
if isnan(F_d)
    F_d=zeros(3,1);
end

d=p.hull.radius-hull_height;
V_sub=-(hull.vol_ellipsoid/2)*cos(pi*d/(2*p.hull.radius))+(hull.vol_ellipsoid/2);
F_b=[0,0,V_sub*hull.density_fluid*p.g]';
%

zForce_hull_relFixed=F_b+F_d;

%Cd_plate=1.28;
%xMoment_hull =-(1/2)*hull.density_fluid*Cd_plate*p.SA_keel*sign(phidot)*(phidot*(h_keel))^2*(4/3)*abs((h_keel));
%M_xx_tot=-F_sail_lean*abs(h_sail-CM)+F_keel_lean*abs(h_keel-CM)-abs(CM)*F_b*sin(phi)+M_xx_drag;

zForce_hull_relBody=Rt*zForce_hull_relFixed;
xMoment_hull_relBody=cross(p.hull.origin,zForce_hull_relBody);


