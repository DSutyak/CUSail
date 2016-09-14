function stateDot = rhs(t,state,p)
boat_height=state(3);
phi=state(4); %roll
theta=state(5); %pitch
psi=state(6); %yaw
boat_angles=state(4:6);
w_boat_relBody=state(10:12);
v_boat_relFixed=state(7:9);

[R,Rt]=euler2Rot(phi,theta,psi);
[p.boat.H,p.boat.Ht,p.boat.R,p.boat.Rt]=euler2Hom(boat_angles,[0,0,0]);

[netForce_sail_relFixed, netMoment_sail_relBody] = airfoil_forces2(state,p.sail,p);

[netForce_keel_relFixed, netMoment_keel_relBody] = airfoil_forces2(state,p.keel,p);

if p.rudder.type==2;
    p.rudder.H=p.sail.H*p.rudder.H_relSail;
    p.rudder.R=p.sail.R*p.rudder.R_relSail;
    p.rudder.Rt=p.rudder.R.';
end

[netForce_rudder_relFixed, netMoment_rudder_relBody] = airfoil_forces2(state,p.rudder,p);

[xyForce_hull_relFixed,zMoment_hull_relBody]=hullForce_xy(Rt,v_boat_relFixed,psi,w_boat_relBody,p);

[zForce_hull_relFixed,xMoment_hull_relBody]=hullForce_yz(R,Rt,p.hull,v_boat_relFixed,boat_height,p);

Force_weight_relFixed=[0,0,-p.g*p.boat.mass]';

% xy-plane
%sum of forces
vdot=(netForce_sail_relFixed+netForce_keel_relFixed+netForce_rudder_relFixed+...
    xyForce_hull_relFixed+zForce_hull_relFixed+Force_weight_relFixed)...
    /p.boat.mass;

psidotdot=dot((netMoment_sail_relBody+netMoment_keel_relBody+...
    netMoment_rudder_relBody+zMoment_hull_relBody),[0;0;1])/p.boat.Izz;

phidotdot=dot((netMoment_sail_relBody+netMoment_keel_relBody+...
    netMoment_rudder_relBody+xMoment_hull_relBody),[1;0;0])/p.boat.Ixx;

if ~p.realTime && floor(20*rand) == 0
    waitbar(t/p.time,p.waitBar);
end

thetadotdot=0; 
stateDot=[state(7:12)',vdot',phidotdot,thetadotdot,psidotdot]';
