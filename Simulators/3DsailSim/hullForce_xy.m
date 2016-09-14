function [xyForce_hull_relFixed,zMoment_hull_relBody]=hullForce_xy(Rt,v_boat_relFixed,psi,omega_boat_relBody,p)
%%%Calculates hull resistance and rotational damping

v_boat_relWater=v_boat_relFixed-p.hull.v_fluid_relFixed;
v_boatXy_relWater=[v_boat_relWater(1:2);0];
angle_boat_apparent=atan2(v_boatXy_relWater(2),v_boatXy_relWater(1));
%angle of attack of keel in water
alpha_hull=wrapTo2Pi(psi-angle_boat_apparent);

if (alpha_hull < pi)
    quarter_chord=(0.25-alpha_hull/(2*pi));
else
    quarter_chord=(-0.75+alpha_hull/(2*pi));
end


%hull resistance
xyForce_hull_relFixed=6.5*norm(v_boatXy_relWater)^2*(-v_boatXy_relWater);
xyForce_hull_relBody=Rt*xyForce_hull_relFixed;
zMoment_hull_CP=cross([quarter_chord*p.hull.length,0,0]',xyForce_hull_relBody);

%hull damping moment
zMoment_hull_damp=[0,0,-2*omega_boat_relBody(3)]';

zMoment_hull_relBody=zMoment_hull_damp+zMoment_hull_CP;