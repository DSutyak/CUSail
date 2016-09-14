function [netForce_relFixed, netMoment_relBody] = airfoil_forces2(state,airfoil,p)
v_boat_relFixed=state(7:9);
w_boat_relBody=state(10:12);
k_hat=[0,0,1]';

R=p.boat.R*airfoil.R;
Rt=R.';
nElements=length(airfoil.points_relAirfoil(1,:));
SA_element=airfoil.SA/nElements;

points_airfoil_relBody=airfoil.H*[airfoil.points_relAirfoil;...
    ones(1,nElements)];

for m=1:nElements
    
    point_relFixed=p.boat.H*points_airfoil_relBody(:,m);
    
    if point_relFixed(3)>0
        density=p.density_air;
        v_fluid_relFixed=p.v_air_relFixed;
    else
        density=p.density_water;
        v_fluid_relFixed=p.v_water_relFixed;
    end
    
    v_app_relFixed=v_fluid_relFixed-v_boat_relFixed;
    v_app_relAirfoil=Rt*v_app_relFixed;
    
    rot2linVel_relBoat=cross(w_boat_relBody,points_airfoil_relBody(1:3,m));
    v_airfoil_relAirfoil=airfoil.Rt*rot2linVel_relBoat-v_app_relAirfoil;
    v_airfoil_relAirfoil=[v_airfoil_relAirfoil(1:2);0];
    alpha_airfoil=wrapTo2Pi(-atan2(v_airfoil_relAirfoil(2),v_airfoil_relAirfoil(1)));
    [c_lift,c_drag]=C_LD(alpha_airfoil,p);
    
    lift_relAirfoil=.5*density*SA_element*norm(v_airfoil_relAirfoil)*...
        c_lift*cross(k_hat,v_airfoil_relAirfoil);
    drag_relAirfoil=.5*density*SA_element*norm(v_airfoil_relAirfoil)*...
        c_drag*(-v_airfoil_relAirfoil);
    
    lift_relBody=airfoil.R*lift_relAirfoil;
    drag_relBody=airfoil.R*drag_relAirfoil;
    
    forces_relBody(:,m)=lift_relBody + drag_relBody;
    moments_relBody(:,m)=cross(points_airfoil_relBody(1:3,m),forces_relBody(:,m));
end

netMoment_relBody=sum(moments_relBody,2);
netForce_relFixed=p.boat.R*sum(forces_relBody,2);