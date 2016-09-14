function v_airfoil_relFixed= velocity_pnt(R,v_boat_relFixed,w_boat_relBody,airfoil)

w_fixed=R*w_boat_relBody;

v_airfoil_relFixed=zeros(size(airfoil.points_relBody));
for k=1:length(airfoil.points_relBody(1,:))
    point_relFixed=R*airfoil.points_relBody(:,k);
    v_airfoil_relFixed(:,k)=cross(w_fixed,point_relFixed) + v_boat_relFixed - airfoil.v_fluid_relFixed;
end
