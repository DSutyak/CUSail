function [H,Ht,R,Rt]=euler2Hom(rot,trans)
phi=rot(1); theta=rot(2); psi=rot(3);
x=trans(1); y=trans(2); z=trans(3);

R=[ cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);...
   cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi);...
       -sin(theta),                  cos(theta)*sin(phi),                              cos(phi)*cos(theta)];

Rt=R.';   

Htemp=[R,[x;y;z]];
H=[Htemp;[0,0,0,1]];

Httemp=[Rt,-Rt*[x;y;z]];
Ht=[Httemp;[0,0,0,1]];