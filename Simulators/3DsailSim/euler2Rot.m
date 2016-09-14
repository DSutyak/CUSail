function [R,Rt]=euler2Rot(phi,theta,psi)

R=[ cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);...
   cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi);...
       -sin(theta),                  cos(theta)*sin(phi),                              cos(phi)*cos(theta)];

Rt=R.';