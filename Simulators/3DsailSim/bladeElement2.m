function [points_relAirfoil] = bladeElement2(airfoil,p)
nElements=p.nElements;
length=airfoil.length;
direction=airfoil.direction;
k=[0,0,1]';
points_relAirfoil=zeros(3,nElements);

for m=1:nElements
    points_relAirfoil(:,m) = (m/(nElements+1))*length*direction*k;
end