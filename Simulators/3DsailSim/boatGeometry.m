function p=boatGeometry(p)
% Calculates the boat's total COM, pose of sail keel and rudder relative to
% the boat's COM, and the moment of inertia of the boat.
%   Inputs:
%       p       structure containing relavent parameters
%
%   Outputs:
%       p       updated structure containing relavent parameters
%
%   Cornell University
%   Author Name: Jesse Miller 
%   Author NetID: jam643

%unpack sail, keel, rudder structures
sail=p.sail; keel=p.keel; rudder=p.rudder; ballast=p.ballast; hull=p.hull;
k_hat=[0,0,1]'; %unit vector along z-axis

sail_COM=sail.origin_relHullCOM+0.5*sail.length*sail.direction*k_hat;
keel_COM=keel.origin_relHullCOM+0.5*keel.length*keel.direction*k_hat;
rudder_COM=rudder.origin_relHullCOM+0.5*rudder.length*rudder.direction*k_hat;
ballast_COM=keel.origin_relHullCOM+keel.length*keel.direction*k_hat;

p.boat.mass=sail.mass+keel.mass+ballast.mass+rudder.mass+hull.mass; %mass of boat [kg]
p.boat.COM=(sail_COM*sail.mass+keel_COM*keel.mass+rudder_COM*rudder.mass...
    +ballast_COM*ballast.mass)/p.boat.mass;

p.sail.origin=sail.origin_relHullCOM-p.boat.COM;
p.keel.origin=keel.origin_relHullCOM-p.boat.COM;
p.rudder.origin=rudder.origin_relHullCOM-p.boat.COM;
p.hull.origin=-p.boat.COM;

sail.Ixx=(1/12)*sail.mass*sail.length^2+sail.mass*(sail_COM(3)-p.boat.COM(3))^2;
keel.Ixx=(1/12)*keel.mass*keel.length^2+keel.mass*(keel_COM(3)-p.boat.COM(3))^2;
rudder.Ixx=(1/12)*rudder.mass*rudder.length^2+rudder.mass*(rudder_COM(3)-p.boat.COM(3))^2;
ballast.Ixx=ballast.mass*(ballast_COM(3)-p.boat.COM(3))^2;
hull.Ixx=(1/2)*hull.mass*hull.radius^2+hull.mass*(p.boat.COM(3))^2;

p.boat.Ixx=sail.Ixx+keel.Ixx+rudder.Ixx+ballast.Ixx+hull.Ixx;

sail.Izz=(1/12)*sail.mass*sail.width^2+sail.mass*(sail_COM(1)-p.boat.COM(1))^2;
keel.Izz=(1/12)*keel.mass*keel.width^2+keel.mass*(keel_COM(1)-p.boat.COM(1))^2;
rudder.Izz=(1/12)*rudder.mass*rudder.width^2+rudder.mass*(rudder_COM(1)-p.boat.COM(1))^2;
ballast.Izz=ballast.mass*(ballast_COM(1)-p.boat.COM(1))^2;
hull.Izz=(1/12)*hull.mass*hull.length^2+hull.mass*(p.boat.COM(1))^2;

p.boat.Izz=sail.Izz+keel.Izz+rudder.Izz+ballast.Izz+hull.Izz;