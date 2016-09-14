function [p,state0]=setBoatParam
% Initializes various boat parameters and initial state of the boat. Make
% changes to the parameters in this file to test various
%   Outputs:
%       p       structure containing relavent parameters
%       state0  12-by-1 array containing the initial pose of the boat
%
%   Cornell University
%   Author Name: Jesse Miller 
%   Author NetID: jam643


%% Real time boat control vs. precalculating trajectory
p.realTime=2;   %0: precalculate boat trajectory
                %1: real time control of boat using keyboard
                %2: ALEC EDIT - Use navigation code to control boat

%% initial conditions
%initial pose and (d/dt)pose
x0=0; y0=0; z0=0; % initial x,y,z coordinates of boat COM
phi0=0; theta0=0; psi0=2; % initial roll, pitch, yaw of boat
xdot0=0; ydot0=0; zdot0=0; % initial x,y,z velocities of boat COM
phidot0=0; thetadot0=0; psidot0=0; %initial roll, pitch, yaw angular velocities of boat
%contains the initial state/pose of the boat
state0=[x0,y0,z0,phi0,theta0,psi0,xdot0,ydot0,zdot0,phidot0,thetadot0,psidot0]';

%% simulation fps and run time
p.time=20; %total run time [s]
p.fps=30; %frames per second
p.tspan=linspace(0,p.time,p.fps*p.time); %timespan of simulation [s]

%% Parameters Affecting Simulation Accuracy
p.nElements=1;  %number of blade elements to use for airfoil calculations
p.numInt=10;    %number of times to perform Euler integration per timestep
                %(increase if solution explodes, decrease if simulation is
                %too slow). Only used when p.realTime=1
p.accuracy=1;   %p.accuracy defines the accuracy with which lift and drag
                %coeff. are interpolated
                %1: most accurate but slower (using pchip interpolation)
                %2: less accurate but fastest (approx. Clift Cdrag as sinusoidal)

%% Environmental Parameters
p.v_air_relFixed=[5,0,0]'; %(x,y,z) components of the true wind [m/s]
p.v_water_relFixed=[0,0,0]'; %(x,y,z) components of the water velocity [m/s]
p.density_air=1.2; %density of air [kg/m^3]
p.density_water=1000; %density of water [kg/m^3]
p.g=9.8; %acceleration due to  gravity [m/s^2]

%% Hull Parameters
p.hull.radius=0.25; %radius of hull at widest part [m]
p.hull.mass=7; %mass of hull [kg]
p.hull.length=0.9; %length of hull [m]
%calculates volume of hull by estimating it as an ellipsoid
p.hull.vol_ellipsoid=(4/3)*pi*(p.hull.radius)^2*(p.hull.length/2);
%indicates hull is submerged in water
p.hull.v_fluid_relFixed=p.v_water_relFixed;
p.hull.density_fluid=p.density_water;

%% Sail Parameters
%(x,y,z) position of base of sail relative to the center of the hull
p.sail.origin_relHullCOM=[0,0,0.33]';
% (x,y,z) Unit vector that points in the direction that the sail extends
p.sail.unit_vector=[0,0,1]';
% 1 or -1 indicating the sail points up or down unit vector
p.sail.direction=1;
p.sail.length=0.88; %length of sail [m]
p.sail.width=0.2; %width of sail [m]
p.sail.SA=p.sail.width*p.sail.length; %surface area sail [m^2]
p.sail.mass=1; %mass of sail [kg]
p.sail.angle_relBody=0.9; %angle of sail relative to boat [rad]

%% Keel Parameters
%(x,y,z) position of base of keel relative to the center of the hull
p.keel.origin_relHullCOM=[0,0,-p.hull.radius]';
% (x,y,z) Unit vector that points in the direction that the keel extends
p.keel.unit_vector=[0,0,-1]';
% 1 or -1 indicating the keel points up or down unit vector
p.keel.direction=-1;
p.keel.length=0.3; %length of keel [m]
p.keel.width=0.1; %width of keel [m]
p.keel.SA=p.keel.width*p.keel.length; %surface area keel [m^2]
p.keel.mass=0.5; %mass of keel [kg]
p.keel.angle_relBody=0; %angle of keel relative to boat [rad]

%% Rudder Parameters
p.rudder.type=2;    %1: rudder is attached to boat
                    %2: rudder is attached to sail, AKA 'tail' design
%p.rudder.angle_relBody=-0.4; %(for rudder.type=1) angle of rudder relative to boat [rad]
p.rudder.angle_relSail=0.6; %(for rudder.type=2) angle of rudder relative to sail [rad]
%(x,y,z) position of base of rudder relative to the center of the hull
p.rudder.origin_relHullCOM=[-0.5,0,-.1]';
% (x,y,z) Unit vector that points in the direction that the rudder extends
p.rudder.origin_relSail=[-0.23,0,0.03]';
% 1 or -1 indicating the rudder points up or down unit vector
p.rudder.direction=1;
p.rudder.length=0.52; %length of rudder [m]
p.rudder.width=0.15; %width of rudder [m]
p.rudder.SA=p.rudder.width*p.rudder.length; %surface area rudder [m^2]
p.rudder.mass=0.5; %mass of rudder [kg]

%% Ballast Parameters
p.ballast.mass=3; %mass of ballast [kg]

%% DO NOT MAKE CHANGES TO THE CODE BELOW
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% calculate boat properties based on chosen parameters 
p=boatGeometry(p);

%% Calculate Rotation and Homogeneous Matrices for sail, keel, hull, and rudder
[p.sail.H,p.sail.Ht,p.sail.R,p.sail.Rt]=euler2Hom([0,0,p.sail.angle_relBody],p.sail.origin);
[p.keel.H,p.keel.Ht,p.keel.R,p.keel.Rt]=euler2Hom([0,0,p.keel.angle_relBody],p.keel.origin);
% different Rot. and Hom. matrix calculations for rudder vs tail design
if p.rudder.type==1 %rudder type
    [p.rudder.H,p.rudder.Ht,p.rudder.R,p.rudder.Rt]=euler2Hom([0,0,p.rudder.angle_relBody],p.rudder.origin);
elseif p.rudder.type==2 %tail type
    [p.rudder.H_relSail,p.rudder.Ht_relSail,p.rudder.R_relSail,p.rudder.Rt_relSail]=...
        euler2Hom([0,0,p.rudder.angle_relSail],p.rudder.origin_relSail);
end
[p.hull.H,p.hull.Ht,p.hull.R,p.hull.Rt]=euler2Hom([0,0,0],p.hull.origin);

%% Breaks up sail, keel, and rudder into Blade Elements
p.sail.points_relAirfoil=bladeElement2(p.sail,p);
p.keel.points_relAirfoil=bladeElement2(p.keel,p);
p.rudder.points_relAirfoil=bladeElement2(p.rudder,p);

%% Tabulated Data for NACA 0015 airfoil to be interpolated later on
%angles [deg] corresponding with the lift and drag coefficient data points
angle = [0,10,15,17,23,33,45,55,70,80,90,100,110,120,130,...
    140,150,160,170,180,190,200,210,220,230,240,250,260,...
    270,280,290,305,315,327,337,343,345,350,360]';
%lift coefficient data points corresponding with the angle array
lift = [0,0.863,1.031,0.58,.575,.83,.962,.8579,.56,.327,...
    .074,-.184,-.427,-.63,-.813,-.898,-.704,-.58,-.813,0,...
    .813,.58,.704,.898,.813,.63,.427,.184,-.074,-.327,...
    -.56,-.8579,-.962,-.83,-.575,-.58,-1.031,-.863,0]';
%drag coefficient data points corresponding with the angle array
drag = [0,.021,.058,.216,.338,.697,1.083,1.421,1.659,1.801,...
    1.838,1.758,1.636,1.504,1.26,.943,.604,.323,.133,0,...
    .133,.323,.604,.943,1.26,1.504,1.636,1.758,1.838,1.801,...
    1.659,1.421,1.083,.697,.338,.216,.058,.021,0]';

p.paraDrag=.15; %parasitic drag used to limit max(Cl/Cd)=5
drag=drag+p.paraDrag; %adjusted drag

p.C0=0.9;  % nominal lift coefficient for sinusoidal lift/drag approx.

% Fit a pchip piecewise-polynomial curve fit to the data
% This is good because it preserves local maximum and minimum
% Another option would be to replace pchip() with spline(), which would
% produce a more smooth curve, but would add new peaks to the data.
p.lift = pchip(angle,lift);
p.drag = pchip(angle,drag);
