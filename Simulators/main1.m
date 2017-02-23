function main1(boatName)
% Main script to run SailSim
%   Inputs:
%       boatName    string name of .mat file created with makeBoat.m
%                   function containing boat parameter information
%
%   Cornell University
%   Author Name: Jesse Miller 
%   Author NetID: jam643

% get boat parameters
[p,state0]=setBoatParam(boatName);

%Runs the simulation using pre-calculated trajectory or real-time boat
%control
if p.realTime == 0 %pre-calculate trajectory
    %set accuracy of ode solver
    options=odeset('abstol',1e-3,'reltol',1e-3);
    %initialize loading bar
    p.waitBar=waitbar(0,'Sailing...');
    %numerically solve for boat trajectory
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    newstateArray=state0; %this is what you need to put into the nav program. The array elements are as follows: [x0,y0,z0,phi0,theta0,psi0,xdot0,ydot0,zdot0,p0,q0,r0]
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    totalstateArray=zeros(1,12);
    totalTarray = 0;
    a=0.1; %nav program cycle speed
    tmax=20; %time simulation time length
    for t=1:a:tmax
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %[param.sail.angle_relBody, param.rudder.angle_relSail]= %put navigation function here to find sail angle and rudder angle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [tarray,stateArray]=ode23(@rhs,[t t+a],newstateArray,options,p);
    [nr,nc] = size(stateArray);
    newstateArray = stateArray(nr,:);
    totalTarray = [totalTarray; tarray];
    totalstateArray = [totalstateArray;stateArray];
    end
    %close the loading bar
    close(p.waitBar);
    %animate the resulting trajectory
    animate(totalTarray,totalstateArray,p);
elseif p.realTime == 1 %allow real time control of boat
    %solve for trajectory at each time step in real-time using Euler
    %approximation
    [tarray,stateArray]=odeEuler(@rhs,state0,p);
end
