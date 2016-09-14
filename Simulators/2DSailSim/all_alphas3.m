function bnew=all_alphas3();
%Creates Coefficient of Lift (CL) and Coefficient of Drag(CD) matrices 
%  given an alpha(angle of attack).  Data is also included to compare the 
...experimental data for all angles of attack to the data using experimental
    ...for low alphas, and theoretical data of a flat plate at high alphas.    

%Theoretical Data for high angles of attack. Used only for comparison
al=linspace(0,180,180*4);
co=1.2; cp=0.15;
cl=co*sind(2*al);  
cd=cp+co*(1-cosd(2*al));
%Experimental Data for low angles of attack. Used only for comparison
lowCD=[0, 0.019420551,0.064626903, 0.129513735,0.17054996,0.264391909,0.329999248,0.450253492,0.684472537,1.11595];
lowalpha=[0,3,6,9,12,15,18,21,25,30];
lowCL=[0.00, 0.37, 0.62, 0.84, 0.80, 1.01, 1.02, 1.17, 1.47, 1.93];

%Experimental Data
b=dlmread('0012-160000-2.txt');
olda=b(:,2);
oldcl=1.2*b(:,3);
oldcd=b(:,4);
newb=[olda, oldcl, oldcd];
fineb=[newb; 360-olda(2:end-1), -1*oldcl(2:end-1), oldcd(2:end-1)];

[an,bn]=sort(fineb(:,1)); bnew=(fineb(bn,:));

end