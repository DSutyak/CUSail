function [c_lift,c_drag]=C_LD(alpha,p)
%%%calculates local coeff. of lift and drag at a given angle of attack
%%%(alpha)

if p.accuracy==1
    %interpolation with pchip
    alpha=alpha*180/pi;
    c_lift=ppval(p.lift,alpha);
    c_drag=ppval(p.drag,alpha);
else
    %approximation as sinusoidal
    c_lift=p.C0*sin(2*alpha);
    c_drag=p.paraDrag+p.C0*(1-cos(2*alpha));
end
