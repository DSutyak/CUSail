function alph=compute_alpha3(th, vr, xdot, thetaboat)
theta.windrel=180/pi*atan2(vr(2),vr(1));
theta.boatvel=180/pi*atan2(xdot(2),xdot(1));

balph.boat=thetaboat-(theta.windrel+180);
balph.s=balph.boat+th.s;
balph.r=balph.boat+th.r;
balph.k=thetaboat-theta.boatvel;
alph.boat=wrapTo360(balph.boat);
alph.r=wrapTo360(balph.r);
alph.s=wrapTo360(balph.s);
alph.k=wrapTo360(balph.k);

end