function animate(t,stateArray,p)
%%% Animates boat's motion

%set up figure
f=figure(1);
clf
set(f,'units','normalized','outerposition',[0 0 1 1],'color',[.5,.8,1]);

%unpack values
hull_l=p.hull.length; hull_r=p.hull.radius;
sail_w=p.sail.width; sail_l=p.sail.length;
keel_w=p.keel.width; keel_l=p.keel.length;
rudder_w=p.rudder.width; rudder_l=p.rudder.length;
buffer=0.6*hull_l;


%initialize axis limits
limx=[stateArray(1,1)-2*hull_l,stateArray(1,1)+2*hull_l];
limy=[stateArray(1,1)-1*hull_l,stateArray(1,1)+1*hull_l];
limz=[min(stateArray(:,3))-max([sail_l,keel_l,rudder_l]),max(stateArray(:,3))+max([sail_l,keel_l,rudder_l])];


h.ax_xy=subplot(2,22,[1:17,23:39],'color',[.5,.8,1]);
axis('equal')
az = 5;
el = 30;
view(az, el);
hold on
grid on
camproj('perspective')
set(gca,'Unit','normalized','Position',[0 0 1 1],...
'gridlinestyle','--','xcolor',[.3,.6,1],'ycolor',...
    [.3,.6,1],'zcolor',[.3,.6,1],'yticklabel',[],'xticklabel',[],'zticklabel',[])
%initialize plot objects in simulation
h.trueWind=text(0,0,0,'True Wind','HorizontalAlignment','left',...
    'verticalalignment','bottom','FontSize',12,'fontweight','bold');
h.trueWindArrow=plot3(0,0,0,'k','linewidth',2);
h.trueWindArrowStart=plot3(0,0,0,'ko','linewidth',2,'markersize',10);
h.trajectory=plot3(0,0,0,'color','k','linewidth',1);
h.hull1_xy=fill3(0,0,0,'k','linewidth',1,'edgecolor','k','FaceAlpha', 0.1);
h.hull2_xy=fill3(0,0,0,[0.3,0.3,0.3],'linewidth',1,'edgecolor','none');
h.hull3_xy=fill3(0,0,0,[0,0,0],'linewidth',1,'edgecolor','none');
h.hull4_xy=fill3(0,0,0,[0.4,0.4,0.4],'linewidth',1,'edgecolor','none');
h.hull5_xy=fill3(0,0,0,[0.1,0.1,0.1],'linewidth',1,'edgecolor','none');
h.hull6_xy=fill3(0,0,0,[0.2,0.2,0.2],'linewidth',1,'edgecolor','none');
h.sail_xy=fill3(0,0,0,[1 .5 0],'linewidth',3,'edgecolor',[1 .5 0],'FaceAlpha', 0.7);
h.keel_xy=fill3(0,0,0,'b','linewidth',3,'edgecolor','b','FaceAlpha', 0.7);
h.rudder_xy=fill3(0,0,0,'g','linewidth',3,'edgecolor','g','FaceAlpha', 0.7);
h.water_xy=fill3(zeros(1,5),zeros(1,5),zeros(1,5),[.3,.6,1],'FaceAlpha',...
    0.2,'edgecolor','none');

% p.ax_yz=subplot(2,22,40:44,'color',[.5,.8,1]);
% plot([1],[3])

%for each instant in time
for m=1:length(t)
    if ~ishandle(f)
        break;
    end
    %extract values
    x=stateArray(m,1);
    y=stateArray(m,2);
    z=stateArray(m,3);
    phi=stateArray(m,4); %roll
    theta=stateArray(m,5); %pitch
    psi=stateArray(m,6); %yaw
    
    %plot trajectory so far
    set(h.trajectory,'xdata',stateArray(1:m,1),'ydata',stateArray(1:m,2),'zdata',stateArray(1:m,3));
    
    windArrow(1,1:3)=[limx(1)+0.5*hull_l,mean(limy),mean(limz)+0.5*(limz(2)-mean(limz))];
    wind_norm=p.v_air_relFixed'/norm(p.v_air_relFixed);
    windArrow(2,:)=windArrow(1,:)+wind_norm*hull_l*0.5;
    
    set(h.trueWind,'position',windArrow(1,:))
    
    set(h.trueWindArrow,'xdata',windArrow(:,1),'ydata',windArrow(:,2),...
    'zdata',windArrow(:,3))

    set(h.trueWindArrowStart,'xdata',windArrow(1,1),'ydata',windArrow(1,2),...
    'zdata',windArrow(1,3))
    
    %plot > rotate > translate hull
    H=euler2Hom([phi,theta,psi],[x,y,z]);
    H_hull=H*p.hull.H;
    x_hull=[-0.5*hull_l,0.05*hull_l,0.5*hull_l,0.05*hull_l,-0.5*hull_l,-0.5*hull_l];
    y_hull=[-.08*hull_l,-.15*hull_l,0,.15*hull_l,.08*hull_l,-.08*hull_l];
    z_hull=zeros(size(x_hull));
    p0_hull=[x_hull;y_hull;z_hull;zeros(size(x_hull))+1];
    p_hull=H_hull*p0_hull;
    set(h.hull1_xy,'xdata',p_hull(1,:),'ydata',p_hull(2,:),'zdata',p_hull(3,:));
    
    x_hull=[-0.5*hull_l,0.05*hull_l,0.05*hull_l,-0.5*hull_l,-0.5*hull_l];
    y_hull=[.08*hull_l,.15*hull_l,.15*hull_l,.08*hull_l,.08*hull_l];
    z_hull=[0,0,-0.7*hull_r,-0.5*hull_r,0];
    p0_hull=[x_hull;y_hull;z_hull;zeros(size(x_hull))+1];
    p_hull=H_hull*p0_hull;
    set(h.hull2_xy,'xdata',p_hull(1,:),'ydata',p_hull(2,:),'zdata',p_hull(3,:));
    
    x_hull=[-0.5*hull_l,-0.5*hull_l,-0.5*hull_l,-0.5*hull_l,-0.5*hull_l];
    y_hull=[-.08*hull_l,.08*hull_l,.08*hull_l,-.08*hull_l,-.08*hull_l];
    z_hull=[0,0,-0.5*hull_r,-0.5*hull_r,0];
    p0_hull=[x_hull;y_hull;z_hull;zeros(size(x_hull))+1];
    p_hull=H_hull*p0_hull;
    set(h.hull3_xy,'xdata',p_hull(1,:),'ydata',p_hull(2,:),'zdata',p_hull(3,:));

    x_hull=[0.05*hull_l,0.5*hull_l,0.05*hull_l,0.05*hull_l];
    y_hull=[.15*hull_l,0,.15*hull_l,.15*hull_l];
    z_hull=[0,0,-0.7*hull_r,0];
    p0_hull=[x_hull;y_hull;z_hull;zeros(size(x_hull))+1];
    p_hull=H_hull*p0_hull;
    set(h.hull4_xy,'xdata',p_hull(1,:),'ydata',p_hull(2,:),'zdata',p_hull(3,:));
    
    x_hull=[-0.5*hull_l,0.05*hull_l,0.05*hull_l,-0.5*hull_l,-0.5*hull_l];
    y_hull=[-.08*hull_l,-.15*hull_l,-.15*hull_l,-.08*hull_l,-.08*hull_l];
    z_hull=[0,0,-0.7*hull_r,-0.5*hull_r,0];
    p0_hull=[x_hull;y_hull;z_hull;zeros(size(x_hull))+1];
    p_hull=H_hull*p0_hull;
    set(h.hull5_xy,'xdata',p_hull(1,:),'ydata',p_hull(2,:),'zdata',p_hull(3,:));
    
    x_hull=[0.05*hull_l,0.5*hull_l,0.05*hull_l,0.05*hull_l];
    y_hull=[-.15*hull_l,0,-.15*hull_l,-.15*hull_l];
    z_hull=[0,0,-0.7*hull_r,0];
    p0_hull=[x_hull;y_hull;z_hull;zeros(size(x_hull))+1];
    p_hull=H_hull*p0_hull;
    set(h.hull6_xy,'xdata',p_hull(1,:),'ydata',p_hull(2,:),'zdata',p_hull(3,:));
    
    %H_sailRelBody=euler2Hom([0,0,p.sail.angle_relBody],p.sail.origin);
    H_sail=H*p.sail.H;
    %plot > rotate > translate hull
    x_sail=[-sail_w/2,sail_w/2,sail_w/2,-sail_w/2,-sail_w/2];
    y_sail=zeros(size(x_sail));
    z_sail=[0,0,sail_l,sail_l,0]*p.sail.direction;
    p0_sail=[x_sail;y_sail;z_sail;zeros(size(x_sail))+1];
    p_sail=H_sail*p0_sail;
    set(h.sail_xy,'xdata',p_sail(1,:),'ydata',p_sail(2,:),'zdata',p_sail(3,:));
    
    H_keel=H*p.keel.H;
    %plot > rotate > translate hull
    x_keel=[-keel_w/2,keel_w/2,keel_w/2,-keel_w/2,-keel_w/2];
    y_keel=zeros(size(x_keel));
    z_keel=[0,0,keel_l,keel_l,0]*p.keel.direction;
    p0_keel=[x_keel;y_keel;z_keel;zeros(size(x_keel))+1];
    p_keel=H_keel*p0_keel;
    set(h.keel_xy,'xdata',p_keel(1,:),'ydata',p_keel(2,:),'zdata',p_keel(3,:));

    if p.rudder.type==1
        H_rudder=H*p.rudder.H;
    elseif p.rudder.type==2
        H_rudder=H*p.sail.H*p.rudder.H_relSail;
    end
    %plot > rotate > translate hull
    x_rudder=[-rudder_w/2,rudder_w/2,rudder_w/2,-rudder_w/2,-rudder_w/2];
    y_rudder=zeros(size(x_rudder));
    z_rudder=[0,0,rudder_l,rudder_l,0]*p.rudder.direction;
    p0_rudder=[x_rudder;y_rudder;z_rudder;zeros(size(x_rudder))+1];
    p_rudder=H_rudder*p0_rudder;
    set(h.rudder_xy,'xdata',p_rudder(1,:),'ydata',p_rudder(2,:),'zdata',p_rudder(3,:));
    
    set(h.water_xy,'xdata',[limx,limx(2:-1:1),limx(1)],'ydata',[limy(1),limy(1),limy(2),limy(2)]);
    
    if (x-buffer)-limx(1)<0
        Lx=(x-buffer)-limx(1);
    elseif (x+buffer)-limx(2)>0
        Lx=(x+buffer)-limx(2);
    else
        Lx=0;
    end
    if (y-buffer)-limy(1)<0
        Ly=(y-buffer)-limy(1);
    elseif (y+buffer)-limy(2)>0
        Ly=(y+buffer)-limy(2);
    else
        Ly=0;
    end
    limx=limx+Lx; limy=limy+Ly;
    axis([limx,limy,limz]);
    
    pause(0.035)
end

color='b';
figure(5)
plot(stateArray(:,1),stateArray(:,2),color,'linewidth',2)
hold on
ylabel('y-position [m]','fontsize',18)
xlabel('x-position [m]','fontsize',18)
axis equal

figure(6)
plot(t,atan2(stateArray(:,2),stateArray(:,1)),color,'linewidth',2)
hold on
ylabel('Boat Velocity Direction [rad]','fontsize',18)
xlabel('Time [sec]','fontsize',18)

figure(7)
plot(t,stateArray(:,4),color,'linewidth',2)
hold on
ylabel('Lean Angle [rad]','fontsize',18)
xlabel('Time [sec]','fontsize',18)