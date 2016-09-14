function [t,stateArray0]=odeEuler(fhandle,stateArray0,ptemp)
%%% allows for user to have active control of sail and rudder

%creates global parameters to be used in subfunctions
p=ptemp;

t=0;
stateArray0=stateArray0';
stateArray=stateArray0;
keyspeed = 1;

%Initialize figure object
f=figure(1);
cla
set(f,'units','normalized','outerposition',[0 0 1 1],'color',[.5,.8,1]);
set(f,'WindowKeyPressFcn',@KeyPress,'WindowKeyReleaseFcn',@KeyRelease);

%Bar scale for up/down arrow key input to control rudder
subplot(2,22,22)
barLim_rudder = [-180 180];   %Bar scale
bar_rudder = bar(0);     %Initialize the bar in the middle of the scale
bar_rudder_title = title(sprintf('Rudder Angle = %d degrees',get(bar_rudder,'ydata')),'fontweight','bold');
ylim(barLim_rudder);       %Set upper limit
xlabel('Up/Down Arrow');
set(gca, 'XTick', []);

%Bar scale for up/down arrow key input to control sail
subplot(2,22,44)
barLim_sail = [-180 180];   %Bar scale
bar_sail = bar(0);     %Initialize the bar in the middle of the scale
ylim(barLim_sail);       %Set upper limit
xlabel('Left/Right Arrow');
set(gca, 'XTick', []);
bar_sail_title=title(sprintf('Sail Angle =  %d degrees',get(bar_rudder,'ydata')),'fontweight','bold');

%Simulation Plot setup
h.ax_xy=subplot(2,22,[1:20,23:42],'color',[.5,.8,1]);
axis('equal')
h.az = 5;
h.el = 30;
h.fps=NaN;
view(h.az, h.el);
hold on
grid on
camproj('perspective')
set(gca,...
'gridlinestyle','--','xcolor',[.3,.6,1],'ycolor',...
    [.3,.6,1],'zcolor',[.3,.6,1],'yticklabel',[],'xticklabel',[],'zticklabel',[])
%initialize plot objects in simulation
h.trueWind=text(0,0,0,'True Wind','HorizontalAlignment','left',...
    'verticalalignment','bottom','FontSize',10,'fontweight','bold');
h.boatStats=text(0,0,0,'True Wind','HorizontalAlignment','left',...
    'verticalalignment','bottom','FontSize',10,'color','w','fontweight','bold');
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

%initialize axis limits
h.limx=[stateArray0(1,1)-2*p.hull.length,stateArray0(1,1)+2*p.hull.length];
h.limy=[stateArray0(1,1)-1.5*p.hull.length,stateArray0(1,1)+1.5*p.hull.length];
h.limz=[stateArray0(1,3)-1.5*max([p.sail.length,p.keel.length,p.rudder.length]),stateArray0(1,3)+1.5*max([p.sail.length,p.keel.length,p.rudder.length])];
 
%Initialize the arrow keys as not being pressed
commands.up = false;
commands.down = false;
commands.left = false;
commands.right = false;
commands.w = false;
commands.s = false;
commands.d = false;
commands.a = false;
set(f,'UserData',commands); %Store these commands in the figure object

%Continuously update until the user closes the figure window
tic
while(ishandle(f))
    
    view(h.az, h.el);
    
    %extract sail and rudder angles from bar plot
    bar2val_rudder = get(bar_rudder,'ydata');
    p.rudder.angle_relBody = pi*bar2val_rudder/180;
    p.rudder.angle_relSail=p.rudder.angle_relBody;
    bar2val_sail=get(bar_sail,'ydata');
    p.sail.angle_relBody=pi*bar2val_sail/180;
    [p.sail.H,p.sail.Ht,p.sail.R,p.sail.Rt]=euler2Hom([0,0,p.sail.angle_relBody],p.sail.origin);
    if p.rudder.type==1
        [p.rudder.H,p.rudder.Ht,p.rudder.R,p.rudder.Rt]=euler2Hom([0,0,p.rudder.angle_relBody],p.rudder.origin);
    elseif p.rudder.type==2
        [p.rudder.H_relSail,p.rudder.Ht_relSail,p.rudder.R_relSail,p.rudder.Rt_relSail]=...
            euler2Hom([0,0,p.rudder.angle_relSail],p.rudder.origin_relSail);
    end
    t(end+1)=toc;
    dt=t(end)-t(end-1);
    state=stateArray(end,:)';
    
    %integrates using Eulers method
    for k=1:p.numInt
        statedot=feval(fhandle,t,state,p);
        state=state+(1/p.numInt)*dt*statedot;
    end
    stateArray(end+1,:)=state';
    
    %animation
    if ~ishandle(f)
        break;
    end
    h=animateRT(t,stateArray,p,h);
    
    %%%% GRAPHICS UPDATE HERE %%%%
   %Get input from the keyboard
   if ishandle(f)
       commands = get(f,'UserData') ;
   end
   
   %Adjust the bar position if the user has given input
   if (commands.up && bar2val_rudder<=barLim_rudder(2))
       set(bar_rudder,'ydata',bar2val_rudder + keyspeed);
       set(bar_rudder_title,'string',sprintf('Rudder Angle = %d degrees',round(get(bar_rudder,'ydata'))));
   elseif (commands.up && bar2val_rudder>=barLim_rudder(2))
       set(bar_rudder,'ydata',barLim_rudder(1));
       set(bar_rudder_title,'string',sprintf('Rudder Angle = %d degrees',round(get(bar_rudder,'ydata'))));
   elseif (commands.down && bar2val_rudder >=barLim_rudder(1))
       set(bar_rudder,'ydata',bar2val_rudder - keyspeed);
       set(bar_rudder_title,'string',sprintf('Rudder Angle = %d degrees',round(get(bar_rudder,'ydata'))));
   elseif (commands.down && bar2val_rudder <=barLim_rudder(1))
       set(bar_rudder,'ydata',barLim_rudder(2));
       set(bar_rudder_title,'string',sprintf('Rudder Angle = %d degrees',round(get(bar_rudder,'ydata'))));
   elseif (commands.right && bar2val_sail <= barLim_sail(2))
       set(bar_sail,'ydata',bar2val_sail + keyspeed);
       set(bar_sail_title,'string',sprintf('Sail Angle = %d degrees',round(get(bar_sail,'ydata'))));
   elseif (commands.right && bar2val_sail >= barLim_sail(2))
       set(bar_sail,'ydata',barLim_sail(1));
       set(bar_sail_title,'string',sprintf('Sail Angle = %d degrees',round(get(bar_sail,'ydata'))));
   elseif (commands.left && bar2val_sail >= barLim_sail(1))
       set(bar_sail,'ydata',bar2val_sail - keyspeed);
       set(bar_sail_title,'string',sprintf('Sail Angle = %d degrees',round(get(bar_sail,'ydata'))));
   elseif (commands.left && bar2val_sail <= barLim_sail(1))
       set(bar_sail,'ydata',barLim_sail(2));
       set(bar_sail_title,'string',sprintf('Sail Angle = %d degrees',round(get(bar_sail,'ydata'))));
   elseif commands.w && (h.el + keyspeed) < 90
       h.el=h.el + keyspeed;
   elseif commands.s && (h.el- keyspeed) > -89
       h.el=h.el - keyspeed;
   elseif commands.d
       h.az=h.az + keyspeed;
   elseif commands.a
       h.az=h.az - keyspeed;
   end
   
   %Pause the program for a tiny amount of time. This is necessary to make
   %it work.
   pause(0.01);
end

%%%%%%%%%%% KEYBOARD CALLBACKS %%%%%%%%%%% 
function KeyPress(varargin)
    %This function is continuously running and checking the keyboard input.
    %It takes in varargin, which stands for variable argument input. 
     fig = varargin{1};
     key = varargin{2}.Key;
     %Change 'UserData' part of the figure object depending on the input.
     if strcmp(key,'uparrow')
         x=get(fig,'UserData'); %Get the UserData
         x.up = true;           %Update the command
         set(fig,'UserData',x); %Put the new UserData back into the object
     elseif strcmp(key,'downarrow')
         x=get(fig,'UserData');
         x.down = true;
         set(fig,'UserData',x);
     elseif strcmp(key,'rightarrow')
         x=get(fig,'UserData');
         x.right=true;
         set(fig,'UserData',x);
     elseif strcmp(key,'leftarrow')
         x=get(fig,'UserData');
         x.left=true;
         set(fig,'UserData',x);
     elseif strcmp(key,'w')
         x=get(fig,'UserData');
         x.w=true;
         set(fig,'UserData',x);
     elseif strcmp(key,'s')
         x=get(fig,'UserData');
         x.s=true;
         set(fig,'UserData',x);
     elseif strcmp(key,'d')
         x=get(fig,'UserData');
         x.d=true;
         set(fig,'UserData',x);
     elseif strcmp(key,'a')
         x=get(fig,'UserData');
         x.a=true;
         set(fig,'UserData',x);
end
end
function KeyRelease(varargin)
    %This function is the same as KeyPress, except it resets the command
    %when the user releases the up/down arrow.
     fig = varargin{1};
     key = varargin{2}.Key;
     if strcmp(key,'uparrow')
         x=get(fig,'UserData');
         x.up = false;
         set(fig,'UserData',x);
     elseif strcmp(key,'downarrow')
         x=get(fig,'UserData');
         x.down = false;
         set(fig,'UserData',x);
     elseif strcmp(key,'rightarrow')
         x=get(fig,'UserData');
         x.right = false;
         set(fig,'UserData',x);
     elseif strcmp(key,'leftarrow')
         x=get(fig,'UserData');
         x.left = false;
         set(fig,'UserData',x)
     elseif strcmp(key,'w')
         x=get(fig,'UserData');
         x.w=false;
         set(fig,'UserData',x);
     elseif strcmp(key,'s')
         x=get(fig,'UserData');
         x.s=false;
         set(fig,'UserData',x);
     elseif strcmp(key,'d')
         x=get(fig,'UserData');
         x.d=false;
         set(fig,'UserData',x);
     elseif strcmp(key,'a')
         x=get(fig,'UserData');
         x.a=false;
         set(fig,'UserData',x);
     end
end

end