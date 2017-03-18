import numpy as np
import matplotlib.pyplot as plt

x=[1,2,3,4,5,6]
y=[1,2,3,4,5,6]
z=[6,5,4,3,2,1]

# bottom stairs, bottom left thurston, middle right of x, bottom small stairs
waypoints=[[42.444240, -76.483258],[42.444259, -76.484435],[42.444545, -76.483094],[42.444808, -76.483158]]
gps=[[42.444228, -76.483666],[42.444612, -76.483492],[42.444240, -76.483258],[42.444735, -76.483275],[42.4444792, -76.483244],[42.4448630, -76.4835293]]
appelgps=[[42.454715, -76.474551],[42.454865, -76.475672],[42.453828, -76.474454],[42.453923, -76.476063]]
# coord_t northQuadStair= [42.444751, -76.483267]
# coord_t bottomRightIntersection= [42.444242, -76.483247]
# coord_t thurstonWestEntrance=[42.444245, -76.484068]
# rotate 90 degrees ccw then flip across x by x=y, y=x
wx, wy, gx, gy=[],[],[],[]
for x in waypoints:
  # wx.append(x[0])
  wx.append(x[1])
  wy.append(x[0])
for x in gps:
  gx.append(x[1])
  gy.append(x[0])
for x in appelgps:
  gx.append(x[1])
  gy.append(x[0])
plt.axes().set_aspect('equal', 'datalim')
plt.scatter(wx,wy,c='r')
plt.scatter(gx,gy,c='b')
plt.show()