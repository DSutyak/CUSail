import numpy as np
import matplotlib.pyplot as plt

import sys

def gui(file_name):
  """ open the log file and read only the lines that start with Latitude or
  Longitude and store them in the waypoints array"""
  F = open(file_name,"r")
  lines= F.readlines()
  waypoints=[]
  gps=[]
  for x in range(0,len(lines)):
    if lines[x][:8]=='Latitude':
      lat=float(lines[x][10:-1])
      x+=1
      lon=float(lines[x][10:-1])
      gps.append([lat,lon])


  # bottom stairs, bottom left thurston, middle right of x, bottom small stairs
  test_waypoints=[[42.444240, -76.483258],[42.444259, -76.484435],[42.444545, -76.483094],[42.444808, -76.483158]]
  waypoints+=test_waypoints
  # testing gps
  test_gps=[[42.444228, -76.483666],[42.444612, -76.483492],[42.444240, -76.483258],[42.444735, -76.483275],[42.4444792, -76.483244],[42.4448630, -76.4835293]]
  gps+=test_gps
  # rotate 90 degrees ccw then flip across x by x=y, y=x
  wx, wy, gx, gy=[],[],[],[]
  for x in waypoints:
    # wx.append(x[0])
    wx.append(x[1])
    wy.append(x[0])
  for x in gps:
    gx.append(x[1])
    gy.append(x[0])
  plt.axes().set_aspect('equal', 'datalim')
  plt.scatter(wx,wy,c='r')
  plt.scatter(gx,gy,c='b')
  plt.show()

# main loop of program
if __name__ == "__main__":
  # makes sure gui is run with a file as the target
  if len(sys.argv)!=2:
    print "RUN WITH python gui.py FILENAME.txt"
  else:
    file_name=sys.argv[1]
    gui(file_name)


