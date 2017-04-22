import numpy as np
import matplotlib.pyplot as plt
import math

import sys

def gui(file_name):
  """ open the log file and read only the lines that start with Latitude or
  Longitude and store them in the waypoints array"""
  F = open(file_name,"r")
  lines= F.readlines()
  waypoints=[]
  gps=[]
  wind_values=[]
  raw_ascii=""
  # convert log file in hex to a string in ascii
  for x in range(1,len(lines)):
    raw=lines[x]
    index=raw.find("RECV,")+5
    hex_string=raw[index:-1]
    ascii_string=hex_string.decode("hex")
    raw_ascii+=ascii_string

  # split the string by line
  lines_ascii=raw_ascii.split("\n")
  # look in the lines for Lat Lon coordinates
  for x in range(0,len(lines_ascii)):
    string_data=lines_ascii[x]
    print string_data
    # print string_data
    # filter out lines that start w "Latitude"
    if len(string_data)>8 and string_data[:8]=='Latitude':
      # get the latitude coordinate and the longitude coord from the next line
      # print string_data
      # print lines_ascii[x+1]
      lat=float(string_data[10:-1])
      # increment the counter
      # x+=1
      lon=float(lines_ascii[x+1][10:-1])
      # add to the gps coordinates
      if lat != 0 and lon !=0:
        gps.append([lat,lon])
      # print lat,lon
    if len(string_data)>13 and string_data[:13]=="Next Waypoint":
      # get lat and lon of the waypoint from the line
      index1=string_data.index(',')
      lat= float(string_data[18:index1])
      lon= float(string_data[index1+2:-1])
      # if we havent added the waypoint to the waypoints array, add it
      # probably can be removed, or made more efficient so as not to search
      if [lat,lon] not in waypoints:
        waypoints.append([lat,lon])
    if len(string_data)>12 and string_data[:4]=="Wind":
      index=string_data.index(':')
      wind_value=float(string_data[index+1:])
      # print wind_value
      wind_values.append(wind_value)

      # plot x, y, u, v

  vector_length=.0005

  # gps.append([0,10])
  # wind_values.append(270)

  xs, ys, us, vs=[],[],[],[]
  for index in range(0,len(gps)):
    if index%20==0:
      x=gps[index][0]
      y=gps[index][1]
      wind=wind_values[index]
      # convert to radian coordinate frame
      wind=(450-wind)%360
      wind_radian=wind*math.pi/180
      # print wind
      u=(vector_length*math.cos(wind_radian))
      v=(vector_length*math.sin(wind_radian))
    # rotate 90 degrees ccw then flip across x by x=y, y=x
      if abs(x)>1 and abs(y)>1:
        xs.append(y)
        ys.append(x)
        us.append(-u)
        vs.append(-v)

  # print xs, ys, us, vs




  # bottom stairs, bottom left thurston, middle right of x, bottom small stairs
  # test_waypoints=[[42.444240, -76.483258],[42.444259, -76.484435],[42.444545, -76.483094],[42.444808, -76.483158]]
  # waypoints+=test_waypoints
  # testing gps
  # test_gps=[[42.444228, -76.483666],[42.444612, -76.483492],[42.444240, -76.483258],[42.444735, -76.483275],[42.4444792, -76.483244],[42.4448630, -76.4835293]]
  # gps+=test_gps
  # rotate 90 degrees ccw then flip across x by x=y, y=x
  wx, wy, gx, gy=[],[],[],[]
  for x in waypoints:
    # print x
    if abs(x[0])>1 and abs(x[1])>1:
      wx.append(x[1])
      wy.append(x[0])
  for x in gps:
    # print x[0], x[1]
    if abs(x[0])>1 and abs(x[1])>1:
      gx.append(x[1])
      gy.append(x[0])
  plt.axes().set_aspect('equal', 'datalim')
  plt.scatter(wx,wy,c='r')
  plt.scatter(gx,gy,c='b')
  # x, y, u, v
  plt.quiver(xs,ys,us,vs,angles='xy',scale_units='xy', scale=1)
  plt.draw()
  plt.show()

# main loop of program
if __name__ == "__main__":
  # makes sure gui is run with a file as the target
  if len(sys.argv)!=2:
    print "RUN WITH python gui.py FILENAME.log"
  else:
    file_name=sys.argv[1]
    gui(file_name)


