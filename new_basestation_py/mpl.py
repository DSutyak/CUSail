import matplotlib
matplotlib.use('TkAgg')
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from tkinter import *

class mclass:
    def __init__(self,  window):
        self.window = window
        self.window.resizable(0,0)
        # self.graph = Canvas(window, width=600, height=600)
        # self.graph.create_line(0, 600, 600, 0, fill="red", dash=(4, 4))
        # self.box = Entry(window)
        self.boatmode = Canvas(window, width=200, height=50, bg="green")
        self.waypoints = Canvas (window, width=200, height=275, bg="red")
        self.bouypoints = Canvas (window, width=200, height=275, bg="blue")
        self.info = Frame (window, bg="purple", height=100, width=800)
        # self.graph.grid(row=0, column=0)
        self.boatmode.grid(row=0,column=1)
        self.waypoints.grid(row=1, column=1)
        self.bouypoints.grid(row=2, column=1)
        self.info.grid(row=3, column=0, columnspan=2)

        #draws a black x in each box (practicing making changes)
       # self.boatmode.create_line(0, 0, 200, 50)
       # self.boatmode.create_line(200, 0, 0, 50)
       # self.waypoints.create_line(0, 0, 200, 275)
       # self.waypoints.create_line(200, 0, 0, 275)
       # self.bouypoints.create_line(0, 0, 200, 275)
       # self.bouypoints.create_line(200, 0, 0, 275)


        #text to display in right-hand side boxes
        self.boatmode.create_text(70, 5, anchor= "nw", text="Boatmode")
        self.waypoints.create_text(70, 5, anchor= "nw", text="Waypoints")
        self.bouypoints.create_text(70, 5, anchor= "nw", text="Bouypoints")

        #waypoint entries
        e = Entry(window, text="Waypoints", width=10)
        self.waypoints.create_window(90, 37, anchor="nw",window=e)
        print(e.get())

        #make buttons to add waypoints
        def callback():
            print('clicked')
        b=Button(window, text="Waypoints", command=callback)
        self.waypoints.create_window(10, 40, anchor="nw", window=b)


        #add a window on top of the canvas
        self.waypoints.create_window(0, 0)

        p = np.array ([])

        fig = Figure(figsize=(6,6))
        a = fig.add_subplot(111)
        a.plot(p, range(0),color='blue')

        a.set_title ("Boat Position", fontsize=16)

        self.canvas = FigureCanvasTkAgg(fig, master=self.window)
        # self.canvas.autoscale(enable=True)
        self.canvas.get_tk_widget().grid(row=0, rowspan=3, column=0)
        self.canvas.draw()

    def update (self, data):
        arr=[]

window= Tk()
start= mclass(window)
window.mainloop()


#want to rename the window so instead of saying tk it says "CUSail GUI or new_basestation"
#want to resize the window so that it fits in my computer screen (currently it is too tall)
#implement into xbeeComm code
