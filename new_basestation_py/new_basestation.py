import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure

from tkinter import *
import tkinter as tk



LARGE_FONT= ("Verdana", 12)

class CUSailGUI(tk.Tk):

	def __init__(self, *args, **kwargs):

		tk.Tk.__init__(self, *args, **kwargs)

		# tk.Tk.iconbitmap(self, default="clienticon.ico")
		tk.Tk.wm_title(self, "CUSail GUI")


		container = tk.Frame(self)
		container.pack(side="top", fill="both", expand = True)
		container.grid_rowconfigure(0, weight=1)
		container.grid_columnconfigure(0, weight=1)

		self.frames = {}


		frame = PageThree(container, self)

		self.frames = frame

		frame.grid(row=0, column=0, sticky="nsew")

	################


class PageThree(tk.Frame):

	def __init__(self, parent, controller):
		tk.Frame.__init__(self, parent)
		label = tk.Label(self, text="GUI", font=LARGE_FONT)
		label.pack(pady=10,padx=10)

		lbl = Label(self, text = "Hello")
		lbl2 = Label(self, text = "Hello world")
		#lbl.grid(column = 0, row = 0)
		#lbl2.grid(column = 0, row = 50)

		def callback():
			print ("click")


		Button(self, text="test", command=callback) #.grid(column=4, row=4)




		"""
        f = Figure(figsize=(5,5), dpi=100)
        a = f.add_subplot(111)
        a.plot([1,2,3,4,5,6,7,8],[5,6,1,3,8,9,3,5])


        canvas = FigureCanvasTkAgg(f, self)
        canvas.show()
        canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        toolbar = NavigationToolbar2TkAgg(canvas, self)
        toolbar.update()
        canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        """
		frame = tk.Frame(self)
		fig = Figure()
		ax = fig.add_subplot(111)
		self.line, = ax.plot(range(10))

		self.canvas = FigureCanvasTkAgg(fig,self)
		self.canvas.show()
		self.canvas.get_tk_widget().pack(side='top', fill='both', expand=1)
		frame.pack()


app = CUSailGUI()
app.mainloop()
