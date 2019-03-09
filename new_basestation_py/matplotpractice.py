import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox
from matplotlib import gridspec
from matplotlib import text



fig = plt.figure(constrained_layout=False)

# create grid, 1 row 4 columns
gs = gridspec.GridSpec(1, 4, figure=fig)

# plot takes up 3 columns
ax = fig.add_subplot(gs[:3])

plt.title(s='GUI')

# Textbox labels
fig.text(0.85, 0.5, s='Waypoints', verticalalignment='top', horizontalalignment='center')

fig.text(0.85, 0.7, s='Buoy Points', verticalalignment='top', horizontalalignment='center')


# plt.subplots_adjust(bottom=0.2)
# t = np.arange(-2.0, 2.0, 0.001)
# s = t ** 2
# initial_text = "t ** 2"
# l, = plt.plot(t, s, lw=2)

# Waypoints textbox
axbox = plt.axes([0.72, 0.4, 0.26, 0.06])
text_box = TextBox(axbox,"")

# Buoy Points textbox
axbox2 = plt.axes([0.72, 0.6, 0.26, 0.06])
text_box2 = TextBox(axbox2,"")

dict = {}

i = 0

# Waypoints are red
def submit_waypoints(text):
    arr = text.split(',')

    x = float(arr[0])
    y = float(arr[1])

    ax.plot([x], [y], marker='o', markersize=13, color="red")

    #i=0
    dict[i] = str(x) + ', ' + str(y)
    i = i+1
    #, int(arr[1])) #, int(arr[1]), marker='o', markersize=3, color="red")
    # ax.set_ylim(np.min(ydata), np.max(ydata))
    # plt.draw()
    # plt.show()
    #plt.text(0.49, 1.45, s='Waypoints', verticalalignment='top', horizontalalignment='center')

# Buoy Points are blue
def submit_buoypoints(text):
    arr = text.split(',')

    x = float(arr[0])
    y = float(arr[1])

    ax.plot([x], [y], marker='o', markersize=13, color="blue")


text_box.on_submit(submit_waypoints)

text_box2.on_submit(submit_buoypoints)



plt.show()

for value in dict.values():
    print(value)

    # import numpy as np
    # import matplotli1111111b.pyplot as plt
    # from matplotlib.widgets import Button

    # freqs = np.arange(2, 20, 3)

    # fig, ax = plt.subplots()
    # plt.subplots_adjust(bottom=0.2)
    # t = np.arange(0.0, 1.0, 0.001)
    # s = np.sin(2*np.pi*freqs[0]*t)
    # l, = plt.plot(t, s, lw=2)11


    # class Index(object):
    #     ind = 0

    #     def next(self, event):
    #         self.ind += 1
    #         i = self.ind % len(freqs)
    #         ydata = np.sin(2*np.pi*freqs[i]*t)
    #         l.set_ydata(ydata)
    #         plt.draw()

    #     def prev(self, event):
    #         self.ind -= 1
    #         i = self.ind % len(freqs)
    #         ydata = np.sin(2*np.pi*freqs[i]*t)
    #         l.set_ydata(ydata)
    #         plt.draw()

    # callback = Index()
    # axprev = plt.axes([0.7, 0.05, 0.1, 0.075])
    # axnext = plt.axes([0.81, 0.05, 0.1, 0.075])
    # bnext = Button(axnext, 'Next')
    # bnext.on_clicked(callback.next)
    # bprev = Button(axprev, 'Previous')
    # bprev.on_clicked(callback.prev)

    # plt.show()
