import imageio
import matplotlib.pyplot as plt
import sys
im = imageio.imread(sys.argv[1])

#print(im.shape)


fig, ax = plt.subplots()
ax.imshow(im)
def onclick(event):
    if (event.button==2):
        print(event.ydata,event.xdata)

cid = fig.canvas.mpl_connect('button_press_event', onclick)

plt.show()
