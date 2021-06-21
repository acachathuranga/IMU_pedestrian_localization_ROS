import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time

figure = None
ax = None
lines = None 
xdata = None
ydata = None 

def show3Dposition(data):
    """ Display 3D visualization of position data.
        
        :param data: Position data matrix with x,y,z positions as first 3 columns
    """
    px = data[:,0]
    py = data[:,1]
    pz = data[:,2]
    plt.figure()
    ax = plt.subplot(111, projection='3d')
    ax.scatter(px, py, pz, color='b')

    # Show best fit plane
    # # do fit
    # tmp_A = []
    # tmp_b = []
    # for i in range(len(px)):
    #     tmp_A.append([px[i], py[i], 1])
    #     tmp_b.append(pz[i])
    # b = np.matrix(tmp_b).T
    # A = np.matrix(tmp_A)
    # fit = (A.T * A).I * A.T * b
    # errors = b - A * fit
    # residual = np.linalg.norm(errors)

    # print ("Plane Solution:")
    # print ("%f x + %f y + %f = z" % (fit[0], fit[1], fit[2]))
    # # print ("errors:", errors)
    # print ("residual:", residual)

    # # plot plane
    # xlim = ax.get_xlim()
    # ylim = ax.get_ylim()

    # step = (xlim[1] - xlim[0]) / 10 + 1
    # X,Y = np.meshgrid(np.arange(xlim[0], xlim[1], step),
    #                 np.arange(ylim[0], ylim[1], step))
    # Z = np.zeros(X.shape)
    # for r in range(X.shape[0]):
    #     for c in range(X.shape[1]):
    #         Z[r,c] = fit[0] * X[r,c] + fit[1] * Y[r,c] + fit[2]
    # ax.plot_wireframe(X,Y,Z, color='k')

    # Show XY Plane
    # plot plane
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()

    step = (xlim[1] - xlim[0]) / 10 + 1
    X,Y = np.meshgrid(np.arange(xlim[0], xlim[1], step),
                    np.arange(ylim[0], ylim[1], step))
    Z = np.zeros(X.shape)
    ax.plot_wireframe(X,Y,Z, color='k')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    # Set Z axis scale from -5 meters to 5 meters
    ax.set_zlim([-5, 5])

    

    plt.show()

def show2Dposition(data, fileName=None):
    """ Display 2D visualization of position data.
        
        :param data: Position data matrix with x,y positions as first 2 columns
    """
    px = data[:,0]
    py = data[:,1]
    plt.plot(px, py)

    if (fileName != None):
        # Save plot to file
        plt.savefig(fileName+'.png')
        print ("Visualization plot saved to " + fileName + ".png")
        
    plt.show()

def interactive2Dposition_init():
    global figure, lines, ax, xdata, ydata

    plt.ion()
    #Set up plot
    figure, ax = plt.subplots()
    lines, = ax.plot([],[], 'o')
    #Autoscale on unknown axis and known lims on the other
    ax.set_autoscaley_on(True)
    # ax.set_xlim(0, 10)
    #Other stuff
    ax.grid()

    xdata = []
    ydata = []


def update2Dposition(x, y):
    global lines, ax, figure, xdata, ydata

    xdata.append(x)
    ydata.append(y)
    #Update data (with the new _and_ the old points)
    lines.set_xdata(xdata)
    lines.set_ydata(ydata)
    #Need both of these in order to rescale
    ax.relim()
    ax.autoscale_view()
    #We need to draw *and* flush
    figure.canvas.draw()
    figure.canvas.flush_events()

    
# Print iterations progress
def printProgressBar (iteration, total, prefix = '', suffix = '', decimals = 1, length = 100, fill = '█', printEnd = "\r"):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
        printEnd    - Optional  : end character (e.g. "\r", "\r\n") (Str)
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print('\r%s |%s| %s%% %s' % (prefix, bar, percent, suffix), end = printEnd)
    # Print New Line on Complete
    if iteration == total: 
        print()