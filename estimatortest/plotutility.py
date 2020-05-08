#utility.py
#Nathan Zimmerberg (nhz2)
#7 MAY 2020
""" Utilities for estimator testing plotting."""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import copy

def estimatorscope(estimator,truth,sensors,listofest,samplerate,traces):
    """Return an matlplotlib animation of the estimator running
       
        Args:
            estimator(An object with a input function that accepts a numpy structure):
                The estimator object.
            truth(numpy structured array indexed by control cycle):
                The true state to estimate.
            sensors(numpy structured array indexed by control cycle):
                The sensor data to input to the estimator each control cycle.
            listofest(empty list):
                List to fill with copies of estimator after every samplerate cycles.
                Does not include the initial state.
            samplerate(positive int):
                Sample rate for ploting and saving estimator, 1 is every cycle.
            traces(list of functions traces[j](estimator,sensors[i],truth[i])-> float):
                Things to plot."""
    #trial run to catch errors
    e_init=copy.deepcopy(estimator)
    e_init.input(sensors[0])
    for trace in traces:
        assert(type(trace(e_init,sensors[0],truth[0]))==float)
    # create figure and axes 
    fig, axes = plt.subplots(len(traces), figsize=(10, 5))
    lines=[]
    
    numcycles= len(sensors)
    numdata= numcycles//samplerate
    numtraces= len(traces)
    datas=[np.full(numdata,np.nan) for i in range(numtraces)]
    if numtraces==1:
        axes=[axes]
    for i in range(numtraces):
        line, = axes[i].plot(datas[i], 'ro')
        lines.append(line)
        axes[i].axis(xmin=0,xmax=numdata)
        axes[i].set_title(str(traces[i]))
    #create animation update function
    def init():
        return lines
    def update(frame_num):
        cyclenum= frame_num*samplerate
        for i in range(samplerate):
            estimator.input(sensors[cyclenum])
            cyclenum+= 1
        listofest.append(copy.deepcopy(estimator))
        for i in range(numtraces):
            datas[i][frame_num]=traces[i](estimator,sensors[cyclenum-1],truth[cyclenum-1])
            lines[i].set_ydata(datas[i])
            axes[i].relim()
            axes[i].autoscale_view(scalex=True, scaley=True)
        return lines
    line_ani = animation.FuncAnimation(fig, update, range(numdata), init_func= init,
                                   interval=1, blit=True,repeat=False)
    return line_ani