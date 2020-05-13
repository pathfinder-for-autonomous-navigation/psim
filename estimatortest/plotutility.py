#utility.py
#Nathan Zimmerberg (nhz2)
#7 MAY 2020
""" Utilities for estimator testing plotting."""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import copy

def estimatorscope(estimator,truth,sensors,listofoutputs,samplerate,traces):
    """Return an matlplotlib animation of the estimator running
       
        Args:
            estimator(An object with an update function that accepts a numpy structure):
                The estimator object.
            truth(numpy structured array indexed by control cycle):
                The true state to estimate.
            sensors(numpy structured array indexed by control cycle):
                The sensor data to input to the estimator each control cycle.
            listofoutputs(empty list):
                fill with a list of copies of estimator after every samplerate cycles.
                Also filled with a list of numpy arrays for each of the traces.
            samplerate(positive int):
                Sample rate for ploting and saving estimator, 1 is every cycle.
            traces(list of functions traces[j](estimator,sensors[i],truth[i])-> float):
                Things to plot."""
    #trial run to catch errors
    e_init=copy.deepcopy(estimator)
    e_init.update(sensors[0])
    numtraces= len(traces)
    for trace in traces:
        assert(type(trace(e_init,sensors[0],truth[0]))==float)
    # create figure and axes 
    fig, axes = plt.subplots(len(traces), figsize=(10, 5*numtraces))
    lines=[]
    numcycles= len(sensors)
    numdata= numcycles//samplerate
    listofest= []
    datas=[np.full(numdata,np.nan) for i in range(numtraces)]
    controlcycledata= np.arange(samplerate-1,numcycles,samplerate)
    assert len(controlcycledata)== numdata
    listofoutputs.append(listofest)
    listofoutputs.append(datas)
    if numtraces==1:
        axes=[axes]
    for i in range(numtraces):
        line, = axes[i].plot(controlcycledata,datas[i], 'ro')
        lines.append(line)
        axes[i].axis(xmin=0,xmax=numcycles)
        axes[i].set_xlabel('Control Cycle')
        axes[i].set_title(str(traces[i]))
    #create animation update function
    def init():
        return lines
    def update(frame_num):
        cyclenum= frame_num*samplerate
        for i in range(samplerate):
            estimator.update(sensors[cyclenum])
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

def estimatorsilentscope(estimator,truth,sensors,listofoutputs,samplerate,traces):
    """estimatorscope but with no animation and blocking.
       
        Args:
            estimator(An object with an update function that accepts a numpy structure):
                The estimator object.
            truth(numpy structured array indexed by control cycle):
                The true state to estimate.
            sensors(numpy structured array indexed by control cycle):
                The sensor data to input to the estimator each control cycle.
            listofoutputs(empty list):
                fill with a list of copies of estimator after every samplerate cycles.
                Also filled with a list of numpy arrays for each of the traces.
            samplerate(positive int):
                Sample rate for ploting and saving estimator, 1 is every cycle.
            traces(list of functions traces[j](estimator,sensors[i],truth[i])-> float):
                Things to plot."""
    numcycles= len(sensors)
    numdata= numcycles//samplerate
    numtraces= len(traces)
    listofest= []
    datas=[np.full(numdata,np.nan) for i in range(numtraces)]
    listofoutputs.append(listofest)
    listofoutputs.append(datas)
    def update(frame_num):
        cyclenum= frame_num*samplerate
        for i in range(samplerate):
            estimator.input(sensors[cyclenum])
            cyclenum+= 1
        listofest.append(copy.deepcopy(estimator))
        for i in range(numtraces):
            datas[i][frame_num]=traces[i](estimator,sensors[cyclenum-1],truth[cyclenum-1])
    for i in range(numdata):
        update(i)
    return None