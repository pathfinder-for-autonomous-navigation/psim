#gengracetest.py
#Nathan Zimmerberg (nhz2)
#6 MAY 2020
""" Script for generating orbit estimator test data 
from GRACE data"""

import numpy as np
import h5py
from utility import getgitinfo
import sys
import time


def gracefile2array(gracefilename):
    """Return a tuple of three numpy arrays of gpstime (ns), position (m), and velocity (m/s)
    all in ECEF from the GRACE file named gracefilename"""
    ts= []
    rs= []
    vs= []
    with open(gracefilename) as f:
        start= False
        for line in f:
            if start:
                try:
                    linelist= line.split(' ')
                    t= int(linelist[0])
                    r= [float(linelist[i+3]) for i in range(3)]
                    v= [float(linelist[i+9]) for i in range(3)]
                    ts.append(t)
                    rs.append(r)
                    vs.append(v)
                except:
                    pass
            if ("# End of YAML header" in line):
                start= True
        ts= np.array(ts)
        rs= np.array(rs)
        vs= np.array(vs)
        ts= (ts+630763200)*1_000_000_000
    return (ts,rs,vs)


def main(h5file,seed,gracefilename,control_cycle_ns,gpsrbias_sdev,gpsvbias_sdev,gpsrnoise_sdev,gpsvnoise_sdev):
    """Creates a hdf5 file to test an Orbit estimator
    
    Args:
        h5file(h5py._hl.files.File): Newly made file to write to.
        seed(int or None): seed for random noise and biases.
        gracefilename(str): name of grace file example 'GNV1B_2019-10-02_D_04.txt'.
        control_cycle_ns(int): nanoseconds per control cycle.
        gpsrbias_sdev(float): gps position bias standard deviation (m).
        gpsvbias_sdev(float): gps velocity bias standard deviation (m/s).
        gpsrnoise_sdev(float): gps position noise standard deviation (m).
        gpsvnoise_sdev(float): gps velocity noise standard deviation (m/s).
    """
    inputstr= str(locals())
    (ts,rs,vs)= gracefile2array(gracefilename)
    rng = np.random.default_rng(seed)
    gpsrbias= gpsrbias_sdev*rng.normal(size=3)
    gpsvbias= gpsvbias_sdev*rng.normal(size=3)
    gpsts=[]
    gpsrs=[]
    gpsvs=[]
    truets=[]
    truers=[]
    truevs=[]
    i= 0
    sat_time=ts[0]
    while(sat_time<=ts[-1]):
        truets.append(ts[i])
        truers.append(rs[i]+gpsrbias+gpsrnoise_sdev*rng.normal(size=3))
        truevs.append(vs[i]+gpsvbias+gpsvnoise_sdev*rng.normal(size=3))
        if (sat_time>=ts[i]):
            #get a GPS reading
            gpsts.append(ts[i])
            gpsrs.append(rs[i]+gpsrbias+gpsrnoise_sdev*rng.normal(size=3))
            gpsvs.append(vs[i]+gpsvbias+gpsvnoise_sdev*rng.normal(size=3))
            i+= 1
        else:
            gpsts.append(0)
            gpsrs.append(np.full(3,np.nan))
            gpsvs.append(np.full(3,np.nan))
        sat_time+= control_cycle_ns;
    n= len(truets)
    truets=np.array(truets)
    truers=np.array(truers)
    truevs=np.array(truevs)
    gpsts=np.array(gpsts)
    gpsrs=np.array(gpsrs)
    gpsvs=np.array(gpsvs)
    #save everything in the hdf5 file
    #turn into a np structured array
    sensorsdtype= np.dtype([('t', 'i8'), ('r', 'f8',3), ('v', 'f8',3)])
    truthdtype= np.dtype([('t', 'i8'), ('r', 'f8',3), ('v', 'f8',3)])
    sensors= np.empty(n,dtype=sensorsdtype)
    truth= np.empty(n,dtype=truthdtype)
    truth['t']=truets
    truth['r']=truers
    truth['v']=truevs
    sensors['t']=gpsts
    sensors['r']=gpsrs
    sensors['v']=gpsvs
    with h5file as f:
        f.attrs['git info']= getgitinfo()
        f.attrs['inputs']= inputstr
        f.attrs['script file']= __file__
        f.attrs['command line args']= str(sys.argv)
        f.attrs['local time made']= time.asctime()
        truth= f.create_dataset('truth',data=truth,compression='gzip')
        truth.attrs['docs']='GRACE FO data in ECEF and GPS time (ns, m, m/s)'
        sensors= f.create_dataset('sensors',data=sensors,compression='gzip')
        sensors.attrs['docs']='intermitnent GRACE FO data in ECEF and GPS time with noise (ns, m, m/s)'

if __name__ == "__main__":
    main(h5py.File("Gracetest.hdf5","w"),None,'../../MATLAB/test/GNV1B_2019-10-02_D_04.txt',120_000_000,1.0,1.1,1.2,1.3)