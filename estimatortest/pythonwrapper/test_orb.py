import cppimport
import numpy as np
import copy

if __name__ != "__main__":
    from pythonwrapper import pwrap
    #This is because pytest is weird with setting the path

def test_orb():
    #Create invalid Orbits
    x= pwrap.orb_Orbit()
    assert x.valid()==False

    #Create valid Orbits
    earth_rate_ecef= pwrap.lin_Vector3d([0.000000707063506E-4,-0.000001060595259E-4,0.729211585530000E-4])
    t0= 2045*7*24*60*60*1000*1000*1000
    r0= pwrap.lin_Vector3d([-6522019.833240811, 2067829.846415895, 776905.9724453629])
    v0= pwrap.lin_Vector3d([941.0211143841228, 85.66662333729801, 7552.870253470936])
    y= pwrap.orb_Orbit(t0,r0,v0)
    assert y.valid()==True

    #Getters
    assert y.nsgpstime()==t0
    assert x.nsgpstime()==0
    assert np.all(np.array(y.recef())==np.array(r0))
    assert np.all(np.array(y.vecef())==np.array(v0))
    assert np.all(np.isnan(np.array(x.recef())))
    assert np.all(np.isnan(np.array(x.vecef())))

    #applydeltav
    y.applydeltav(pwrap.lin_Vector3d([1, 0, 0]))
    assert np.all(np.array(y.vecef())==np.array(v0)+np.array([1, 0, 0],dtype='f8'))

    #specificenergy
    e= y.specificenergy(earth_rate_ecef)
    assert type(e) is float

    #shortupdate returns a tuple of specific energy and the jacobian.
    e1,jac = y.shortupdate(1000,earth_rate_ecef)
    assert np.abs(e-e1) < 1E-5
    assert type(e1) is float
    assert type(jac) is pwrap.lin_Matrix6x6d

    #update is used for longer updates
    y.update(t0+1000_000_000_000,earth_rate_ecef)
    assert y.valid()

    #calc_geograv is a static member returns a tuple of acceleration and potential
    g,p= pwrap.orb_Orbit.calc_geograv(r0)

    #orb_Orbit is deepcopy able
    y= pwrap.orb_Orbit(t0,r0,v0)
    z= copy.deepcopy(y)
    y.update(t0+1_000_000_000, earth_rate_ecef)
    assert z.valid()
    assert z.nsgpstime()==t0
    assert np.all(np.array(z.recef())==np.array(r0))
    assert np.all(np.array(z.vecef())==np.array(v0))


if __name__ == "__main__":
    pwrap= cppimport.imp("pwrap")
    test_orb()

