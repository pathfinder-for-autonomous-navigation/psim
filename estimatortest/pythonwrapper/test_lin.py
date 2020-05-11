import cppimport
import numpy as np
import copy

def test_lin():
    from pythonwrapper import pwrap
    testvectors()
    testmatix()

def testvectors():
    # lin_Vectors can be made from regular lists
    x= pwrap.lin_Vector3f([1.0,2.0,1.0E-9])
    x= pwrap.lin_Vector3f([1,2.0,1.0E-9])
    x= pwrap.lin_Vector3d([1.0,2.0,1.0E-9])
    x= pwrap.lin_Vector3d([1,2.0,1.0E-9])

    # lin_Vectors can be made from numpy arrays, types are auto casted to f4 or f8
    x= pwrap.lin_Vector3f(np.array([1.0,2.0,1.0E-9]))
    x= pwrap.lin_Vector3d(np.array([1.0,2.0,1.0E-9]))
    x= pwrap.lin_Vector3d(np.array([1,2,3]))

    # lin_Vectors can be made from other lin_Vectors types are auto casted to f4 or f8
    x= pwrap.lin_Vector3f(pwrap.lin_Vector3f(np.array([1.0,2.0,1.0E-9])))
    x= pwrap.lin_Vector3f(pwrap.lin_Vector3d(np.array([1.0,2.0,1.0E-9])))
    x= pwrap.lin_Vector3d(pwrap.lin_Vector3f(np.array([1,2,3])))
    x= pwrap.lin_Vector3d(pwrap.lin_Vector3d(np.array([1,2,3])))

    # lin_Vectors can be cast back into numpy arrays
    a= np.array([1.0,2.0,1.0E-9],dtype='f4')
    x= np.array(pwrap.lin_Vector3f(a))
    assert np.all(a==x)

    a= np.array([1.0,2.0,1.0E-9],dtype='f8')
    x= np.array(pwrap.lin_Vector3d(a))
    assert np.all(a==x)

    # lin_Vectors can be cast into numpy arrays without copying
    # changing elements in the numpy array will also change the lin_Vector
    a= pwrap.lin_Vector3f(np.array([1.0,2.0,1.0E-9],dtype='f4'))
    x= np.array(a,copy=False)
    x[0]=5
    assert np.all(np.array(a)==x)

    a= pwrap.lin_Vector3d(np.array([1.0,2.0,1.0E-9],dtype='f8'))
    x= np.array(a,copy=False)
    x[0]=5
    assert np.all(np.array(a)==x)

    # lin_Vectors are references
    a= pwrap.lin_Vector3f(np.array([1.0,2.0,1.0E-9],dtype='f4'))
    b= a #b is just a reference to a, not a copy
    x= np.array(a,copy=False)
    x[0]=5 #changes to x also change a and b
    assert np.all(np.array(a)==x)
    assert np.all(np.array(b)==x)

    # lin_Vectors can be deep copied
    a= pwrap.lin_Vector3f(np.array([1.0,2.0,1.0E-9],dtype='f4'))
    b= copy.deepcopy(a) #b is an independent copy of a
    x= np.array(a,copy=False)
    x[0]=5
    assert np.all(np.array(a)==x)
    assert np.all(np.array(b)==np.array([1.0,2.0,1.0E-9],dtype='f4'))

    a= pwrap.lin_Vector3f(np.array([1.0,2.0,1.0E-9],dtype='f4'))
    b= pwrap.lin_Vector3f(a) #b is an independent copy of a
    x= np.array(a,copy=False)
    x[0]=5
    assert np.all(np.array(a)==x)
    assert np.all(np.array(b)==np.array([1.0,2.0,1.0E-9],dtype='f4'))

def testmatix():
    # lin_Matix can be made from regular lists
    x= pwrap.lin_Matrix2x2f([[1,2],[3,4]])
    x= pwrap.lin_Matrix2x2d([[1,2],[3,4]])

    # lin_Matix can be made from numpy arrays, types are auto casted to f4 or f8
    x= pwrap.lin_Matrix2x2f(np.array([[1,2],[3,4]]))
    x= pwrap.lin_Matrix2x2d(np.array([[1,2],[3,4]]))

    # lin_Matix can be made from other lin_Matix types are auto casted to f4 or f8
    x= pwrap.lin_Matrix2x2f(np.array([[1,2],[3,4]]))
    x= pwrap.lin_Matrix2x2d(np.array([[1,2],[3,4]]))

    # lin_Matix can be cast back into numpy arrays
    a= np.array([[1,2],[3,4]],dtype='f4')
    x= np.array(pwrap.lin_Matrix2x2f(a))
    assert np.all(a==x)

    a= np.array([[1,2],[3,4]],dtype='f8')
    x= np.array(pwrap.lin_Matrix2x2d(a))
    assert np.all(a==x)

    # lin_Matix can be cast into numpy arrays without copying
    # changing elements in the numpy array will also change the lin_Vector
    a= pwrap.lin_Matrix2x2f(np.array([[1,2],[3,4]]))
    x= np.array(a,copy=False)
    x[0,0]=5
    assert np.all(np.array(a)==x)

    a= pwrap.lin_Matrix2x2d(np.array([[1,2],[3,4]]))
    x= np.array(a,copy=False)
    x[0,0]=5
    assert np.all(np.array(a)==x)

    # lin_Matix are references
    a= pwrap.lin_Matrix2x2d(np.array([[1,2],[3,4]]))
    b= a #b is just a reference to a, not a copy
    x= np.array(a,copy=False)
    x[0,0]=5 #changes to x also change a and b
    assert np.all(np.array(a)==x)
    assert np.all(np.array(b)==x)

    # lin_Matix can be deep copied
    a= pwrap.lin_Matrix2x2d(np.array([[1,2],[3,4]]))
    b= copy.deepcopy(a) #b is an independent copy of a
    x= np.array(a,copy=False)
    x[0,0]=5
    assert np.all(np.array(a)==x)
    assert np.all(np.array(b)==np.array([[1,2],[3,4]],dtype='f8'))

    a= pwrap.lin_Matrix2x2d(np.array([[1,2],[3,4]]))
    b= pwrap.lin_Matrix2x2d(a) #b is an independent copy of a
    x= np.array(a,copy=False)
    x[0,0]=5
    assert np.all(np.array(a)==x)
    assert np.all(np.array(b)==np.array([[1,2],[3,4]],dtype='f8'))




if __name__ == "__main__":
    pwrap= cppimport.imp("pwrap")
    testvectors()
    testmatix()

