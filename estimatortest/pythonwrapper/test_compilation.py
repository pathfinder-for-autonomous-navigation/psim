import cppimport

def test_comp():
    from pythonwrapper import pwrap
    print(pwrap.orb_Orbit())

if __name__ == "__main__":
    cppimport.set_quiet(False)
    cppimport.force_rebuild(True)
    pwrap= cppimport.imp("pwrap")
    print(pwrap.orb_Orbit())
