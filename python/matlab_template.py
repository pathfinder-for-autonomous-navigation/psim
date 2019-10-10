import matplotlib.pyplot as plt
import matlab.engine

#
# This file is currently out of date and was simply used as a proof of concept.
# It will be brought up to speed with the rest of the sim once sensor models are
# created.
#

def main():

    eng = matlab.engine.start_matlab()
    eng.addpath('MATLAB', nargout=0)
    eng.addpath('MATLAB/utl', nargout=0)

    eng.config(nargout=0)

    t = []
    r = ([], [], [])
    for i in range(0, 4 * 90):

        _r = eng.workspace['truth']['r']
        r[0].append(_r[0][0])
        r[1].append(_r[1][0])
        r[2].append(_r[2][0])
        _t = eng.workspace['truth']['mission_time__ns']
        t.append(float(_t) * 1e-9)

        print('Progress at {i}/360')

        for _ in range(0, 10 * 60):  # Minute between data points
            eng.truth_update(nargout=0)

    eng.quit()

    plt.figure()
    plt.plot(r[0], r[1])
    plt.show()

if __name__ == "__main__":
    main()
