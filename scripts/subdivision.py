
from scipy.interpolate import splprep, splev
from scipy.interpolate import interp1d
import numpy as np
import matplotlib.pyplot as plt



def ChaikinSubdivisionClosedPolygon(P):
    N = len(P)
    print P
    for j in range(7):
        Q = np.zeros(shape=(2 * N,2))
        # print Q

        for i in range(N-1):
    		Q[2*i][0] = 3./4. * P[i][0] + 1./4. * P[i+1][0]
    		# print Q[2*i][0]
    		Q[2*i][1] = 3./4. * P[i][1] + 1./4. * P[i+1][1]
    		Q[2*i+1][0] = 1./4. * P[i][0] + 3./4. * P[i+1][0]
    		Q[2*i+1][1] = 1./4. * P[i][1] + 3./4. * P[i+1][1]
        N = N * 2
        P = Q
        print P
    return Q


def main():
    pts = np.array([[100, 300], [500, 300], [500, 500], [300, 400], [200, 800]])
    # P = np.array([[0, 0], [0, 2], [2, 3], [4, 0], [6, 3], [8, 2], [8, 0]])
    # Q = ChaikinSubdivisionClosedPolygon(P)

    # x, y = pts.T
    # i = np.arange(len(P))
    # # 5x the original number of points
    # interp_i = np.linspace(0, i.max(), 10 * i.max())
    #
    # xi = interp1d(i, x, kind='cubic')(interp_i)
    # yi = interp1d(i, y, kind='cubic')(interp_i)
    #
    # fig, ax = plt.subplots()
    # ax.plot(xi, yi)
    # ax.plot(x, y, 'ko')
    # plt.show()



    tck, u = splprep(pts.T, u=None, s=0.0, per=1)
    u_new = np.linspace(u.min(), u.max(), 1000)
    x_new, y_new = splev(u_new, tck, der=0)

    plt.plot(pts[:,0], pts[:,1], 'ro')
    plt.plot(x_new, y_new, 'b--')
    plt.show()




    # plt.plot(Q[:,0],Q[:,1])
    # plt.plot(P[:,0],P[:,1], color='g')

    # plt.show()
    # print Q

main()
