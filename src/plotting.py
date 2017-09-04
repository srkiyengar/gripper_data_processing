__author__ = 'srkiyengar'
"""
A simple example of an animated plot
"""
import numpy as np
import matplotlib.pyplot as plt
import transform as tf


from mpl_toolkits.mplot3d import Axes3D



def plot_unit_vector(ax,m,Origin,UvectorX,UvectorY,UvectorZ):

    m1 = m/2
    m2 = 5*m
    m3 = m

    #d1,d2,d3 = tf.vector_length(UvectorX,UvectorY,UvectorZ)
    #print d1,d2,d3

    #A = np.cross(UvectorX,UvectorY)
    #B = np.cross(UvectorY,UvectorZ)
    #C = np.cross(UvectorZ,UvectorX)

    Ux = [m1*e for e in UvectorX]
    Uy = [m2*e for e in UvectorY]
    Uz = [m3*e for e in UvectorZ]

    #d1,d2,d3 = tf.vector_length(Ux,Uy,Uz)
    #print "scaled"
    #print d1,d2,d3

    Uxo = [a + b for a, b in zip(Ux, Origin)]
    Uyo = [a + b for a, b in zip(Uy, Origin)]
    Uzo = [a + b for a, b in zip(Uz, Origin)]

    x = [Origin[0],Uxo[0]]
    y = [Origin[1],Uxo[1]]
    z = [Origin[2],Uxo[2]]
    ax.plot(x, y, z, color="r")

    x = [Origin[0],Uyo[0]]
    y = [Origin[1],Uyo[1]]
    z = [Origin[2],Uyo[2]]
    ax.plot(x, y, z, color="b")

    x = [Origin[0],Uzo[0]]
    y = [Origin[1],Uzo[1]]
    z = [Origin[2],Uzo[2]]
    ax.plot(x, y, z, color="g")



if __name__ == '__main__':


    fig = plt.figure()
    ax = fig.gca(projection='3d')

    left,right = ax.set_xlim3d(-250, 100)
    print "x"
    print left, right

    ax.set_ylim3d(-250,100)
    print "y"
    print left, right

    ax.set_zlim3d(-2000,0)
    print "z"
    print left, right


    for i in np.arange(0,10,0.9):
        l = [0+i,0+i,0+i]
        m = [l[0]+1,l[1]+0,l[2]+0]
        n = [l[0]+0,l[1]+1,l[2]+0]
        o = [l[0]+0,l[1]+0,l[2]+1]

        plot_unit_vector(ax,l,m,n,o)
        plt.pause(0.5)


    pass


