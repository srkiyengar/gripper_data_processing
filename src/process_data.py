__author__ = 'srkiyengar'




import numpy as np
import transform as tp
from datetime import datetime, timedelta
from scipy.interpolate import UnivariateSpline
import matplotlib.pyplot as plt

def extVec(vec):
    """
    Takes in an array as [x, y, z]
    """
    vec = [_ for _ in vec] + [1]
    vec = np.array(vec)
    vec.shape = (4,1)
    return vec

def unExtVec(extVec):
    return extVec[:-1]

def rotMatFromQuat(quat):
    """
    Takes in array as [qr, qx, qy, qz]
    https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    """
    qr, qx, qy, qz = quat
    first = [1-2*qy*qy-2*qz*qz, 2*(qx*qy-qz*qr),   2*(qx*qz+qy*qr)]
    second= [2*(qx*qy+qz*qr),   1-2*qx*qx-2*qz*qz, 2*(qy*qz-qx*qr)]
    third = [2*(qx*qz-qy*qr),   2*(qy*qz+qx*qr),   1-2*qx*qx-2*qy*qy]
    R = np.array([first,second,third])
    return R

def homogenousTransformationMatrix(rotMat, origin):
    H = np.zeros((4,4))
    H[0:3,0:3] = rotMat
    H[:,3] = extVec(origin).flatten()
    return H
def inverseHomegenousTransformation(H):
    R = H[0:3,0:3]
    origin = H[:-1,3]
    origin.shape = (3,1)

    R = R.T
    origin = -R.dot(origin)

    return homogenousTransformationMatrix(R, list(origin.flatten()))


if __name__ == '__main__':

    #v1 = [97.663788, -180.389755, -1895.446655]
    #q1 = [0.416817, -0.806037, 0.028007, -0.419267]
    #v2 = [78.019791, -26.525036, -1980.021118]
    #q2 = [0.222542, 0.551251, 0.281243, 0.753326]

    fname = "1197-2017-05-09-at-20-22-18 .txt"


    v1 = [74.179947, -40.639923, -1818.838379]
    q1 = [0.307065, -0.268301, -0.865436, 0.291115]
    v2 = [69.082581, -192.611542, -1907.999756]
    q2 = [0.546920, 0.229265, -0.742266, -0.312023]

    H = tp.static_transform_449_top(q1,v1,q2,v2)

    print H

    with open(fname) as f:
        lines = f.readlines()


    r00,r01,r02,r03,r10,r11,r12,r13,r20,r21,r22,r23 = ([] for i in range(12))
    cTime = []
    rTime = []
    hMatrix=[]


    for line in lines[6:]:
        time_str = line[13:39]
        dateSetting = '%Y-%m-%d-%H-%M-%S.%f'
        clockDiff = 1854131
        clockDiff = timedelta(microseconds=int(clockDiff))
        labviewTime = datetime.strptime(time_str, dateSetting)
        adj_labviewTime = labviewTime-clockDiff
        cTime.append(adj_labviewTime)
        rTime.append((adj_labviewTime-cTime[0]).total_seconds())

        vq_str = line[52:]
        vq_str = vq_str[:vq_str.rfind("*")-4]
        x,y,z,qr,qi,qj,qk = map(float,vq_str.split(","))
        print "{},{},{}, -- {}, {}, {}, {}".format(x,y,z,qr,qi,qj,qk)

        q = [qr,qi,qj,qk]                           #Quaternions representing rotation of the top tool frame wrt to NDI
        v = [x,y,z]                                 #Vector position of the top tool with respect to (wrt) NDI frame
        M = tp.rotation_matrix_from_quaternions(q)  #Rotation Matrix from
        N = tp.homogenous_transform(M,v)            #H represents the rotation and translation to move to gripper center
        P = (H.dot(N)).flatten()                    #N is the quat and vect of the Gripper center with respect to NDI

        r00.append(P[0])
        r01.append(P[1])
        r02.append(P[2])
        r03.append(P[3])
        r10.append(P[4])
        r11.append(P[5])
        r12.append(P[6])
        r13.append(P[7])
        r20.append(P[8])
        r21.append(P[9])
        r22.append(P[10])
        r23.append(P[11])

    hMatrix.append(r00)
    hMatrix.append(r01)
    hMatrix.append(r02)
    hMatrix.append(r03)
    hMatrix.append(r10)
    hMatrix.append(r11)
    hMatrix.append(r12)
    hMatrix.append(r13)
    hMatrix.append(r20)
    hMatrix.append(r21)
    hMatrix.append(r22)
    hMatrix.append(r23)

    Spl = {}

    for k in range(0,12,1):
        Spl[k] = UnivariateSpline(rTime,hMatrix[k])

        if(k == 3 or k== 7 or k == 11):
            pass
        #Ux,Uy,Uz,V = tp.extract_unit_q_and_vector(P)# unit vectors of the frame and its origin
        #plt.plot(cTime,hMatrix[k],"ro",cTime,Spl[k],"b")
        #lx,ly,lz = tp.vector_length(Ux,Uy,Uz)
        #print "{},{},{},{} --- lengths {}---{}---{}".format(Ux,Uy,Uz,V,lx,ly,lz)
        else:
            plt.plot(rTime,hMatrix[k])

    plt.subplot(211)
    plt.plot(rTime,hMatrix[k])

    plt.show()
    pass


