__author__ = 'srkiyengar'




import numpy as np
import transform as tp

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

    v1 = [97.663788, -180.389755, -1895.446655]
    q1 = [0.416817, -0.806037, 0.028007, -0.419267]
    v2 = [78.019791, -26.525036, -1980.021118]
    q2 = [0.222542, 0.551251, 0.281243, 0.753326]

    H = tp.static_transform_449_top(q1,v1,q2,v2)
    print H
