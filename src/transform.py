__author__ = 'srkiyengar'


# homogenous transformation to rotate and translate axes, points, and vector
# inverse will do the rotation and translation in the opposite direction

# Rigid body tools 449 and 339 (NDI polaris) are attached to the modified white gripper shell while taking measurement
# When holding the gripper with the modified shell, the tool closer to the open end of the handle (bottom tool) is 339
# the top tool is 449 and is closer to the 2 fingers

# The center of the gripper is obtained by covering the shell and placing a metal plate, aligning the tool
# such that the one of the axis is aligned to travel in between the two gripper fingers (opposed to the single finger)

# The axis for the gripper (as a rigid body) is fixed (arbitarily) as follows:
# The axis coming out of the gripper towards the object it wants to grip is the +z axis.
# +ive y-axis axis runs between and parallel to the two fingers.

# The static transformation matrix that takes any measurement obtained at the tool position to the center of the gripper
# A homogenous transformation takes from front tool to center expresses. If we get a measurement of the front with
# respect to the ndi frame and multiply with this static transformation matrix, we will measurement for the center with
# respect to NDI. Rnf*Rfc = Rnc ; Rnf to be read as Rotation with respect to reference axes ndi to the axes in the front.
# static transformation:
# Keeping 449 at its position and 339 at the center, a static transformation matrix is calculated
# Keeping 339 at its position and 449 at the center a static transformation matrix is calculated


import numpy as np
import math

def rotation_matrix_from_quaternions(q_vector):

    '''
    :param q_vector: array, containing 4 values representing a unit quaternion that encodes rotation about a frame
    :return: an array of shape 3x3 containing the rotation matrix.
    Takes in array as [qr, qx, qy, qz]
    https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation, s = 1
    '''

    qr, qi, qj, qk = q_vector
    first = [1-2*(qj*qj+qk*qk), 2*(qi*qj-qk*qr),   2*(qi*qk+qj*qr)]
    second= [2*(qi*qj+qk*qr),   1-2*(qi*qi+qk*qk), 2*(qj*qk-qi*qr)]
    third = [2*(qi*qk-qj*qr),   2*(qj*qk+qi*qr),   1-2*(qi*qi+qj*qj)]
    R = np.array([first,second,third])

    return R


def homogenous_transform(R,vect):

    '''
    :param R: 3x3 matrix
    :param vect: list x,y,z
    :return:Homogenous transformation 4x4 matrix using R and vect
    '''

    H = np.zeros((4,4))
    H[0:3,0:3] = R
    frame_displacement = vect + [1]
    D = np.array(frame_displacement)
    D.shape = (1,4)
    H[:,3] = D

    return H

def inverse_homogenous_transform(H):

    '''
    :param H: Homogenous Transform Matrix
    :return: Inverse Homegenous Transform Matrix
    '''


    R = H[0:3,0:3]
    origin = H[:-1,3]
    origin.shape = (3,1)

    R = R.T
    origin = -R.dot(origin)
    return homogenous_transform(R,list(origin.flatten()))

def extract_unit_q_and_vector(H):
    '''

    :param H:Homogenous transform matrix
    :return:the 3 unit vectors and the origin from flattened list [r00,r01,r02,x,r10,r11,r12,y,r20,r21,r22,z
    (r00,r10,r20) is the x-axis unit vector, (r01,r11,r21) is y-axis and (r02,r12,r22) is z-axis (r03,r13,r23) is origin
    '''
    N = list(H.flatten())
    UVx = [N[0],N[4],N[8]]
    UVy = [N[1],N[5],N[9]]
    UVz = [N[2],N[6],N[10]]
    origin = [N[3],N[7],N[11]]

    return UVx, UVy, UVz, origin

def vector_length(X1,Y1,Z1):
    '''

    :param X1: unit vector along x
    :param Y1: unit vector along y
    :param Z1: unit vector along z
    :return:
    '''
    X1_length = math.sqrt(X1[0]*X1[0]+X1[1]*X1[1]+X1[2]*X1[2])
    Y1_length = math.sqrt(Y1[0]*Y1[0]+Y1[1]*Y1[1]+Y1[2]*Y1[2])
    Z1_length = math.sqrt(Z1[0]*Z1[0]+Z1[1]*Z1[1]+Z1[2]*Z1[2])

    return X1_length,Y1_length,Z1_length

def center_tool_339_to_gripper_frame():

    '''
    The y-axis of 339 is aligned with the y axis of the gripper. The z-axis of the 339 will require a rotation of 90
    (counter clockwise with respect to z R (z,90) to get align gripper z axis to outward pointing. the origin of the
    339 needs to be moved in z-axis by + 40.45mm to get it to the origin of the gripper

    :return: homogenous transformation from 339 center to gripper center
    '''

    d =[0.0,0.0,40.45,1.0]
    H = np.zeros((4,4))
    H.shape = (4,4)
    H[:,3]= d
    H[(1,0),(1,2)]=1
    H[2,0]= -1

    return H

def static_transform_449_top(q1,v1,q2,v2):
    '''

    :param q1: unit quaternions representing the rotation of the frame of 449 tool at the top
    :param v1: vector representing the rotation of the frame of 449 tool at the top
    :param q2: unit quaternions representing the rotation of the frame of 339 tool at the center
    :param v2: vector representing the rotation of the frame of 339 tool at the center
    :return: homogenous tranformation
    '''
    # H1 -  Homogenous transform from reference NDI frame to front tool
    # H2 -  Homogenous transform from reference NDI frame to center tool
    # H3 -  Homogenous transformation from the center tool frame to center of the gripper with axis rotated where the y
    # is parallel and between the two fingers and z is pointing outward


    R1 = rotation_matrix_from_quaternions(q1)
    H1 = homogenous_transform(R1, v1)
    h1 = inverse_homogenous_transform(H1)

    R2 = rotation_matrix_from_quaternions(q2)
    H2 = homogenous_transform(R2, v2)

    H3 = center_tool_339_to_gripper_frame()
    H = (h1.dot(H2)).dot(H3)
    return H



if __name__ == '__main__':

    R1 = np.zeros((3,3))
    R1[(2,1),(0,1)] = 1
    R1[0,2]=-1
    print "R1 = "
    print R1
    H1 = homogenous_transform(R1, [0, -10, 0])
    print "H1 = "
    print H1
    h1 = inverse_homogenous_transform(H1)
    print "h1 = "
    print h1

    R2 = np.zeros((3,3))
    R2[(1,2),(0,2)] = 1
    R2[0,1]=-1

    print "R2 ="
    print R2

    H2 = homogenous_transform(R2, [0, -10, -6])
    print "H2 ="
    print H2


    K = h1.dot(H2)
    print "K ="
    print K




    K1 = np.zeros((3,3))
    K1[(0,1,2),(0,1,2)] = 1
    print "K1 ="
    print K1

    k1 = homogenous_transform(K1, [0, 2, 0])
    print "k1 ="
    print k1

    K2 = K.dot(k1)
    print "K2 ="
    print K2

    H1 = homogenous_transform(R1, [0, -11, 0])



    J = H1.dot(K2)
    print J
    J2 = list(J.flatten())[:-4]
    print J2
    pass


