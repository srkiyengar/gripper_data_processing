__author__ = 'srkiyengar'




import numpy as np
import transform as tp
from datetime import datetime, timedelta
from scipy.interpolate import UnivariateSpline
import matplotlib.pyplot as plt
import plotting
'''
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
'''
def plot_frame_axes(ax,P):



    Uxx,Uyx,Uzx,x,Uxy,Uyy,Uzy,y,Uxz,Uyz,Uzz,z,_,_,_,_ = P
    Ux = [Uxx, Uxy, Uxz]
    Uy = [Uyx, Uyy, Uyz]
    Uz = [Uzx, Uzy, Uzz]
    O = [x,y,z]
    print "{}, -- {}, {}, {}".format(O,Ux,Uy,Uz)

    m = 10
    plotting.plot_unit_vector(ax,m,O,Ux,Uy,Uz)
    plt.pause(0.2)



if __name__ == '__main__':

    #v1 = [97.663788, -180.389755, -1895.446655]
    #q1 = [0.416817, -0.806037, 0.028007, -0.419267]
    #v2 = [78.019791, -26.525036, -1980.021118]
    #q2 = [0.222542, 0.551251, 0.281243, 0.753326]

    labview_fname = "1684-2017-05-23-at-20-42-30 .txt"
    gripper_fname = "Servo-displacement-2017-05-23 20:42-1684"


    v1 = [74.179947, -40.639923, -1818.838379]
    q1 = [0.307065, -0.268301, -0.865436, 0.291115]
    v2 = [69.082581, -192.611542, -1907.999756]
    q2 = [0.546920, 0.229265, -0.742266, -0.312023]



    H = tp.static_transform_449_top(q1,v1,q2,v2)

    print H

    '''
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    left,right = ax.set_xlim3d(-100, 100)
    print "x"
    print left, right

    left,right = ax.set_ylim3d(-350,-150)
    print "y"
    print left, right

    left,right = ax.set_zlim3d(-2010, -1990)
    print "z"
    print left, right
    '''

    with open(labview_fname) as f:
        lines = f.readlines()

    r00,r01,r02,r03,r10,r11,r12,r13,r20,r21,r22,r23 = ([] for i in range(12))
    cTime = []
    rTime = []
    hMatrix=[]

    for line in lines[6:]:
        time_str = line[12:38]
        dateSetting = '%Y-%m-%d-%H-%M-%S.%f'
        clockDiff = 1854131
        clockDiff = timedelta(microseconds=int(clockDiff))
        labviewTime = datetime.strptime(time_str, dateSetting)
        adj_labviewTime = labviewTime-clockDiff
        cTime.append(adj_labviewTime)
        rTime.append((adj_labviewTime-cTime[0]).total_seconds())

        vq_str = line[51:]
        vq_str = vq_str[:vq_str.rfind("*")-4]
        x,y,z,qr,qi,qj,qk = map(float,vq_str.split(","))

        q = [qr,qi,qj,qk]                           #Quaternions representing rotation of the top tool frame wrt to NDI
        v = [x,y,z]                                 #Vector position of the top tool with respect to (wrt) NDI frame
        M = tp.rotation_matrix_from_quaternions(q)  #Rotation Matrix from
        N = tp.homogenous_transform(M,v)            #H represents the rotation and translation to move to gripper center
        P = (N.dot(H)).flatten()                    #N is the quat and vect of the Gripper center with respect to NDI

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

        #plot_frame_axes(ax,P)


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


        #Ux,Uy,Uz,V = tp.extract_unit_q_and_vector(P)# unit vectors of the frame and its origin
        #plt.plot(cTime,hMatrix[k],"ro",cTime,Spl[k],"b")
        #lx,ly,lz = tp.vector_length(Ux,Uy,Uz)
        #print "{},{},{},{} --- lengths {}---{}---{}".format(Ux,Uy,Uz,V,lx,ly,lz)

    #plt.subplot(211)
    #plt.plot(rTime,hMatrix[3],"r", rTime,hMatrix[7],"b", rTime,hMatrix[11],"g")

    #plt.show()

    with open(gripper_fname) as f:
        lines = f.readlines()

    goal_ts = []
    goal_position_f1 = []
    goal_position_f2 = []
    goal_position_f3 = []
    goal_position_f4 = []
    joy_ts = []
    joy_displacement = []
    current_position_f1 = []
    current_position_f2 = []
    current_position_f3 = []
    current_position_f4 = []
    current_position_ts =[]

    #2017-05-09 20:22:12.289170,0.00518798828125,0.0,2017-05-09 20:22:12.291152,15369, 14444, 17101, 12703,**2017-05-09 20:22:12.314791,26955,0, 0, 0, 12697

    for line in lines[2:]:

        dateSetting = '%Y-%m-%d %H:%M:%S.%f'

        joy_time_str,rest = line.split(",",1)
        joy_time = datetime.strptime(joy_time_str, dateSetting)

        y_displacement,rest = rest.split(",",1)
        x_displacement,rest = rest.split(",",1)

        gp_time_str,rest = rest.split(",",1)
        gp_time = datetime.strptime(gp_time_str, dateSetting)

        a,b,c,d,rest = rest.split(",",4)
        gp_f1 = int(a)
        gp_f2 = int(b)
        gp_f3 = int(c)
        gp_f4 = int(d)


        cp_time_str,rest = rest.split(",",1)
        cp_time = datetime.strptime(cp_time_str[2:], dateSetting)

        _,e,f,g,h = rest.split(",",4)
        cp_f1 = int(e)
        cp_f2 = int(f)
        cp_f3 = int(g)
        cp_f4 = int(h)

        if ((cp_f1 == 0) or (cp_f2 == 0) or (cp_f3 == 0) or (cp_f4 == 0)):
            pass
        else:
            joy_ts.append(joy_time)
            joy_displacement.append([x_displacement,y_displacement])
            goal_ts.append(gp_time)
            goal_position_f1.append(gp_f1)
            goal_position_f2.append(gp_f2)
            goal_position_f3.append(gp_f3)
            goal_position_f4.append(gp_f4)
            current_position_ts.append(cp_time)
            current_position_f1.append(cp_f1)
            current_position_f2.append(cp_f2)
            current_position_f3.append(cp_f3)
            current_position_f4.append(cp_f4)


    rcp = []
    spl_value = [[] for i in range(12)]

    start = current_position_ts[0]
    f_pos = []
    s_fpos = current_position_f1[0]
    j = 0
    for cpt in current_position_ts:
        diff = (cpt-start).total_seconds()
        rcp.append(diff)
        f_pos.append(current_position_f1[j]-s_fpos)
        j += 1

        for i in range(0,12,1):
            g = Spl[i](diff)
            spl_value[i].append(g)


    #plt.subplot(311)
    #plt.plot(rcp,current_position_f1,"r", rcp,current_position_f2,"b", rcp,current_position_f3,"g")

    #plt.subplot(312)
    #plt.plot(rcp,goal_position_f1,"r", rcp,goal_position_f2,"b", rcp,goal_position_f3,"g")

    x1,y1,z1,t_time = [],[],[],[]
    f1 = []
    for i in range(30):
        x1.append(float(spl_value[3][i]))
        y1.append(float(spl_value[7][i]))
        z1.append(float(spl_value[11][i]))
        f1.append(current_position_f1[i])
        t_time.append(rcp[i])

    '''
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    left,right = ax.set_xlim3d(-200, 0)
    print "x"
    print left, right

    left,right = ax.set_ylim3d(1984,1996)
    print "y"
    print left, right

    left,right = ax.set_zlim3d(320,197)
    print "z"
    print left, right
    ax.plot(k, l, m, color="r")
    '''
    fig = plt.figure()
    fig.suptitle('Gripper location (mm) vs time (seconds)')

    a1 = fig.add_subplot(411)
    #a1.title.set_text('x')
    a1.set_ylabel("x-coordinate")
    a1.plot(t_time, x1, "r")

    a2 = fig.add_subplot(412)
    #a2.title.set_text('y')
    a2.set_ylabel("y-coordinate")

    a2.plot(t_time, y1,"b")

    a3 = fig.add_subplot(413)
    #a3.title.set_text('z')
    a3.set_ylabel("z-coordinate")
    a3.plot(t_time, z1,"g")

    a4 = fig.add_subplot(414)
    a4.set_ylabel("Finger 1")
    a4.plot(t_time,f1,"m")
    fig.subplots_adjust(hspace=1)
    plt.show()
    pass


