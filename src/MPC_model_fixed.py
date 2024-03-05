#!../venvs/robot_env/bin/python3.7
from timeit import default_timer as timer
import rospy
import numpy as np
import scipy.optimize as opt
import numpy.matlib
import math
import random
from std_msgs.msg import Int32MultiArray, Int16
import os

## B matrix
def shaping(temp, B, ns):
    for k in range(0, ns):
        brow = B[k,:]
        brow = np.delete(brow, k)
        temp[k,:] = brow
    return temp

def Bmatrix(nrows, ncols):
    ns = nrows * ncols
    B = np.zeros([ns, ns], dtype=int)
    Bnew = np.zeros([ns, ns - 1], dtype=int)
    Bin = np.zeros([ns, ns * (ns - 1)], dtype=int)
    Bout = np.zeros([ns, ns * (ns - 1)], dtype=int)
    Neigh = np.zeros([ns, ns], dtype=int)

    for i in range(0, nrows):
        for j in range(0, ncols):
            if i == 0:
                if j == 0:
                    B[0][1] = 1
                    B[0][ncols] = 1
                    B[0][ncols + 1] = 1
                elif j == (ncols - 1):
                    s = (i) * (ncols - 1) + j
                    B[s][s - 1] = 1
                    B[s][2 * s] = 1
                    B[s][2 * s + 1] = 1
                else:
                    s = (i) * (ncols - 1) + j
                    B[s][s - 1] = 1
                    B[s][s + 1] = 1
                    B[s][s + ncols - 1] = 1
                    B[s][s + ncols] = 1
                    B[s][s + ncols + 1] = 1

            elif i == (nrows - 1):
                if j == 0:
                    s = i * ncols + j
                    sminus = i * (ncols - 1)
                    B[s][s + 1] = 1
                    B[s][sminus] = 1
                    B[s][sminus - 1] = 1
                elif j == (ncols - 1):
                    s = i * ncols + j
                    sminus = i * (ncols - 1)
                    B[s][s - 1] = 1
                    B[s][sminus + j - 1] = 1
                    B[s][sminus + j - 2] = 1
                else:
                    s = i * ncols + j
                    sminus = i * (ncols - 1)
                    B[s][s - 1] = 1
                    B[s][s + 1] = 1
                    B[s][sminus + j - 1] = 1
                    B[s][sminus + j - 2] = 1
                    B[s][sminus + j] = 1

            else:
                s = i * ncols + j
                sminus = (i - 1) * ncols
                splus = (i + 1) * ncols
                if j == 0:
                    B[s][sminus] = 1
                    B[s][sminus + 1] = 1
                    B[s][splus] = 1
                    B[s][splus + 1] = 1
                    B[s][s + 1] = 1
                elif j == (ncols - 1):
                    B[s][s - 1] = 1
                    B[s][sminus + j - 1] = 1
                    B[s][sminus + j] = 1
                    B[s][splus + j - 1] = 1
                    B[s][splus + j] = 1
                else:
                    B[s][s + 1] = 1
                    B[s][s - 1] = 1
                    B[s][sminus + j + 1] = 1
                    B[s][sminus + j - 1] = 1
                    B[s][sminus + j] = 1
                    B[s][splus + j + 1] = 1
                    B[s][splus + j - 1] = 1
                    B[s][splus + j] = 1

    Bnew = shaping(Bnew, B, ns)

    for i in range(0, ns):
        brow = Bnew[i, :]
        ind_st = i * (ns - 1)
        ind_end = ind_st + ns - 1
        Bin[i, ind_st:ind_end] = brow

    for i in range(0, ns):
        neigh = np.nonzero(Bnew[i,:])

        for elements in neigh:
            elements = elements + 1
            for element in elements:
                if i < element:
                    Neigh[i][element] = 1
                else:
                    Neigh[i][element - 1] = 1

        for neighbors in neigh:
            for neighbor in neighbors:
                neighbor = neighbor + 1
                if i < neighbor:
                    ind = neighbor * (ns - 1)
                    Bout[i][ind + i] = 1
                else:
                    neighbor = neighbor - 1
                    ind = neighbor * (ns - 1)
                    Bout[i][ind + i - 1] = 1

    return Bin, Bout, Neigh
    
def coordinates(resource, nrows, ncols):
    ns = nrows * ncols
    if resource == 0:
        x = 0
        y = 0
    elif resource == (ns - 1):
        x = nrows - 1
        y = ncols - 1
    else:
        if (resource % nrows) == 0:
            x = 0
            y = resource / ncols
        else:
            x = resource % nrows
            y = math.floor(resource / ncols)

    return x, y

def distance(dest_array, neigh_e, nrows, ncols):
    xs, ys = coordinates(neigh_e, nrows, ncols)

    ys = ys + 1
    xs = xs + 1

    sectors = np.nonzero(dest_array)

    len_sec = len(sectors[0])
    dist = np.zeros(len_sec, dtype=int)
    yref = np.zeros(len_sec, dtype=int)
    sectors = sectors[0]

    for sec in range(0, len(sectors)):
        yref[sec] = math.ceil((sectors[sec])/ncols)
        xref = sectors[sec] - (yref[sec] - 1) * nrows
        pow1 = (ys - yref[sec]) ** 2
        pow2 = (xs - xref) ** 2
        sq = math.sqrt(pow1 + pow2)
        dist[sec] = math.floor(sq)

    b = np.argmin(dist)
    out = dist[b]
    return out, b

def pdfgen(pmf):
    N = len(pmf)
    F = np.zeros((N))
    F[0] = pmf[0]
    for k in range(1,N):
        F[k] = F[k - 1] + pmf[k]
    b = random.random()
    ind = np.asarray(F >= b).nonzero()
    out = ind[0]
    return out

def sign(t1):
    if t1 > 0:
        return 1
    elif t1 < 0:
        return -1
    elif t1 == 0:
        return 0
    

def Diffusionmatrix(x_source, x_dest, nrows, ncols, neigh, tau_diff):

    ns = nrows * ncols
    G_diff = np.zeros((ns * (ns - 1), ns))
    loc_source = np.nonzero(x_source)
    loc_source = loc_source[0]

    for sector in loc_source:
        neigh_row = neigh[sector, :]
        neigh_s = np.asarray(neigh_row == 1).nonzero()
        neigh_s = neigh_s[0]
        dist = np.zeros(len(neigh_s))
        for n in range(len(neigh_s)):
            dist_cal = distance(x_dest, neigh_s[n], nrows, ncols)
            dist[n] = dist_cal[0]

        index = np.argmin(dist)
        output = dist[index]
        tau = tau_diff
        term1 = -(dist - output) / tau
        prob = np.divide(np.exp(term1), (sum(np.exp(term1))))

        minimum = np.zeros(len(neigh_s), dtype=int)
        for m in range(len(neigh_s)):
            minimum[m] = min(0, sign(neigh_s[m] - sector))
        
        sec_s_adj = sector + minimum
        ind = ((neigh_s) * (ns - 1)) + sec_s_adj
        for p in range(len(prob)):
            G_diff[ind[p], sector] = prob[p]
    return G_diff

def AssumedModel_enemy(xref, xe, B, Tp, nrows, ncols, neigh, tau_diff_e):
    ns = nrows * ncols
    out2 = np.zeros((ns, Tp))
    xe_prev = np.zeros((ns, 1))
    Ge = np.zeros((ns * (ns - 1), ns))
    xe_prev = xe

    for time2 in range(0, Tp):
        Ge = Diffusionmatrix(xe_prev, xref, nrows, ncols, neigh, tau_diff_e)
        ue = np.dot(Ge, xe_prev)
        add2 = xe_prev + np.dot(B, ue)
        for z2 in range(ns):
            out2[z2][time2] = add2[z2]
        xe_prev = add2
    return out2

def FlowConstraints(Bin, Bout, Tp, ns):
    B = Bin - Bout
    BTp = np.zeros((ns, ns * (ns - 1) * Tp))
    row_eq1 = 2 * Tp * ns
    col_eq1 = Tp * ns + Tp * ns * (ns - 1)
    A = np.zeros((row_eq1, col_eq1))

    for l in range(0, Tp):
        if l == 0:
            BTp = Bout
        else:
            BTp = np.concatenate((-B, BTp), axis=1)

        row_st1 = l * ns + 1
        row_end1 = row_st1 + ns - 1
        col_st1 = Tp * ns + 1
        col_end1 = col_st1 + np.size(BTp[1]) - 1
        A[(row_st1 - 1): row_end1, (col_st1 - 1) : col_end1] = BTp

        row_st1 = row_st1 + Tp * ns
        row_end1 = row_st1 + ns - 1
        col_st1 = l * (ns * (ns - 1)) + 1 + Tp * ns
        col_end1 = col_st1 + ns * (ns - 1) - 1
        A[(row_st1 - 1) : row_end1, (col_st1 - 1) : col_end1] = -Bout

    return A

def DynamicConstraints(Bin, Bout, Tp, ns):
    B = Bin - Bout

    row_eq2 = Tp * ns
    col_eq2 = Tp * ns + Tp * ns * (ns - 1)
    Aeq = np.zeros((row_eq2, col_eq2), dtype=int)

    Aeq[0: Tp * ns, 0 : Tp * ns] = np.identity(Tp * ns, dtype= int)

    for iter1 in range(Tp):
        row_st2 = 0
        row_end2 = 0
        col_st2 = 0
        col_end2 = 0

        BTp = np.zeros((ns, ns * (ns - 1) * (iter1 + 1)))
        BTp = np.tile(B, iter1 + 1)
        row_st2 = iter1 * ns + 1
        row_end2 = row_st2 + ns - 1
        col_st2 = Tp * ns + 1

        col_end2 = col_st2 + len(BTp[0]) - 1
        Aeq[(row_st2 - 1) : row_end2, (col_st2 - 1) : col_end2] = -BTp

    return Aeq


def LP_defenders(xe_assumed, xref, xf, Aeq, A, Tp, nrows, ncols):
    Bin, Bout, neigh = Bmatrix(nrows, ncols)
    loc_xf = np.nonzero(xf)

    for defender in loc_xf:
        n = neigh[defender, :]

    ns = nrows * ncols
    ee1 = np.zeros((ns, Tp))
    ee2 = np.zeros((ns * (ns - 1), Tp))
    Xf = np.zeros((ns, Tp))
    Uf = np.zeros((ns * (ns - 1), Tp))

    Xe = np.zeros((ns * Tp, 1))
    Xe_trans = np.zeros((ns * Tp, 1))
    Xe_trans = np.transpose(xe_assumed)
    Xe1 = Xe_trans.flatten(order='C')

    for xx in range(ns * Tp):
        Xe[xx, 0] = Xe1[xx]

    Xref = np.transpose(numpy.matlib.repmat(np.transpose(xref), 1, Tp))
    
    alphaf = 0.9
    betaf = 1 - alphaf

    f1 = np.transpose(-(alphaf * Xe + betaf * Xref))
    f2 = np.zeros((1, Tp * ns * (ns - 1)))
    f = np.transpose(np.concatenate((f1, f2), axis=1))

    beq = np.transpose(numpy.matlib.repmat(np.transpose(xf), 1, Tp))

    b1 = numpy.matlib.repmat(np.transpose(xf), 1, Tp)
    b2 = np.zeros((1, ns * Tp))
    b = np.concatenate((b1, b2), axis=1)
    b = b[0]

    LB = np.zeros((Tp * (ns + ns * (ns - 1)), 1))
    UB = np.ones((Tp * (ns + ns*(ns - 1)), 1))
    LB = LB[0]
    UB = UB[0]

    out1 = np.zeros((len(f), 1))
    start = timer()
    out1 = opt.linprog(f, A_ub = A, b_ub = b, A_eq = Aeq, b_eq = beq, method="highs", bounds = [0,1], options = {"maxiter": 5000, "tol" : 1.000e-6, "disp" : False})
    end = timer()
    rospy.loginfo("seconds taken to calculate: {}".format(end - start))
    optimize1 = out1.x

    for l in range(Tp):
        ind_st_xf = (l) * ns + 1
        ind_end_xf = ind_st_xf + ns - 1
        ee1[:, l] = optimize1[ind_st_xf - 1: ind_end_xf]

    Xf[:, 0] = ee1[:, 0]

    for l in range(Tp):
        ind_st_uf = (l) * (ns * (ns - 1)) + Tp * ns + 1
        ind_end_uf = ind_st_uf + (ns * (ns - 1)) - 1
        ee2[:, l] = optimize1[ind_st_uf - 1 : ind_end_uf]

    Uf[:, 0] = ee2[:,0]

    out_xf = Xf[:, 0]
    out_uf = Uf[:, 0]
    return out_xf, out_uf

robot_sectors = [13,1,2,3]

def callback(data):
    global robot_sectors
    temp = data.data
    robot_sectors = temp


nrows = 8
ncols = 8
meter_per_sector_length = 0.5

T = 30
Tp = 3

ns = nrows * ncols

nf = 3

ne = 1

alphaf = 0.9; betaf = 1-alphaf

round = 0

[Bin, Bout, Neigh] = Bmatrix(nrows, ncols)
# B = Bin - Bout

# Aeq = DynamicConstraints(Bin, Bout, Tp, ns)

# A = FlowConstraints(Bin, Bout, Tp, ns)

# path = os.path.abspath("")

# B = np.load("{}/../catkin_ws/src/robomaster_interface/src/B.npy".format(path))
# A = np.load("{}/../catkin_ws/src/robomaster_interface/src/A.npy".format(path))
# Aeq = np.load("{}/../catkin_ws/src/robomaster_interface/src/Aeq.npy".format(path))

username = os.environ["USER"]
B = np.load("/home/{}/catkin_ws/src/robomaster_interface/src/B8.npy".format(username))
A = np.load("/home/{}/catkin_ws/src/robomaster_interface/src/A8.npy".format(username))
Aeq = np.load("/home/{}/catkin_ws/src/robomaster_interface/src/Aeq8.npy".format(username))


num_games = 20; num_lost = 0; num_won = 0
num_iterations = np.zeros((1, num_games))
cost = np.zeros((1, num_games))

Aest = np.zeros((num_games, ns))

rospy.init_node('defender')
robot_name = rospy.get_param('~robot_number')
robot_number = int(robot_name[-1])
pub = rospy.Publisher('/{}/goal_sector'.format(robot_name), Int16, queue_size=1)

# prev_sector = 0
while not rospy.is_shutdown():

    for g in range(num_games):

        xf = np.zeros((ns, T + 1))
        Xf = np.zeros((ns, Tp))

        xe = np.zeros((ns, T + 1))
        Xe_model = np.zeros((ns, Tp))

        xref = np.zeros((ns, 1))

        nu = ns * (ns - 1)
        uf = np.zeros((nu, T))
        Uf = np.zeros((nu, Tp))

        xref[0, 0] = 1
        xref[1, 0] = 1
        xref[2, 0] = 1
        xref[3, 0] = 1
        # xref[4, 0] = 1

        t = 1
        tau_diff_e = 0.1

        while t < T:
            rospy.Subscriber("/sectors", Int32MultiArray, callback)

            xe[robot_sectors[0] - 1, t - 1] = 1
            xf[robot_sectors[1] - 1, t - 1] = 1
            xf[robot_sectors[2] - 1, t - 1] = 1
            xf[robot_sectors[3] - 1, t - 1] = 1


            rospy.loginfo("xf[t-1]: {}".format(xf[:, t - 1]))
            xe_assumed = AssumedModel_enemy(xref, xe[:, t - 1], B , Tp, nrows, ncols, Neigh, tau_diff_e)
            _, uf[:, t] = LP_defenders(xe_assumed, xref, xf[:, t - 1], Aeq, A, Tp, nrows, ncols)

            controls = np.nonzero(uf[:, t])
            controls = controls[0]
            next_sector = -1
            rospy.loginfo("B @ uf: {}".format(np.matmul(B,uf[:,t])))

            rospy.loginfo("controls: {}".format(controls))
            # rospy.loginfo("xf: {}".format(xf[:,t]))
            rospy.loginfo("uf: {}".format(uf[:,t]))

            for control in controls:
                next_sector = math.floor(control / (ns - 1)) + 1
                offset = control % (ns - 1)
                if offset >= next_sector:
                    prev_sector = offset + 2
                else:
                    prev_sector = offset + 1

                if prev_sector == robot_sectors[robot_number]:
                    rospy.loginfo("sending to sector : {}".format(next_sector))
                    pub.publish(next_sector)
                    break
            

            t = t + 1