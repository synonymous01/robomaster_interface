#!../venvs/robot_env/bin/python3.7
# import rospy
# import rospy
from timeit import default_timer as timer
import numpy as np
import scipy.optimize as opt
import numpy.matlib
import math
import time
import random
# from std_msgs.msg import Int32MultiArray, Int16
import os

def shaping(temp,B,ns):
    for k in range(0, ns):
        brow = B[k,:]
        brow = np.delete(brow,k)
        temp[k,:] = brow
    return temp

def Bmatrix(nrows, ncols):
    ns = nrows * ncols
    B = np.zeros([ns, ns], dtype = int) 
    Bnew = np.zeros([ns, ns - 1], dtype = int)  
    Bin = np.zeros([ns, ns * (ns - 1)], dtype = int)
    Bout = np.zeros([ns, ns * (ns - 1)], dtype = int) 
    Neigh = np.zeros([ns,ns], dtype = int);

    for i in range(0, nrows):
        for j in range(0, ncols): 
            if (i == 0):                   #First row
                if (j == 0):               #First col
                    B[0][1] = 1
                    B[0][ncols] = 1
                    B[0][ncols + 1] = 1
                elif (j == ncols - 1):         #Last sector, First row
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
            
            elif (i == nrows - 1):             #Last row
                if (j == 0):
                    s = (i) * (ncols) + j
                    sminus = (i) * (ncols - 1)
                    B[s][s + 1] = 1
                    B[s][sminus] = 1
                    B[s][sminus - 1] = 1
                elif (j == ncols - 1):         #Last row, Last col
                    s = (i) * (ncols) + j
                    sminus = (i) * (ncols - 1)
                    B[s][s - 1] = 1
                    B[s][sminus + j - 1] = 1 
                    B[s][sminus + j - 2] = 1
                else:
                    s = (i) * (ncols) + j
                    sminus = (i) * (ncols - 1)
                    B[s][s - 1] = 1
                    B[s][s + 1] = 1
                    B[s][sminus + j - 1] = 1
                    B[s][sminus + j - 2] = 1 
                    B[s][sminus + j] = 1

            else:
                s = (i) * (ncols) + j
                sminus = (i - 1) * (ncols) 
                splus = (i + 1) * (ncols) 
                if (j == 0):
                    B[s][sminus] = 1
                    B[s][sminus + 1] = 1 
                    B[s][splus] = 1
                    B[s][splus + 1] = 1
                    B[s][s + 1] = 1
                elif (j == ncols - 1):
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

    Bnew = shaping(Bnew,B,ns)  #resizing B from (100x100) to (100x99)
    #print("B =")
    #print(Bnew) 

    for i in range(0, ns):
        brow = Bnew[i,:]
        ind_st = i * (ns - 1)
        ind_end = ind_st + ns - 1
        Bin[i,ind_st:ind_end] = brow
    #print("Bin =")
    #print(Bin)

    for i in range(0, ns):
      neigh = np.where(Bnew[i,:] != 0)
      
      for elements in neigh:
        elements = elements + 1;
        for element in elements:
          if i < element:
            Neigh[i][element] = 1;
          else:
            Neigh[i][element - 1] = 1;
        #y = np.divide(x,ncols)
        #row = np.zeros(len(neigh[0]), dtype = int)
        #for b in range(0,len(neigh[0])):
           #row[b] = math.ceil(y[b])
           #print(row)
           #col = x - np.multiply((row-1),ncols);
      #Compute Bout matrix
      for neighbors in neigh:
        for neighbor in neighbors:
          neighbor = neighbor + 1
          if i < neighbor:
            ind = neighbor * (ns-1)
            #print('ind:')
            #print(ind)
            Bout[i][ind + i] = 1
          else:
            neighbor = neighbor - 1
            ind = neighbor * (ns-1)
            Bout[i][ind + i - 1] = 1 
    #print('neigh')
    #print(Neigh)
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
            if l == 1:
                np.save('BTpmine.npy', BTp)

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


def LP_defenders(xe_assumed, xref, xf, Aeq, A, Tp, nrows, ncols, alpha_f):
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
    
    alphaf = alpha_f
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
    print("seconds taken to calculate: {}".format(end - start))
    optimize1 = out1.x
    print("status of optimization: {}".format(out1.status))

    for l in range(Tp):
        ind_st_xf = (l) * ns + 1
        ind_end_xf = ind_st_xf + ns - 1
        ee1[:, l] = optimize1[ind_st_xf - 1: ind_end_xf]

    Xf[:, 0] = ee1[:, 0]

    for l in range(Tp):
        ind_st_uf = l * (ns * (ns - 1)) + Tp * ns + 1
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
meter_per_sector_length = 1

T = 10
Tp = 7

ns = nrows * ncols

nf = 3

ne = 1

alphaf = 0.99; betaf = 1-alphaf

round = 0
Bin , Bout, Neigh = Bmatrix(nrows, ncols)
B = Bin - Bout
Aeq = DynamicConstraints(Bin, Bout, Tp, ns)
A = FlowConstraints(Bin, Bout, Tp, ns)

num_games = 1; num_lost = 0; num_won = 0
num_iterations = np.zeros((1, num_games))
cost = np.zeros((1, num_games))

Aest = np.zeros((num_games, ns))

while True:

    for g in range(num_games):

        xf = np.zeros((ns, T + 1))
        Xf = np.zeros((ns, Tp))

        xe = np.zeros((ns, T + 1))
        Xe_model = np.zeros((ns, Tp))

        xref = np.zeros((ns, 1))

        nu = ns * (ns - 1)
        uf = np.zeros((nu, T))
        Uf = np.zeros((nu, Tp))
        enemy_init_pos = 35
        xref[0, 0] = 1
        xref[1, 0] = 1
        xref[2, 0] = 1
        xref[3, 0] = 1
        xref[4, 0] = 1
        xref[5, 0] = 1
        xref[6, 0] = 1
        xref[7, 0] = 1
        # xref[4, 0] = 1
        # xe[15 - 1, 0] = 2/3
        # xe[15, 0] = 1/3
        xe[enemy_init_pos - 1,0] = 1
        xf[1 - 1, 0] = 1
        xf[2 - 1, 0] = 1
        xf[3 - 1, 0] = 1

        t = 1
        tau_diff_e = 0.1

        while t < T:
            # rospy.Subscriber("/sectors", Int32MultiArray, callback)

            print("Xf[t-1]: {}".format(xf[:,t-1]))
            # xe[15 - 1, t-1] = 2/3
            # xe[15,t-1] = 1/3
            xe[enemy_init_pos-1,t-1] = 1
            print("xE: {}".format(xe[:, t-1]))
            xe_assumed = AssumedModel_enemy(xref, xe[:, t - 1], B , Tp, nrows, ncols, Neigh, tau_diff_e)
            # start = timer()
            print("xE_assumed: {}".format(xe_assumed))
            xf[:, t], uf[:, t] = LP_defenders(xe_assumed, xref, xf[:, t - 1], Aeq, A, Tp, nrows, ncols, alphaf)
            # end = timer()
            # print("seconds taken to calculate: {}".format(end - start))
            controls = np.nonzero(uf[:, t])
            controls = controls[0]
            next_sector = -1
            # print("B @ uf: {}".format(np.matmul(B,uf[:,t])))

            # print("controls: {}".format(controls))
            print("xf: {}".format(xf[:,t]))
            # print("uf: {}".format(uf[:,t]))


            for control in controls:
                next_sector = math.floor(control / (ns - 1)) + 1
                offset = control % (ns - 1)
                if offset >= next_sector:
                    prev_sector = offset + 2
                    # break
                else:
                    prev_sector = offset + 1
                    # break
                print("robot at sector {} will go to sector {}.".format(prev_sector, next_sector))

            # rate.sleep()
            time.sleep(1)
            t = t + 1


# GRAVEYARD

# def bmatrix(nrows, ncols):
#     ns = nrows * ncols
#     B = np.zeros((ns, ns-1), dtype=int)
#     Bin = np.zeros((ns, ns*(ns-1)), dtype=int)
#     Bout = np.zeros((ns, ns*(ns-1)), dtype=int)
#     N = np.zeros((nrows, ncols, ns), dtype=int)
#     Neigh = np.zeros((ns, ns), dtype=int)

#     for i in range(1, nrows + 1):
#         for j in range(1, ncols + 1):
#             s = (i - 1) * ncols + j
#             if i == 1:  # First row
#                 if j == 1:  # First column
#                     B[0, 0] = 1
#                     B[0, ncols - 1] = 1
#                     B[0, ncols] = 1
#                 elif j == ncols:  # Last sector in first row
#                     B[s - 1, s - 2] = 1
#                     B[s - 1, 2 * s - 2] = 1
#                     B[s - 1, 2 * s - 3] = 1
#                 else:
#                     B[s - 1, s - 2] = 1
#                     B[s - 1, s - 1] = 1
#                     B[s - 1, s + ncols - 3] = 1
#                     B[s - 1, s + ncols - 2] = 1
#                     B[s - 1, s + ncols - 1] = 1
#             elif i == nrows:  # Last row
#                 sminus = (i - 2) * ncols
#                 if j == 1:
#                     B[s - 1, s - 1] = 1
#                     B[s - 1, sminus] = 1
#                     B[s - 1, sminus + 1] = 1
#                 elif j == ncols:  # Last row, last column
#                     B[s - 1, s - 2] = 1
#                     B[s - 1, sminus + j - 1] = 1
#                     B[s - 1, sminus + j - 2] = 1
#                 else:
#                     B[s - 1, s - 2] = 1
#                     B[s - 1, s - 1] = 1
#                     B[s - 1, sminus + j - 2] = 1
#                     B[s - 1, sminus + j - 1] = 1
#                     B[s - 1, sminus + j] = 1
#             else:
#                 sminus = (i - 2) * ncols
#                 splus = i * ncols
#                 if j == 1:
#                     B[s - 1, sminus + j - 1] = 1
#                     B[s - 1, sminus + j] = 1
#                     B[s - 1, splus + j - 2] = 1
#                     B[s - 1, splus + j - 1] = 1
#                     B[s - 1, s - 1] = 1
#                 elif j == ncols:
#                     B[s - 1, s - 2] = 1
#                     B[s - 1, sminus + j - 1] = 1
#                     B[s - 1, sminus + j - 2] = 1
#                     B[s - 1, splus + j - 3] = 1
#                     B[s - 1, splus + j - 2] = 1
#                 else:
#                     B[s - 1, s - 2] = 1
#                     B[s - 1, s - 1] = 1
#                     B[s - 1, sminus + j - 2] = 1
#                     B[s - 1, sminus + j - 1] = 1
#                     B[s - 1, sminus + j] = 1
#                     B[s - 1, splus + j - 3] = 1
#                     B[s - 1, splus + j - 2] = 1
#                     B[s - 1, splus + j - 1] = 1

#     for i in range(ns):
#         ind_st = i * (ns - 1)
#         ind_end = ind_st + (ns - 1)
#         Bin[i, ind_st:ind_end] = B[i, :]

#     for i in range(1, ns + 1):
#         neigh1 = np.nonzero(B[i - 1,:])[0]
#         neigh = np.add(neigh1,1) 
#         a = np.asarray(neigh > i).nonzero()[0]
#         for j in range(len(a)):
#             neigh[a[j]] += 1
#         Neigh[i - 1, neigh - 1] = 1
#         row = np.ceil(neigh / ncols).astype(int)
#         col = neigh - (row - 1) * ncols
#         for k in range(len(row)):
#             N[row[k] - 1, col[k] - 1, i - 1] = 1

#         ind = (neigh - 1) * (ns - 1)
#         for k in range(len(neigh)):
#             if i< neigh[k]:
#                 Bout[i - 1, ind[k] + i ] = 1
#             else:
#                 Bout[i - 1, ind[k] + i - 1 ] = 1

#     return Bin, Bout, Neigh