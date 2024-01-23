#!/usr/bin/env python
#---INITIALIZATION---# 
import rospy
import numpy as np
import scipy.optimize as opt
import numpy.matlib 
import math
import random
from std_msgs.msg import Int32MultiArray, Int16
from robot_handler import handler

#                                                                                                                                                        
#

#---BMATRIX---#
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

#
#

#---Coordinates---#
def coordinates(resource, nrows, ncols):
  ns = nrows * ncols;
  if (resource == 0 ): 
    x = 0;
    y = 0;
  elif (resource == (ns - 1) ):
    x = nrows - 1;
    y = ncols - 1;
  else:
    if (np.mod(resource,nrows) == 0):
      x = 0;
      y = (resource/ncols);
    else: 
      x = np.mod(resource,nrows);
      y = np.floor(resource/ncols); 
  
  return x, y

#
#

#---Distance---#
def distance(dest_array,neigh_e,nrows,ncols):
    #print('dest_array')
    #print(dest_array)
    #ys = np.ceil((neigh_e + 1)/ncols);
    #xs = (neigh_e +  1) - (ys - 1)*nrows;
    
    xs , ys = coordinates(neigh_e, nrows, ncols)
    #print('ys')
    #print(ys)
    #print('xs')
    #print(xs)
    ys = ys + 1
    xs = xs + 1


    sectors = np.where(dest_array != 0)
    #print('sectors are:')
    #print(sectors)
    len_sec = len(sectors[0])
    dist = np.zeros(len_sec, dtype = int);
    yref = np.zeros(len_sec, dtype = int);
    sectors = sectors[0];
    #print('sectors are:')
    #print(sectors)
    for sec in range(0,len(sectors)):
      yref[sec] = math.ceil((sectors[sec])/ncols);
      xref = sectors[sec] - (yref[sec]-1)*nrows ;
      pow1 = (ys - yref[sec]) ** 2;
      pow2 = (xs - xref) ** 2;
      sq = math.sqrt(pow1 + pow2)
      dist[sec] = math.floor(sq);
    #print('hello distance array is ')
    #print(dist)
    b = np.argmin(dist)
    out = dist[b]
    return out, b 

#
#

#---PDF Generator---#
def pdfgen(pmf):
  N = len(pmf);
  F = np.zeros((N));
  F[0] = pmf[0];
  #generate the distribution function
  for k in range(1,N):
      F[k] = F[k-1] + pmf[k];
  b = random.random();
  ind = np.where(F >= b);
  out = ind[0];
  return out 

#
#

#---Diffusion Matrix---#
 
def sign(t1):
    if t1 > 0: 
      return 1;
    elif t1 < 0: 
      return -1
    elif t1 == 0:
      return 0;

  
def Diffusionmatrix (x_source,x_dest,nrows,ncols,neigh,tau_diff):

#This function computes diffusion matrix that will be used as a predicted model for opponents
#x_source: vector containing locations of agents whose trajectories are modeled
#x_dest: vector of locations towards which sources are diffusing.

  ns = nrows*ncols;
  G_diff = np.zeros((ns*(ns-1),ns));
  loc_source = np.where(x_source != 0)
  loc_source = loc_source[0]
  #print('loc_source')
  #print(loc_source)


  for sector in loc_source:
    #print('sector')
    #print(sector)
    neigh_row = neigh[sector,:]
    neigh_s = np.where(neigh_row == 1)
    neigh_s = neigh_s[0];
    print('neigh_S')
    print(neigh_s)
    dist = np.zeros(len(neigh_s));
    for n in range(len(neigh_s)):
      dist_cal = distance(x_dest, neigh_s[n],nrows,ncols);
      dist[n] = dist_cal[0] 
    #print('dist')
    #print(dist)
    
    index = np.argmin(dist)
    output = dist[index]
    tau = tau_diff; #Noise level
    term1 = -(dist-output)/tau;
    prob = np.divide(np.exp(term1),(sum(np.exp(term1))));
    #print('prob')
    #print(prob)
    minimum = np.zeros(len(neigh_s) , dtype = int)
    for m in range(len(neigh_s)):
      minimum[m] = min(0,sign(neigh_s[m] - sector));
    sec_s_adj = sector + minimum;
    ind = ((neigh_s) * (ns-1)) + sec_s_adj;
    for p in range(len(prob)):
      G_diff[ind[p],sector] = prob[p];       
  return(G_diff)

#
#

#---Assumed Model---# 


#--Attacker--#
def AssumedModel_enemy(xref,xe,B,Tp,nrows,ncols,neigh,tau_diff_e):

  #Compute the trajectory for enemy agents based on assumed model for enemies
  ns = nrows*ncols;
  out2 = np.zeros((ns,Tp));
  xe_prev = np.zeros((ns,1));
  Ge = np.zeros((ns*(ns-1),ns));
  print('hello xe is:')
  print(xe)
  xe_prev = xe;

  for time2 in range(0,Tp):
      Ge = Diffusionmatrix(xe_prev,xref,nrows,ncols,neigh,tau_diff_e);
      ue = np.dot(Ge,xe_prev);
      add2 = xe_prev + np.dot(B,ue)
      for z2 in range(ns):
        out2[z2][time2] = add2[z2];
      xe_prev = add2;
  return out2

#
#

#---Flow Constraints---# 
def FlowConstraints(Bin,Bout,Tp,ns):
  B = Bin-Bout;
  BTp = np.zeros((ns, ns*(ns-1)*Tp))
  row_eq1 = 2*Tp*ns;
  col_eq1 = Tp*ns+ Tp*ns*(ns-1);
  A = np.zeros((row_eq1 ,col_eq1));
  for l in range(0, Tp):
    if (l == 0):
        BTp = Bout;
    else:
      BTp = np.concatenate((-B,BTp), axis=1)

    row_st1  = l*ns+1;
    row_end1 =  row_st1 + ns-1;
    col_st1 = Tp*ns+1;
    col_end1 = col_st1 + np.size(BTp[1]) - 1;
    A[(row_st1 - 1) : row_end1, (col_st1 - 1) : col_end1] = BTp;

    row_st1 = row_st1 + Tp*ns;
    row_end1 = row_st1 + ns-1;
    col_st1 = (l)*(ns*(ns-1))+1+Tp*ns;
    col_end1 = col_st1 + ns*(ns-1)-1;
    A[(row_st1 - 1) : row_end1, (col_st1 - 1) : col_end1]= -Bout;

  
  #print(A)
  return(A)

#
#

#---Dynamic Constraints---# 
def DynamicConstraints(Bin,Bout,Tp,ns):
  #Define matrix for equality constraint
  B = Bin - Bout;
  
  #Define Aeq matrix
  row_eq2 = Tp*ns;
  col_eq2 = Tp*ns + Tp*ns*(ns-1);
  Aeq = np.zeros((row_eq2,col_eq2), dtype = int);

  Aeq[ 0 : Tp*ns , 0 : Tp*ns] = np.identity(Tp*ns , dtype = int);

  for iter1 in range(Tp):
    row_st2 = 0;
    row_end2 = 0;
    col_st2 = 0;
    col_end2  = 0;

    BTp = np.zeros((ns, ns*(ns-1)*(iter1+1)));
    BTp = np.tile(B, iter1+1)
    row_st2  = iter1 * ns + 1;
    row_end2 =  row_st2 + ns - 1;
    col_st2 = Tp * ns + 1;
      #print('Btp_size')
      #print(np.size(BTp[0]))
    col_end2 = col_st2 + len(BTp[0]) - 1;
    Aeq[(row_st2 - 1) : row_end2, (col_st2 -1) : col_end2] = -BTp;
    #Aeq = np.concatenate((Aeq,-BTp), axis = 1)
  #print('Aeq')
  #print(Aeq)
  return Aeq

#
#

#---Linear Programming---#

#--Defenders--#
def LP_defenders (xe_assumed,xref,xf,Aeq,A,Tp,nrows,ncols):
  Bin, Bout, neigh = Bmatrix(nrows,ncols)
  loc_xf = np.where(xf!=0)
  #print(loc_xf)

  for defender in loc_xf:
    n = neigh[defender,:]  
  #print(n)



  ns = nrows*ncols;
  ee1 = np.zeros((ns,Tp))
  ee2 = np.zeros((ns*(ns-1),Tp))
  Xf = np.zeros((ns,Tp))
  Uf = np.zeros((ns*(ns-1),Tp))

  Xe = np.zeros((ns*Tp,1))
  Xe_trans = np.zeros((ns*Tp,1))
  Xe_trans = np.transpose(xe_assumed)
  Xe1 = Xe_trans.flatten(order = 'C');


  for xx in range(ns*Tp):
    Xe[xx,0] = Xe1[xx];
  Xref = np.transpose(numpy.matlib.repmat(np.transpose(xref),1,Tp));



  #Cost function parameter
  alphaf = 0.9;
  betaf = 1-alphaf;

  #Cost function
  f1 = np.transpose(-(alphaf*Xe + betaf*Xref));
  f2 = np.zeros((1,Tp*ns*(ns-1)));
  f = np.transpose(np.concatenate((f1, f2), axis=1))
   
  #Dynamic costraints
  beq = np.transpose(numpy.matlib.repmat(np.transpose(xf),1,Tp));
               
  #Flow constraints
  b1 = numpy.matlib.repmat(np.transpose(xf),1,Tp);
  b2 = np.zeros((1,ns*Tp));
  b = np.concatenate((b1, b2), axis=1);
  b = b[0]


  LB = np.zeros((Tp * (ns+ns*(ns-1)),1));
  UB = np.ones((Tp * (ns+ns*(ns-1)),1));
  LB = LB[0]
  UB = UB[0]
  #LB = np.transpose(LB_trans);
  #UB = np.transpose(UB_trans);

  

  out1 = np.zeros((len(f),1));


  out1 = opt.linprog(f, A_ub = A,  b_ub = b, A_eq = Aeq, b_eq = beq, bounds = [0,1], method = 'highs', options = {"maxiter" : 5000, "tol" : 1.000e-6, "disp" : False})
  optimize1 = out1.x

  for l in range(Tp):
    ind_st_xf = (l)*ns + 1;
    ind_end_xf = ind_st_xf + ns - 1;
    ee1[:,l] = optimize1[ind_st_xf - 1 : ind_end_xf];
  #print('ee1')
  #print(ee1[:,0])
  
  Xf[:,0] = ee1[:,0]


  for l in range(Tp):
    ind_st_uf = l*(ns*(ns-1)) + Tp*ns +1;
    ind_end_uf = ind_st_uf + (ns*(ns-1)) - 1;
    ee2[:,l] = optimize1[ind_st_uf - 1 : ind_end_uf];

  Uf[:,0] = ee2[:,0]

  #print('Uf')
  #print(Uf[:,0])   
  out_xf = Xf[:,0];
  out_uf = Uf[:,0];
  return out_xf, out_uf 

#
#

#--ROS--#
robot_sectors = [0,0,0,0]

def callback(data):
  global robot_sectors
  temp = data.data
  robot_sectors = temp
  
  # rospy.loginfo("The attacker's position that i listened is %s", loc)


#---Main---#
#Arena size
nrows = 5;
ncols = 5;

#Number of iterations
T = 30;

#Prediction Horizon
Tp = 3;

#Number of sectors
ns = nrows*ncols;

#Number of defenders
nf = 3;

#Number of attackers
ne = 1;

#Cost function parameter
alphaf = 0.9; betaf = 1-alphaf;

#round
round = 0

#Neighborhood matrices
[Bin,Bout,Neigh] = Bmatrix(nrows,ncols);
B = Bin - Bout;

#Compute Dynamic Constraint Matrix
Aeq = DynamicConstraints(Bin,Bout,Tp,ns);
#print('Aeq')
#print(Aeq)

#Compute Flow Constraint Matrix
A = FlowConstraints(Bin,Bout,Tp,ns);
num_games = 20; num_lost = 0; num_won = 0;
num_iterations = np.zeros((1,num_games));
cost = np.zeros((1,num_games));


#Define matrix for estimation
Aest = np.zeros((num_games,ns));



rospy.init_node('defender', anonymous=True)
robot_name = rospy.get_param('~robot_number')
robot_number = int(robot_name[-1])
pub = rospy.Publisher(f'{robot_name}/goal_sector', Int16, queue_size=10)


while not rospy.is_shutdown():

  for g in range(num_games): 

    
    #State vector for defenders
    xf = np.zeros((ns,T+1));
    Xf = np.zeros((ns,Tp));   #Defenders trajectory over prediction horizon
    
    #State vector for attackers
    xe = np.zeros((ns,T+1));
    Xe_model = np.zeros((ns,Tp)); #Attackers trajectory over prediction horizon
    
    #Reference sectors
    xref = np.zeros((ns,1));
    
    #Input vector for defenders
    nu = ns*(ns-1);
    uf = np.zeros((nu,T));
    Uf = np.zeros((nu,Tp)); #Input over predictive horizon
    
      
    #Initialization for 25 sectors
    xref[0,0] = 1;
    xref[1,0] = 1;
    xref[2,0] = 1;
    xref[3,0] = 1;
    xref[4,0] = 1;


    # xf[0,0] = 1;
    # xf[4,0] = 1;

    t = 1;
    tau_diff_e = 0.1

    while t < T:

      rospy.Subscriber("sectors", Int32MultiArray, callback)
      # loc = int(loc)
      xe[robot_sectors[0],t-1] = 1
      xf[robot_sectors[1], t-1] = 1
      xf[robot_sectors[2], t-1] = 1
      xf[robot_sectors[3], t-1] = 1
      
      xe_assumed = AssumedModel_enemy(xref,xe[:,t-1],B,Tp,nrows,ncols,Neigh,tau_diff_e)
      xf[:,t], uf[:,t] = LP_defenders(xe_assumed,xref,xf[:,t-1],Aeq,A,Tp,nrows,ncols)
      controls = np.where(uf[:,t])
      next_sector = 0
      for control in controls:
        prev_sector = math.floor(control / (ns-1)) + 1

        if prev_sector == robot_sectors[robot_number]:
          offset = control % (ns - 1)
          if offset >= prev_sector:
            next_sector = offset + 2
            break
          else:
            next_sector = offset + 1
            break
      
      pub.publish(next_sector)


      # loc_f = np.where(xf[:,t]!= 0)
      # loc_f = loc_f[0]
      # pub0.publish(loc_f)
      t = t + 1







#num_iterations
#average_iterations = sum(num_iterations)/g