#!/usr/bin/env python

"""
ROS compatible implementation of SATO for x,y,z controllable systems

The service takes in a list of current positions, and target positions, along with
the index of the agent within the current positions list, and the number N of
independent agents.

"""
import rospy
#import tf
#import csv
import os
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from hivemind.srv import SATO
import cvxpy as cvx
#import pickle
#==========================================================================
# SATO Constants
#==========================================================================
T_max = 20
dt = 1
T_H = 3

R = 0.500
R_obs = 0.250
R_sens = 10.500
R_safe = 1.2*R
U_max = 2.5
V_max = 0.2
bound = 50
B = np.vstack([np.zeros([3, 3]), np.identity(3)])
Trust = 1
Tol_u = 0.01
Tol_x = 0.1
Tol_col = 0.001
eps = 0.001

computed_trajectories = {}

def sato(req):
    """Callback function for each service call

    Parameters
    ----------
    req : SATO.service request
        Parameters to service -- initial/final positions, number of agents, and
        index of this agent
        geometry_msgs/PoseArray req.initial_poses
        geometry_msgs/PoseArray req.final_poses
        int32 req.n
        int32 req.index

    Returns
    -------
    geometry_msgs/PoseArray
        Computed Trajectory of agent

    """
    initial_positions = req.initial_positions
    final_positions = req.final_positions
    index = req.index
    N=req.n

    #computation occured for others in group -- dont redo the whole thing identically
    #unclear if this helps much, they seem to run in parallel? (without being told to...)
    #print("I've already run ", len(computed_trajectories), " times")
    #Yeah this doesnt actually do anything.
    if (tuple(initial_positions), tuple(final_positions)) in computed_trajectories:
        print("Already Computed!")
        return {'trajectory': computed_trajectories[(tuple(initial_positions), tuple(final_positions))][index]}

    initial_pos = np.zeros([3, N])
    final_pos = np.zeros([3, N])
    for i in range(N):
    	initial_pos[:,i] = np.array((initial_positions[i].x, initial_positions[i].y, initial_positions[i].z))

    for i in range(N):
    	final_pos[:,i] = np.array((final_positions[i].x, final_positions[i].y, final_positions[i].z))


    price = np.array([])
    targets = np.array([])
    assignment = np.array([])

    velocity = np.zeros([3, N])

    #perform auction algorithm and trajectory generation -- all the actual work
    SLP_output = AA_SLP(velocity, rospy.get_time(), -1, price, targets, assignment, initial_pos, final_pos, N)

    #print("SLP flag ",  SLP_output['flag'])
    if SLP_output['flag'] == 1:
        trajout = SLP_output['traj']
        traj = np.zeros([trajout.shape[1], 3*N])
        for t in range(trajout.shape[1]):
            for j in range(N):
                for i in range(0,3):
                    traj[t,3*j+i] = trajout[i,t,j]

        trajectories = []
        for i in range(N):
            trajectory = []
            for tcount in range (traj.shape[0]):
                position = Point(traj[tcount,0 + 3 * i], traj[tcount,1 + 3 * i], traj[tcount,2 + 3 * i])
                trajectory.append(position)
            trajectories.append(trajectory)

        computed_trajectories[(tuple(initial_positions), tuple(final_positions))] = trajectories;
        return {'trajectory': trajectories[index]}
    return []

def auction(N, M, price, targets, assignment, initial_pos, xf):
    """Decide assignment of final positions to agents with bidding process
    Parameters
    ----------
    N : int
        Number of agents
    M : type
        Number of final positions
    price : NxM (or empty) np array
        Bidding Values
    targets : Nx1 (or empty) np array
        Number of available targets for each agent
    assignment : Nx1 (or empty) np array
        Assigned final position for each agent
    initial_pos : 3xN np array
        Starting position of each agent
    xf : 3xM np array
        Final target positions

    Returns (price, targets, assignment, initial_pos, x_f)
    -------
    (same types as parameters)
        Final values of parameters

    """
    #---------------------------------Initialization---------------------------
    max_comm_iter = 2*N
    #bidding prices
    if price.shape[0] == 0:
        price = np.zeros([N, M])
        price_old = -np.ones([N, M])
    else:
        price_old = price
    if assignment.shape[0] == 0:
        assignment = np.zeros([N]).astype(int)

    if targets.shape[0] == 0:
        targets = M*np.ones([N]).astype(int)

    #c = cost of  agent choosing each target
    c = np.zeros([N, M])
    for i in range(N):
        for k in range(M):
            c[i, k] = np.linalg.norm(initial_pos[:, i] - xf[:, k])/1000

    price_temp = np.zeros([N, M])
    done = np.zeros([N], dtype=bool)
    count = np.zeros([N])
    final_pos = np.zeros([3, N])

    #---------------------------------Bidding Loop---------------------------
    while np.any(np.logical_not(done)):
        for i in range(N):
            if np.logical_not(done[i]):
                if price[i, assignment[i]] > price_old[i, assignment[i]]: #this agent (i) was outbid
                    #make sure enough targets for agents to bid on
                    nonzero = np.count_nonzero(price[i, :])
                    if nonzero >= targets[i]:
                        targets[i] = nonzero + 1

                    cost = price[i, :] + c[i, :]
                    v = np.amin(cost[0:targets[i]]) #cheapest target
                    assignment[i] = np.argmin(cost[0:targets[i]])
                    ran = np.hstack([range(int(assignment[i])), range(int(assignment[i]+1), int(targets[i]))])
                    ran_int = [int(x) for x in ran]
                    w = np.amin(cost[ran_int]) #second cheapest target
                    gamma = w - v + eps #bidding amount-- adding epsilon so strictly increasing
                    price[i, assignment[i]] += gamma
                    count[i] = 0
                elif np.any(np.logical_not(price == price_old)): #some bid changed, but i wasnt outbid
                    count[i] = 0
                else:
                    count[i] += 1
                #store my bid estimates
                price_old[i, :] = price[i, :]
                done[i] = count[i] > max_comm_iter

        #update actual bid values from neighbors
        for i in range(N):
            Neighbors = np.sqrt((initial_pos[0, i]-initial_pos[0, :])**2 + (initial_pos[1, i] - initial_pos[1, :])**2 + (initial_pos[2, i] - initial_pos[2, :])**2) <= R_sens
            price_temp[i, :] = np.amax(price[Neighbors, :], axis=0)

        price = price_temp

    for i in range(N):
        final_pos[:, i] = xf[:, int(assignment[i])]
    return (price, targets, assignment, initial_pos, final_pos)

def AA_SLP(velocity, time, time_0, price, targets, assignment, initial_pos, x_f, N):

    # if time_0 is empty, make it equal to time
    if time_0 == -1:
        time_0 = time

    T = T_max - int((time-time_0)/dt)
    vel = velocity[:, 0:N]

    #==========================================================================
    # Auction Algorithm
    #==========================================================================
    M = x_f.shape[1] #number of final positions

    (price, targets, assignment, initial_pos, x_f) \
        = auction(N, M, price, targets, assignment, initial_pos, x_f)

    #==========================================================================
    # Initialization
    #==========================================================================

    X = np.zeros([6, T+1, N])
    U = np.zeros([3, T, N])
    J = np.zeros(N)
    distance = np.zeros([T+1, N, N])
    detect = np.zeros([N, N], dtype=bool)
    done = np.zeros(N, dtype=bool)
    feas = np.zeros(N, dtype=bool)
    prior_next = np.array(range(N))

    # Initial and Final Conditions
    x_0 = initial_pos
    dxdt_0 = vel
    dxdt_f = np.zeros([3, N])

    for i in range(N):
        for j in range(i+1, N):
            if np.linalg.norm(x_0[:, i] - x_0[:, j]) < R:
                print("Agents too close", i, j, x_0[:, i], x_0[:, j], np.linalg.norm(x_0[:, i] - x_0[:, j]))
            if np.linalg.norm(x_f[:, i] - x_f[:, j]) < R:
                print("Targets too close", i, j, x_f[:, i], x_f[:, j], np.linalg.norm(x_f[:, i] - x_f[:, j]))

    #==========================================================================
    # Optimization (No collision aviodance)
    #==========================================================================

    #we need the same problems later when we do collision avoidance
    problems = []
    x_variables = []
    x_dot_variables =[]
    u_variables = []
    solved =[]
    for i in range(N):
        states =[]
        x = cvx.Variable(3,T+1)
        x_dot = cvx.Variable(3,T+1)
        u = cvx.Variable(3,T)
        for t in range(T):
            constraints = [ cvx.norm(u[:,t], "inf") <= U_max,
                            cvx.norm(x[:,t], "inf") <= bound,
                            cvx.norm(x_dot[:,t],2) <= V_max,
                            x[:,t+1] == x[:,t] + dt * x_dot[:,t] + 0.5 * dt**2 * u[:,t],
                            x_dot[:,t+1] == x_dot[:,t] + dt * u[:,t]
                            ]
            cost = cvx.norm(u[:,t], 2)
            states.append(cvx.Problem(cvx.Minimize(cost), constraints))

        prob = sum(states)
        initial_final_constraints = [   x[:,0] == x_0[:,i],
                                        x[:,T-1] == x_f[:,i],
                                        x_dot[:,0] == dxdt_0[:,i],
                                        x_dot[:,T-1] == dxdt_f[:,i]
                                    ]
        prob = cvx.Problem(prob.objective, prob.constraints + initial_final_constraints)
        #save problem
        problems.append(prob)
        x_variables.append(x)
        x_dot_variables.append(x_dot)
        u_variables.append(u)


        prob.solve()
        flag = prob.status == cvx.OPTIMAL
        solved.append(flag)
        if flag:
            X[:, :, i] = np.vstack([x.value, x_dot.value])#rows = x,y,z, x_dot, y_dot, z_dot. columns = time steps
            U[:, :, i] = u.value
            J[i] = np.linalg.norm(u.value[0], ord = 1)

    X_prev = X
    J_prev = J

    for i in range(N):
        for k in range(N):
            distance[:, i, k] = np.linalg.norm(X[0:3, :, i] - X[0:3, :, k])
            if i != k:
                detect[i, k] = np.linalg.norm(x_0[0:3, i] - x_0[0:3, k]) < R_sens
            else:
                detect[i, k] = False

    if all(solved):
        cont = True
    else:
        cont = False


    #==========================================================================
    # Optimization (With collision aviodance)
    #==========================================================================
    count = 1
    while cont:
        #print ("Iteration: ", count)
        prior = prior_next
        sc = np.vstack([prior, np.array(range(N))])
        sc_order = sc[:, sc[0, :].argsort()]
        for i in sc_order[1]:
            avoid = detect[i, :]
            # Define Problem -- reuse problem from before, add collision avoidance
            x = x_variables[i]
            x_dot = x_dot_variables[i]
            u = u_variables[i]
            prob = problems[i]
            #collision avoidance
            collision_constraints=[]
            for t in range(1, min(T_H+1, T+1)):
                for k in np.flatnonzero(avoid & (prior < prior[i])):
                    #use estimate of other agents position from previous iter
                    #to avoid collision
                    prev_dist = X_prev[0:3, t, i] - X_prev[0:3, t, k]
                    collision_constraints += \
                        [prev_dist[0]*(x[0,t]-X_prev[0, t, k]) \
                            + prev_dist[1]*(x[1,t]-X_prev[1, t, k]) \
                            + prev_dist[2]*(x[2,t]-X_prev[2, t, k]) \
                        >= R*np.linalg.norm(prev_dist)]
            prob = cvx.Problem(prob.objective, prob.constraints + collision_constraints)
            prob.solve()
            flag = prob.status ==cvx.OPTIMAL
            if flag:
                X[:, :, i] = np.vstack([x.value, x_dot.value])#rows = x,y,z, x_dot, y_dot, z_dot. columns = time steps
                U[:, :, i] = u.value
                J[i] = np.linalg.norm(u.value, ord = 1)
                feas[i] = 1
            else:
                feas[i] = 0
                print "No Solution"
                prior_next[prior_next < prior_next[i]] = prior_next[prior_next < prior_next[i]] + 1
                prior_next[i] = 0

        for i in range(N):
            for k in range(N):
                distance[:, i, k] = np.linalg.norm(X[0:3, :, i] - X[0:3, :, k])
                if i != k:
                    detect[i, k] = np.linalg.norm(x_0[0:3, i] - x_0[0:3, k]) < R_sens
                else:
                    detect[i, k] = False


        coll = np.amax((distance[1:min(T_H+1, T+1), :, :] < (R - Tol_x)) & (distance[1:min(T_H+1, T+1), :, :] > 0), 0)
        safe = np.logical_not(np.amax(coll & detect, 0))
        error = np.amax(np.amax(np.abs(X - X_prev), 0), 0)
        conv = ((np.abs(J-J_prev) < Tol_u) & (J <= J_prev)) | (error < Tol_x)
        done = safe & conv & feas
        # print safe, coll, detect
        if np.amin(safe) == 1:
            prior_next = prior

        if np.amin(done) == 1 or count >= 20:
            cont = False

        X_prev = X
        J_prev = J
        count += 1

    traj = np.hstack([np.zeros([X.shape[0], T_max + 1 - X.shape[1], X.shape[2]]), X])
    control = np.hstack([np.zeros([U.shape[0], T_max - U.shape[1], U.shape[2]]), U])

    if (time - time_0) > (T_max*dt):
        guidance_run = False
    else:
        guidance_run = True

    print "Solved"
    return {'traj': traj, 'control': control, 'flag': flag, 'dt': dt, 'guidance_run': guidance_run,
            'time_0': time_0, 'price': price, 'targets': targets, 'assignment': assignment}

if __name__ == "__main__":
        rospy.init_node('sato_server')
        s = rospy.Service('sato_traj_generation', SATO, sato)
        print "Ready to generate SATO trajectories."
        rospy.spin()
