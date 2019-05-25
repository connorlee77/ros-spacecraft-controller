#!/usr/bin/env python

import rospy
#from CVXpy.srv import *
from hivemind.srv import *
from cvxpy import *
import numpy as np
import sys
from numpy import transpose
from geometry_msgs.msg import PoseArray, Pose
from scipy.linalg import expm
from math import factorial

def Single_Int_Linearized_matrices_v1(T): # For Single Integrator

	#Linearization
	A = np.zeros((3,3), float)
	B = np.identity(3)

	#Discretization

	F = expm(A*T)

	int_exp_A = np.zeros((3,3), int)

	for ii in range(1,16):
		int_exp_A = int_exp_A + T* (np.linalg.matrix_power(A*T,ii-1)/factorial(ii-1))

	G = np.matmul(int_exp_A,B)

	return F, G

def Double_Int_Linearized_matrices_v1(T): # For Single Integrator

	#Linearization
	A = np.zeros((6,6), float)
	A[0:3,3:6] = np.identity(3)
	B = np.zeros((3,6),float)
	B[0:3,3:6] = np.identity(3)

	#Discretization

	F = np.zeros((6,6),float)
	G = np.zeros((3,6),float)
	F = expm(A*T)

	int_exp_A = np.zeros((6,6), int)

	for ii in range(1,16):
		int_exp_A = int_exp_A + T* (np.linalg.matrix_power(A*T,ii-1)/factorial(ii))

	G = np.matmul(int_exp_A,B.transpose())

	return F, G

def handle_CVXpy(req):

	# Problem data. (To be given as inputs)
	num_steps = req.num_steps 	#5
	num_this_path = req.num_this_path
	#print num_steps
	x_0m = req.x_0 				#vec([1., 1., 0.])
	# print x_0m
	x_0 = vec([x_0m.x, x_0m.y, x_0m.z])
	# print x_0

	x_fm = req.x_f 				#vec([2., 3., 4.])
	# print x_fm
	x_f = vec([x_fm.x, x_fm.y, x_fm.z])
	# print x_f

	U_max = req.U_max 			#1
	# print U_max

	# this_pathm = req.this_path 	#[0, 5, 2, 8, 1]
	# print "Printing this_path"
	# print this_pathm
	# this_path = []
	# for i in range(num_steps):
	# 	this_path += [this_pathm.data[i]]

	# print this_path

	nodesm = req.nodes 			#bmat([[1.,1.,0.], [2.,3.,4.], [1.6,2.5,2.7], [3.2,1.2,3.4], [1.3,4.,2.], [1.3,1.9,1.5], [9.2,3.4,2.8], [3.2,1.2,8.3], [1.9,2.8,3.8]] )
	# print "Printing nodes"

	nodesn = [];
	for i in range(num_this_path):
		# print i
		# print nodesm.poses[i].position.x
		one_node = [nodesm.poses[i].position.x, nodesm.poses[i].position.y, nodesm.poses[i].position.z]
		nodesn += [one_node]

	# print "Nodesn ", nodesn

	nodes = bmat(nodesn)
	# print nodes

	# nodes_old = bmat([[1.,1.,0.], [2.,3.,4.], [1.6,2.5,2.7], [3.2,1.2,3.4], [1.3,4.,2.], [1.3,1.9,1.5], [9.2,3.4,2.8], [3.2,1.2,8.3], [1.9,2.8,3.8]] )
	# print nodes_old

	radiim = req.radii			#[2,2,2,2,2,2,2,2,3]
	# print "Radii ", radiim

	radii = [];
	for i in range(num_this_path):
		radii += [radiim.data[i]]

	# print radii

	states = []
	x_opt = PoseArray()
	u_opt = PoseArray()
	val = 0.0

	if req.type_of_dynamics == 1:
		#    TODO cvx_precision high
		# Construct the problem.
		x = Variable(num_steps,3)
		u = Variable(num_steps-1,3)
		# *, +, -, / are overloaded to construct CVXPY objects.
		# <=, >=, == are overloaded to construct CVXPY constraints.

		for k in range(num_steps-1):
			cost = norm(u[k,:],2)	#cost
			constr = [x[k+1,:] == x[k,:] + u[k,:], norm(u[k,:],'inf')<= U_max ]  #State constraints
			states.append( Problem( Minimize( cost ), constr ) )

		prob = sum(states)
		prob.constraints += [x[0,:] == x_0.T, x[num_steps-1,:] == x_f.T]   # BC constraints

		# # REBECCA
		# for k in range(1,num_steps-1):
		# 	prob.constraints.append(norm( x[k,:] - nodes[k-1,:]) <= radii[k-1] )
		# 	prob.constraints.append(norm( x[k,:] - nodes[k,:] ) <= radii[k] )


		for k in range(0,num_this_path-2):
			prob.constraints.append(norm( x[2*k+1,:] - nodes[k+1,:]) <= radii[k+1] )
			prob.constraints.append(norm( x[2*k+1,:] - nodes[k,:] ) <= radii[k] )

		for k in range(1,num_this_path-1):
			prob.constraints.append(norm( x[2*k,:] - nodes[k,:]) <= radii[k] )
			# The optimal objective is returned by prob.solve().
		result = prob.solve()
		# The optimal value for x is stored in x.value.
		print("status:", prob.status)
		print("optimal value", prob.value)
		print(x.value)
		# print(u.value)
		#return CVXpyResponse(x.value, u.value, prob.value)

		for i in range(num_steps):
			some_pose = Pose()
			some_pose.position.x = x.value[i,0]
			some_pose.position.y = x.value[i,1]
			some_pose.position.z = x.value[i,2]
			x_opt.poses.append(some_pose)

		for i in range(num_steps-1):
			some_pose = Pose()
			some_pose.position.x = u.value[i,0]
			some_pose.position.y = u.value[i,1]
			some_pose.position.z = u.value[i,2]
			u_opt.poses.append(some_pose)

		val = prob.value


	elif req.type_of_dynamics == 2:

		delta_t = req.delta_t

		print("delta t", delta_t)

		print("num_steps: ", num_steps)

		which_radius_arraym = req.which_radius_array
		which_radius_array = [];
		for i in range(0,num_steps):
			which_radius_array += [which_radius_arraym.data[i]]

		# this_pathm = req.this_path
		# this_path = [];
		# for i in range(num_this_path):
		# 	this_path += [this_pathm.data[i]]

		x = Variable(num_steps,3)
		u = Variable(num_steps-1,3)

		# for k in range(0, num_steps):
		# 	print"which_radius_array:", which_radius_array[k]

		# for k in range(0, num_this_path):
		# 	print"this_path: ", this_path[k]

		F,G = Single_Int_Linearized_matrices_v1(delta_t)
		print(F,G)
		for k in range(num_steps-1):
			cost = (norm(u[k,:],2))*delta_t
			#constr = [x[k+1,:] == x[k,:] + u[k,:], norm(u[k,:],'inf')<= U_max ]  #State constraints
			constr = [x[k+1,:].T == F*x[k,:].T + G*u[k,:].T, norm(u[k,:].T,'inf')<= U_max ]  #State constraints
			states.append( Problem( Minimize( cost ), constr ) )

		prob = sum(states)
		prob.constraints += [x[0,:] == x_0.T, x[num_steps-1,:] == x_f.T]   # BC constraints

		for k in range(0,num_steps-1):
			idx_start = which_radius_array[k]
			#print "idx start:", idx_start
			idx_end = which_radius_array[k+1]
			#print "idx end:", idx_end

			if idx_start == idx_end: # Only in one sphere
				prob.constraints.append(norm( x[k, :] - nodes[idx_start,:] ) <= radii[idx_start] )
			else: #intersection of two spheres
				prob.constraints.append(norm( x[k, :] - nodes[idx_start,:] ) <= radii[idx_start] )
				prob.constraints.append(norm( x[k, :] - nodes[idx_end,:] ) <= radii[idx_end] )
			# The optimal objective is returned by prob.solve().
		result = prob.solve()
		# The optimal value for x is stored in x.value.
		print("status:", prob.status)
		print("optimal value", prob.value)
		print("Optimal X", x.value)
		# print u.value
		#return CVXpyResponse(x.value, u.value, prob.value)
		if prob.status == "infeasible":
			print("problem didn't solve. Exiting")
		else:
			for i in range(num_steps):
				some_pose = Pose()
				some_pose.position.x = x.value[i,0]
				some_pose.position.y = x.value[i,1]
				some_pose.position.z = x.value[i,2]
				x_opt.poses.append(some_pose)

			for i in range(num_steps-1):
				some_pose = Pose()
				some_pose.position.x = u.value[i,0]
				some_pose.position.y = u.value[i,1]
				some_pose.position.z = u.value[i,2]
				u_opt.poses.append(some_pose)

			val = prob.value

	elif req.type_of_dynamics == 3:

		delta_t = req.delta_t

		# print "delta t", delta_t

		# print "num_steps: ", num_steps

		which_radius_arraym = req.which_radius_array
		which_radius_array = [];
		for i in range(0,num_steps):
			which_radius_array += [which_radius_arraym.data[i]]

		# this_pathm = req.this_path
		# this_path = [];
		# for i in range(num_this_path):
		# 	this_path += [this_pathm.data[i]]

		# Velocity constraints
		v_0 = vec([0,0,0])
		v_f = vec([0,0,0])

		x = Variable(num_steps,6)
		u = Variable(num_steps-1,3)

		# for k in range(0, num_steps):
		# 	print"which_radius_array:", which_radius_array[k]

		# for k in range(0, num_this_path):
		# 	print"this_path: ", this_path[k]
		F,G = Double_Int_Linearized_matrices_v1(delta_t)
		print("F ", F)
		print("G ", G)
		for k in range(num_steps-1):
			cost = (norm(u[k,:],2))*delta_t
			#constr = [x[k+1,:] == x[k,:] + u[k,:], norm(u[k,:],'inf')<= U_max ]  #State constraints
			constr = [x[k+1,:].T == F*x[k,:].T + G*u[k,:].T, norm(u[k,:].T,'inf')<= U_max ]  #State constraints
			states.append( Problem( Minimize( cost ), constr ) )

		prob = sum(states)
		prob.constraints += [x[0,0:3] == x_0.T, x[num_steps-1,0:3] == x_f.T]   # BC constraints
		prob.constraints += [x[0,3:6] == v_0.T, x[num_steps-1,3:6] == v_f.T]   # Vel BC constraints

		for k in range(0,num_steps-1):
			idx_start = which_radius_array[k]
			idx_end = which_radius_array[k+1]
			# print "Radius Start ", radii[idx_start], " Radius End ", radii[idx_end]

			if idx_start == idx_end: # Only in one sphere
				prob.constraints.append(norm( x[k, 0:3] - nodes[idx_start,:] ) <= radii[idx_start] )
			else: #intersection of two spheres
				# print "idx start:", idx_start
				# print "idx end:", idx_end
				# print "Node start ", nodesn[idx_start], " Rad Start ", radii[idx_start]
				# print "Node end ", nodesn[idx_end]
				# print " Rad End ", radii[idx_end]
				prob.constraints.append(norm( x[k, 0:3] - nodes[idx_start,:] ) <= radii[idx_start] )
				prob.constraints.append(norm( x[k, 0:3] - nodes[idx_end,:] ) <= radii[idx_end] )
		# The optimal objective is returned by prob.solve().
		result = prob.solve()
		# The optimal value for x is stored in x.value.
		print("status:", prob.status)
		print("optimal value", prob.value)
		# print x.value
		# print u.value
		#return CVXpyResponse(x.value, u.value, prob.value)
		if prob.status == "infeasible":
			print("problem didn't solve. Exiting")
		else:
			for i in range(num_steps):
				some_pose = Pose()
				some_pose.position.x = x.value[i,0]
				some_pose.position.y = x.value[i,1]
				some_pose.position.z = x.value[i,2]
				x_opt.poses.append(some_pose)

			for i in range(num_steps-1):
				some_pose = Pose()
				some_pose.position.x = u.value[i,0]
				some_pose.position.y = u.value[i,1]
				some_pose.position.z = u.value[i,2]
				u_opt.poses.append(some_pose)

			val = prob.value


	return CVXpyResponse(x_opt, u_opt, val)

def cvxpy_server():
	rospy.init_node('cvxpy_server')
	s = rospy.Service('CVXpy', CVXpy, handle_CVXpy)
	print("Ready to Optimize.")
	rospy.spin()

if __name__ == "__main__":
	cvxpy_server()
