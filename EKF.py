import numpy as np
import math
import random as random
from manipulation import *
GRAVITY=np.mat(np.array([0,0,9.8]))
x=np.mat(np.zeros(16))#16 states q p v bw ba
xdot=np.mat(np.zeros(16))#16 states derivatives
z=np.mat(np.zeros(3))#real raw data from sensor
zhat=np.mat(np.zeros(3))#H*x_bar
P=np.mat(np.eye(16))#covariance matrix
Q=np.mat(np.zeros((6,6)))#process noise covariance
F=np.mat(np.zeros((16,16)))#state transition
G=np.mat(np.zeros((16,6)))
H=np.mat(np.zeros((3,16)))#observation Matrix
R=np.mat(np.eye(3))#observation noise Matrix
gyro_cov=0.0001
acc_cov = 0.0001
gravity_cov=5.0
current_t=0
#gravity=np.array([0,0,9.8])
initialized = False
imu_initialized = False
magnetic_initialized = False
acc=np.mat(np.zeros(3))
gyro=np.mat(np.zeros(3))

initialized = False
x[0]=1
Q[0:3,0:3]*=gyro_cov
Q[3:6,3:6]*=acc_cov
R*=gravity_cov
x[10:13]=[0,0,0]
x[13:16]=[0,0,0]

initialized = False
imu_initialized = False
magnetic_initialized = False

def predict(gyro, acc, t,bA,bb):#t is the time we read data from sensor
	if imu_initialized ==False:
		imu_initialized=True
		initialized = True
		current_t=t
		phy = math.atan2(acc[1],acc[2])#initial eular angles by using first data from IMU 
		theta = math.atan2(-acc[0],acc[2])
		phy1=phy*180/math.pi
		theta1=theta*180/math.pi	
		rpy=np.array([phy, theta, 0])
		print "phy theta: ", phy1, theta1
		q_init=euler2quaternion(rpy)# returns quaternion
		x[0] = q_init.w
		x[1:4] = q_init.x,q_init.y,q_init.z
	if t <= current_t: return

	#dt = t - current_t #the time difference between reading time 
	dt=0.0001

	Process(gyro, acc,bA,bb) # get state transition matrix. The input parameters are raw data from sensor
	print "x_qian: ", x[10:16]
	x += xdot*dt
	print "x_hou: ", x[10:16]
	F=np.eye(16)+F*dt
	G=G*dt
	P=F*P*F.T+G*Q*G.T
	#P=np.dot(np.dot(F,P),F.transpose())+\
	#np.dot(np.dot(G,Q),G.transpose())

	#!!!!normalize x first 4 terms,i.e. quaternions
	x /= np.linalg.norm(x[0:4],ord = 2)
	current_t=t
	acc=acc
	gyro=gyro


def process(gyro, acc,bA,bb):
	q=np.quaternion(0,0,0,0)#share addtress just make another name
	p=x[4:7]
	v=x[7:10]
	bw=x[10:13]#what is the initail value of bias?! maybe we could use the first 3 seconds average value
	ba=x[13:16]# when the drone is static as init bias
	q.w=x[0]
	q.x,q.y,q.z=x[1:4]
	print "quaternion: ", x[0:4]
	print "position: ", x[4:7]
	print "velocity: ", x[7:10]
	print "biasw: ", x[10:13]
	print "biasa: ", x[13:16]

	gyro_q=np.quaternion(0,0,0,0)
	gyro_q.x, gyro_q.y, gyro_q.z=gyro-bw#-random.gauss(0,0.01) #-bb#
	q_dot=q*gyro_q #matrix multiply this line is correct
	q_dot.w/=2.0
	q_dot.x/=2.0
	q_dot.y/=2.0
	q_dot.z/=2.0
	xdot[0] = q_dot.w
	xdot[1:4] = q_dot.x, q_dot.y, q_dot.z
	xdot[4:7] = v

	acc_b_q = np.zeros(4)
	acc_b_q[1:4] = acc-ba#-random.gauss(0,0.01)#ba-bA
	#print "acc_b_q: ", acc_b_q
	acc_b_q = array2q(acc_b_q)
	#print "acc_b_q quat: ", acc_b_q
	acc_n_q = q2array(q*acc_b_q*q_inverse(q))
	#print "acc_n_q array: ", acc_n_q
	xdot[7:10] = acc_n_q[1:4]-GRAVITY
	#print "acc_before gravity minus: ", acc_n_q
	print "final acc_from_model: ", xdot[7:10]

	F[0:4,0:4]=0.5*diff_pq_p(gyro_q)
	F[0:4,10:13]=-0.5*diff_pq_q(q)[0:4,1:4]
	F[4:7,7:10]=np.eye(3)
	F[7:10,0:4]=diff_qvqstar_q(q,q2array(acc_b_q)[1:4])
	F[7:10,13:16]=-diff_qvqstar_v(q)

	G[0:4,0:3]=0.5*diff_pq_q(q)[0:4,1:4]
	G[7:10,3:6]=diff_qvqstar_v(q)


def update(acc, t):#acc is the raw data from IMU
	if initialized==False:
		initialized = True
		current_t = t
	if t < current_t: return

	z=acc/np.linalg.norm(acc,ord=2)
	measurement()
	#print "H: ", H
	K = P*H.T*(H*P*H.T+R).I
	#print "K: ",K
	x += K*(z-zhat)
	print "z-zhat: ", z-zhat
	I=np.eye(16)
	print "P qian: ", np.diag(np.mat(P))
	P = (I - K*H)*P
	print "P hou: ", np.diag(np.mat(P))
	x[0:4] = q2array(q_normalize(array2q(x[0:4])))

def measurement(): #acc is model result
	q=np.quaternion(0,0,0,0)
	q.w=x[0]
	q.x,q.y,q.z=x[1:4]
	#ba=x[13:16]
	g_n_q=np.quaternion(0,0,0,1)
	acc_q=q_inverse(q)*g_n_q*q #????????normalize
	#print "acc_q: ", acc_q
	zhat[0:3] = acc_q.x, acc_q.y, acc_q.z
	H[0:3,0:4] = diff_qstarvq_q(q, GRAVITY)

def q_normalize(q):
	sum=math.sqrt(q.w**2+q.x**2+q.y**2+q.z**2)
	q.w/=sum
	q.x/=sum
	q.y/=sum
	q.z/=sum
	return q

def q_inverse(q):
	q.x, q.y, q.z=-q.x, -q.y, -q.z
	 
	q=q/(q.w**2+q.x**2+q.y**2+q.z**2)
	return q

def q2array(q):
	a=np.mat(np.array([q.w,q.x,q.y,q.z]))
	return a

def array2q(a):
	q=np.quaternion(a[0],a[1],a[2],a[3])
	return q

def q_vec(q):
	return q_vec