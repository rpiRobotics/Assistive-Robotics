
from os import umask
from general_robotics_toolbox.general_robotics_toolbox import R2q
import numpy as np
import general_robotics_toolbox as rox
from numpy.core.fromnumeric import size
from numpy.core.records import array
import quadprog as qp
from math import pi, sin,cos,atan2
from scipy.linalg import norm
from cvxopt import matrix, solvers
from trac_ik_python.trac_ik import IK

# solvers.options['show_progress'] = False
# ik_solver = IK("j2n6s300_link_base","j2n6s300_link_6")

ex = np.array([1,0,0])
ey = np.array([0,1,0])
ez = np.array([0,0,1])

class Oarbot(object):
    def __init__(self) -> None:
        super().__init__()

        # robot parameters
        wheel_r,d1 = 0.127, 0.2755
        l1,l2 = 0.3, wheel_r+0.3+d1
        d2, d3, e2, d4, d5, d6= 0.41, 0.2073, 0.0098, 0.0741, 0.0741, 0.16
        aa = pi/6
        p89x = d4*(sin(aa)/sin(2*aa)) + 2*d4*(sin(aa)/sin(2*aa))*cos(2*aa)
        p89y = 2*d4*sin(aa)
        H = np.array([ex,ey,ez,ez,-ez,ey,-ey,-ex,-sin(pi/6)*ex+cos(pi/6)*ey,-ex]).T
        P = np.array([0*ex,0*ex,0*ex,l1*ex+l2*ez,0*ex,0*ex,d2*ez,d3*ex+e2*ey,p89x*ex-p89y*ey,0*ex,(d6+d4*sin(aa)/sin(2*aa))*ex]).T
        joint_type = np.array([1,1,0,1,0,0,0,0,0,0])
        joint_upper_limit = np.append([10000,10000,10000,0.5],np.radians([10000,130,180,10000,10000,10000]))
        joint_lower_limit = np.append([-10000,-10000,-10000,-0.001],np.radians([-10000,-130,-71,-10000,-10000,-10000]))
        
        # decalre robot
        self.bot = rox.Robot(H,P,joint_type,joint_lower_limit,joint_upper_limit)
        self.q_zeros = np.array([0,0,0,0,pi,pi,pi/2,0,0,0])

        # opt param
        self._ep = 0.01
        self._er = 0.02
        self._n = 10

        # for arm inv
        H_arm = np.array([-ez,ey,-ey,-ex,-sin(pi/6)*ex+cos(pi/6)*ey,-ex]).T
        P_arm = np.array([d1*ez,0*ez,d2*ez+e2*ey,0*ex,(p89x+d3)*ex-p89y*ey,0*ex,(d6+d4*sin(aa)/sin(2*aa))*ex]).T
        # P_arm = np.array([d1*ez,0*ez,d2*ez,d3*ex+e2*ey,p89x*ex-p89y*ey,0*ex,(d6+d4*sin(aa)/sin(2*aa))*ex]).T
        joint_type_arm = np.array([0,0,0,0,0,0])
        # joint_upper_limit_arm = np.radians([10000,130,90,10000,10000,10000])
        # joint_lower_limit_arm = np.radians([-10000,-130,-71,-10000,-10000,-10000])
        joint_upper_limit_arm = np.radians([10000,130,180,10000,10000,10000])
        joint_lower_limit_arm = np.radians([-10000,-130,-71,-10000,-10000,-10000])
        self.arm_bot = rox.Robot(H_arm,P_arm,joint_type_arm,joint_lower_limit_arm,joint_upper_limit_arm)
        self.q_zeros_arm = np.array([pi,pi,pi/2,0,0,0])

    def fwdkin(self,q):
        
        # foward kinematics for oarbot
        return rox.fwdkin(self.bot,q)
    
    def fwdkin_arm(self,q):

        # foward kinematics for arm
        return rox.fwdkin(self.arm_bot,q)
    
    def jacobian(self,q):

        # jacobian matrix for oarbot
        return rox.robotjacobian(self.bot,q)
    
    def arm_jacobian(self,q):

        # jacobian matrix for kinova_arm
        return rox.robotjacobian(self.arm_bot,q)

    def invkin(self, end_T, init_guess):

        alpha = 1
        Kp = 1
        
        robot_i = self.bot
        q = init_guess

        # start qp
        now_T = self.fwdkin(q)
        dX = np.reshape(np.append(self.s_err(now_T.R*end_T.R.T,2), now_T.p-end_T.p),(6,1))
        umax = (self.bot.joint_upper_limit-q)/alpha
        umin = (self.bot.joint_lower_limit-q)/alpha

        upper = np.array([1,1,2*pi,1,2*pi,2*pi,2*pi,2*pi,2*pi,2*pi])

        umax = np.multiply((umax>upper),upper)+np.multiply((umax<=upper),umax)
        umin = np.multiply((umin<-upper),-upper)+np.multiply((umin>=-upper),umin)

        A = np.vstack((-np.eye(10), np.eye(10)))
        b = np.reshape(np.append(-umax, umin),(20,))
        J = self.jacobian(q)
        H = np.matmul(J.T,J) + 0.0000001*np.eye(10)
        f = Kp*np.matmul(J.T,dX).flatten()

        sc = norm(H,'fro')
        qp_sln = qp.solve_qp(H/sc, -f/sc, A.T, b)[0]
        q = q+alpha*qp_sln[0:self._n]

        return q
    
    def inequality_bound(self, h):
        sigma = np.zeros((h.shape))
        h2 = h - self._eta
        sigma[np.array(h2 >= self._epsilon)] = -np.tan(self._c*np.pi/2)
        sigma[np.array(h2 >= 0) & np.array(h2 < self._epsilon)] = -np.tan(self._c*np.pi/2/self._epsilon*h2[np.array(h2 >= 0) & np.array(h2 < self._epsilon)])
        sigma[np.array(h >= 0) & np.array(h2 < 0)] = -self._E*h2[np.array(h >= 0) & np.array(h2 < 0)]/self._eta
        sigma[np.array(h < 0)] = self._E

        return sigma

    def getqp_f(self):
        f = -2*np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, self._er, self._ep]).reshape(self._n+2, 1)

        return f

    def getqp_H(self, J, vr, vp):
        H1 = np.dot(np.hstack((J,np.zeros((6,2)))).T,np.hstack((J,np.zeros((6,2)))))

        tmp = np.vstack((np.hstack((np.hstack((np.zeros((3, self._n)),vr)),np.zeros((3,1)))),np.hstack((np.hstack((np.zeros((3,self._n)),np.zeros((3,1)))),vp)))) 
        H2 = np.dot(tmp.T,tmp)

        H3 = -2*np.dot(np.hstack((J,np.zeros((6,2)))).T, tmp)
        H3 = (H3+H3.T)/2

        tmp2 = np.vstack((np.array([0,0,0,0,0,0,0,0,0,0,np.sqrt(self._er),0]),np.array([0,0,0,0,0,0,0,0,0,0,0,np.sqrt(self._ep)])))
        H4 = np.dot(tmp2.T, tmp2)

        H = 2*(H1+H2+H3+H4)

        return H

    def s_err(self,er_mat,type):
        
        qua = rox.R2q(er_mat)
        qk = rox.R2rot(er_mat)
        if type == 1:
            err = 4*qua[0]*qua[1:4]
        elif type == 2:
            err = 2*qua[1:4]
        elif type == 3:
            err = 2*qk[1]*qk[1:4]
        else:
            raise ValueError("Type = 1 or 2 or 3")
        
        return err
    
    def invkin_arm(self,end_T,init_guess):
        
        alpha = 1
        Kp = 0.3
        N=30
        jn = len(self.arm_bot.joint_lower_limit)
        
        robot_i = self.arm_bot
        q = init_guess

        # start qp
        for i in range(N):
            now_T = rox.fwdkin(self.arm_bot,q)
            dX = np.reshape(np.append(self.s_err(now_T.R*end_T.R.T,2), now_T.p-end_T.p),(6,1))

            umax = (self.arm_bot.joint_upper_limit-q-0.0001)
            umin = (self.arm_bot.joint_lower_limit-q+0.0001)

            upper = np.array([2*pi,2*pi,2*pi,2*pi,2*pi,2*pi])

            umax = np.multiply((umax>upper),upper)+np.multiply((umax<=upper),umax)
            umin = np.multiply((umin<-upper),-upper)+np.multiply((umin>=-upper),umin)

            # A = np.vstack((-np.eye(jn), np.eye(jn)))
            # b = np.reshape(np.append(-umax, umin),(jn*2,))
            A = np.vstack((np.eye(jn), -np.eye(jn)))
            # b = np.reshape(np.append(umax, -umin),(jn*2,))
            b = np.append(umax, -umin)
            J = rox.robotjacobian(self.arm_bot,q)
            # H = np.matmul(J.T,J) + 0.0000001*np.eye(jn)
            H = np.matmul(J.T,J)
            f = Kp*np.matmul(J.T,dX).flatten()

            sc = norm(H,'fro')
            # qp_sln = qp.solve_qp(H/sc, -f/sc, A.T, b)[0]
            # qp_sln = solvers.qp(matrix(H/sc), matrix(f/sc), matrix(A), matrix(b))
            qp_sln = solvers.qp(matrix(H), matrix(f), matrix(A), matrix(b))
            # print("jt",J.Tz)
            # print("inv",np.linalg.inv((J*J.T+0.01*np.diag([1./50,1./50,1./50,1,1,1]))))
            # print("dx",dX)
            # qp_sln = np.matmul(J.T,np.matmul(np.linalg.inv(np.matmul(J,J.T)+0.01*np.diag([1./50,1./50,1./50,1,1,1])),dX))
            # print("sln",qp_sln)

            # q = q+qp_sln[0:jn]
            q = q+np.array(qp_sln['x']).flatten()
            # q = q-alpha*qp_sln.flatten()
            q = np.multiply((q>pi),q-2*pi)+np.multiply((q<=-pi),q+2*pi)+np.multiply(np.multiply((-pi<q),(q<=pi)),q)

        # ik_solver.set_joint_limits(self.arm_bot.joint_lower_limit, self.arm_bot.joint_upper_limit)
        # qua = rox.R2q(end_T.R)
        # q = ik_solver.get_ik(init_guess, end_T.p[0], end_T.p[1], end_T.p[2], qua[1],qua[2],qua[3],qua[0]) 
        
        
        return q
