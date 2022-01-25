

import numpy as np
import general_robotics_toolbox as rox
import quadprog as qp
from math import pi, sin,cos,atan2
from numpy.linalg import norm
from cvxopt import matrix, solvers
# from trac_ik_python.trac_ik import IK

# solvers.options['show_progress'] = False
# ik_solver = IK("j2n6s300_link_base","j2n6s300_link_6")


class Oarbot(object):
    def __init__(self, mobile_base2arm_base_xy, is_left_arm_config, base_z_up_limit, base_z_low_limit):
        self.ex = np.array([1.,0.,0.])
        self.ey = np.array([0.,1.,0.])
        self.ez = np.array([0.,0.,1.])

        # Kinova arm link lengths
        self.D1 = 0.2755
        self.D2 = 0.41
        self.e2 = 0.0098
        self.D3 = 0.2073
        self.D4 = 0.0741
        self.D5 = 0.0741
        self.D6 = 0.1600

        # Auxiliary variables
        self.aa =  pi/6
        self.ca =  cos(self.aa)
        self.sa =  sin(self.aa)
        self.c2a = cos(2*self.aa)
        self.s2a = sin(2*self.aa)
        self.d4b = self.D3 + self.D5*(self.sa/self.s2a) 
        self.d5b = self.D4*(self.sa/self.s2a) + self.D5*(self.sa/self.s2a) 
        self.d6b = self.D5*(self.sa/self.s2a) + self.D6
        
        # Product of exponential parameters (kinova arm)
        if is_left_arm_config:
            h1 = -self.ez
            h2 = -self.ey
            h3 = self.ey
            h4 = self.ez
            h5 = self.ey*self.ca + self.ez*self.sa
            h6 = self.ey*self.ca - self.ez*self.sa
        else: # Default arm configuration is right arm
            h1 = -self.ez
            h2 = self.ey
            h3 = -self.ey
            h4 = self.ez
            h5 = -self.ey*self.ca + self.ez*self.sa
            h6 = -self.ey*self.ca - self.ez*self.sa
        self.H_arm = np.array([h1, h2, h3, h4, h5, h6]).T

        if is_left_arm_config:
            P01 = (self.D1)*self.ez
            P12 = 0*self.ez
            P23 = -(self.D2)*self.ex - (self.e2)*self.ey
            P34 = 0*self.ez
            P45 = -(self.d5b*self.ca)*self.ey - (self.d5b*self.sa + self.d4b)*self.ez
            P56 = 0*self.ez 
            P6e = -(self.d6b*self.ca)*self.ey + (self.d6b*self.sa)*self.ez 
        else: # Default arm configuration is right arm
            P01 = (self.D1)*self.ez
            P12 = 0*self.ez
            P23 = (self.D2)*self.ex + (self.e2)*self.ey
            P34 = 0*self.ez
            P45 = (self.d5b*self.ca)*self.ey - (self.d5b*self.sa + self.d4b)*self.ez
            P56 = 0*self.ez 
            P6e = (self.d6b*self.ca)*self.ey + (self.d6b*self.sa)*self.ez 
        self.P_arm = np.array([P01, P12, P23, P34, P45, P56, P6e]).T

        # Tool frame (end effector) adjustment 
        if is_left_arm_config:
            self.p_tool = 0*self.ez
            ex_tool = self.ey*self.sa + self.ez*self.ca
            ey_tool = -self.ex
            ez_tool = -self.ey*self.ca + self.ez*self.sa 
        else: # Default arm configuration is right arm
            self.p_tool = 0*self.ez
            ex_tool = -self.ey*self.sa + self.ez*self.ca
            ey_tool = self.ex
            ez_tool = self.ey*self.ca + self.ez*self.sa
        self.R_tool = np.array([ex_tool,ey_tool,ez_tool]).T

        # Parameters to create the robot object properly
        self.joint_types_arm = np.array([0,0,0,0,0,0]) # All revolute
        # self.joint_upper_limits = np.radians([10000,310,341,10000,10000,10000])
        # self.joint_lower_limits = np.radians([-10000,50,19,-10000,-10000,-10000])
        self.joint_upper_limits_arm = None
        self.joint_lower_limits_arm = None

        # Joint angles in Zero config 
        self.q_zeros_arm = np.array([pi,-pi/2,pi/2,pi,pi,0.])
        # self.q_zero = np.deg2rad(np.array([180,270,90,180,180,0]))

        # for arm inv
        # Create the kinova robot object with the general robotics toolbox
        self.arm_bot = rox.Robot(self.H_arm,
                                self.P_arm,
                                self.joint_types_arm,
                                self.joint_lower_limits_arm,
                                self.joint_upper_limits_arm, 
                                R_tool=self.R_tool, p_tool=self.p_tool)

        # Oarbat base robot parameters
        self.L1 = mobile_base2arm_base_xy[0]
        self.L2 = mobile_base2arm_base_xy[1]
        self.L3 = base_z_low_limit + self.D1 

        hB1 = self.ex
        hB2 = self.ey
        hB3 = self.ez # revolute
        hB4 = self.ez
        self.H = np.array([hB1, hB2, hB3, hB4, h1, h2, h3, h4, h5, h6]).T

        PB01 = 0.*self.ex
        PB12 = 0.*self.ey
        PB23 = 0.*self.ez
        PB34 = (self.L1)*self.ex + (self.L2)*self.ey + (self.L3)*self.ez  
        P01_new = 0*self.ez # Since PB34 already captures D1 length
        self.P = np.array([PB01, PB12, PB23, PB34, P01_new, P12, P23, P34, P45, P56, P6e]).T

        self.tolerance_meter = 0.01 # 1 cm tolerance for joint limits
        self.joint_types = np.array([1,1,0,1,0,0,0,0,0,0])
        self.joint_upper_limits = np.append([10000.,10000.,10000.,base_z_up_limit-base_z_low_limit+self.tolerance_meter],np.radians([10000.,10000.,10000.,10000.,10000.,10000.]))
        self.joint_lower_limits = np.append([-10000.,-10000.,-10000.,-self.tolerance_meter],np.radians([-10000.,-10000.,-10000.,-10000.,-10000.,-10000.]))
        
        # Joint angles in Zero config 
        self.q_zeros = np.array([0.,0.,0.,0.,pi,-pi/2,pi/2,pi,pi,0])

        # Create the kinova robot object with the general robotics toolbox
        self.bot = rox.Robot(self.H,
                                self.P,
                                self.joint_types,
                                self.joint_lower_limits,
                                self.joint_upper_limits, 
                                R_tool=self.R_tool, p_tool=self.p_tool)

        # opt param
        self._ep = 0.01
        self._er = 0.02
        self._n = 10.

        

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

        alpha = 1.
        Kp = 1.
        
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

        H3 = -2.*np.dot(np.hstack((J,np.zeros((6,2)))).T, tmp)
        H3 = (H3+H3.T)/2.

        tmp2 = np.vstack((np.array([0,0,0,0,0,0,0,0,0,0,np.sqrt(self._er),0]),np.array([0,0,0,0,0,0,0,0,0,0,0,np.sqrt(self._ep)])))
        H4 = np.dot(tmp2.T, tmp2)

        H = 2.*(H1+H2+H3+H4)

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
