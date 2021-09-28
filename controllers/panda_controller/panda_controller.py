from math import pi
from controller import Supervisor
import numpy as np
import math as m
from pytransform3d import rotations as pr

def fk(arm,base=np.identity(4),joint_num=-1):
    """Computes the end-effector pose from the robot's DH parameters and joint configuration"""

    pEE = base                # Cumulative pose of the End Effector 
                              # (initially set up as the base of the robot)
    if joint_num==-1:
        for joint in arm:
            pEE=np.dot(pEE, joint.dhMatrix())
    else:
        for i in range(joint_num):
            pEE=np.dot(pEE, arm[i].dhMatrix())

    return pEE


TIME_STEP = 2000
SENSOR_SAMPLE_RATE = 10

###########################################################################################
## INIT and some class definitions
###########################################################################################
# 0.1: Create the DH class
class DH:
    a = 0.0
    d = 0.0
    alpha = 0.0
    theta = 0.0

    def __init__(self, a=0.0, d=0.0, alpha=0.0, theta=0.0):
        self.a = a
        self.d = d
        self.alpha = alpha
        self.theta = theta

    def print(self):
        print("a = {} d = {} alpha = {} theta = {} \n".format(self.a, self.d, self.alpha, self.theta))

    # 1.1: fill up the DHmatrix function in the DH class
    def dhMatrix(self):
        """Returns a numpy-based Denavit-Hartenberg matrix"""
        row1 = np.array([np.cos(self.theta), -np.sin(self.theta)*np.cos(self.alpha),  np.sin(self.theta)*np.sin(self.alpha), self.a*np.cos(self.theta)])
        row2 = np.array([np.sin(self.theta),  np.cos(self.theta)*np.cos(self.alpha), -np.cos(self.theta)*np.sin(self.alpha), self.a*np.sin(self.theta)])
        row3 = np.array([0.0,                 np.sin(self.alpha),                     np.cos(self.alpha),                    self.d])
        row4 = np.array([0.0,                 0.0,                                    0.0,                                   1.0])
        T = np.array([row1, row2, row3, row4])
        return T

    def setTheta(self, theta):
        self.theta = theta

    def getTheta(self):
        return self.theta

# 0.1: Create the Chain class
class Chain:
    """A chain is a sequence of links, i.e. DH objects"""
    DH = []
    q = []
    dof = 0
    joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
    joint_nodes = []

    sensor_names = ['panda_joint1_sensor', 'panda_joint2_sensor', 'panda_joint3_sensor', 'panda_joint4_sensor', 'panda_joint5_sensor', 'panda_joint6_sensor', 'panda_joint7_sensor']
    sensor_nodes = []
    sensor_sample_rate = 1

    def __init__(self, supervisor, DH = []):
        self.DH = DH
        self.dof = len(self.DH)
        self.supervisor = supervisor
        for i, dh in enumerate(self.DH):
            self.q.append(dh.getTheta())

        for i in range(self.dof):
            joint_name = self.joint_names[i]
            joint_node = supervisor.getDevice(joint_name)

            if joint_node is None:
                print('FAILED to find joint with name: {}'.format(joint_name))
            else:
                print('found motor: {}'.format(joint_name))
                self.joint_nodes.append(joint_node)

            sensor_name = self.sensor_names[i]
            sensor_node = supervisor.getDevice(sensor_name)
            if sensor_node is None:
                print('FAILED to sensor joint with name: {}'.format(sensor_name))
            else:
                print('found sensor: {}'.format(sensor_name))
                sensor_node.enable(self.sensor_sample_rate)
                self.sensor_nodes.append(sensor_node)


    def printDH(self):
        for i, dh in enumerate(self.DH):
            dh.print()

    def printSensorValues(self):
        sensor_values = []
        for i in range(self.dof):
            sensor_node = self.sensor_nodes[i]
            sensor_values.append(sensor_node.getValue())
        print('sensor values: ', sensor_values)


    def printjoints(self):
        for i, q in enumerate(self.q):
            print("joint {} position: {}".format(i,q))

    # 1.2: create a function that moves the transform objects where they should according to DH
    def moveTransforms(self):
        for i in range(self.dof):
            dh = self.DH[i].setTheta(self.q[i])

        T = np.identity(4)
        for i in range(self.dof):
            T = np.dot(T, self.DH[i].dhMatrix())
            #print(T)

    # 2.3: create a function that assigns new values to the joints in the arm
    def moveJoints(self,q=[]):
        if(self.dof != len(q)):
            print("moveJoints: invalid input")
            print("self.dof: {}, len(q): {}".format(self.dof, len(q)))
            return
        for i in range(self.dof):
            self.q[i] = q[i]
            joint_node = self.joint_nodes[i]
            joint_node.setPosition(self.q[i])

    def moveInJointIncrements(self, start_joints, goal_joints, time_step = 10, num_pts = 1000):
        joint_traj = []
        for i in range(self.dof):
            j = np.linspace(start_joints[i], goal_joints[i], num_steps, dtype=float)
            joint_traj.append(j)

        #print('joint trajectory: ', joint_traj)
        step_num = 0
        while supervisor.step(TIME_STEP) != -1 and step_num < num_steps:
            j = []
            for i in range(self.dof):
                j_traj = joint_traj[i]
                j_step = j_traj[step_num]
                j.append(j_step)
            self.moveJoints(j)
            self.moveTransforms()
            print('current joint states: ', self.printjoints())
            step_num+=1



### STEP 0: Preliminaries
# 0.1 : Create the DH class
# 0.2 : Create the Chain class
# Import DH parameters for the UR5e from here:
# https://frankaemika.github.io/docs/control_parameters.html

# dh is your array of DH elements to pass to the constructor of arm
# a, d, alpha, theta
dh1 = DH( 0.0,     0.333,   0.0,        0.0)
dh2 = DH( 0.0,     0.0,    -np.pi/2.0,  0.0)
dh3 = DH( 0.0,     0.316,   np.pi/2.0,  0.0)
dh4 = DH( 0.0825,  0.0,     np.pi/2.0,  0.0)
dh5 = DH(-0.0825,  0.384,  -np.pi/2.0,  0.0)
dh6 = DH( 0.0,     0.0,     np.pi/2.0,  0.0)
dh7 = DH( 0.088,   0.0,     np.pi/2.0,  0.0)
dh = np.array([dh1, dh2, dh3, dh4, dh5, dh6, dh7])

# Creation of a chain object with the DH parameters and transform objects from supervisor
supervisor = Supervisor()
arm = Chain(supervisor,dh)

# Printing parameters just to check that everything works
arm.printDH()
arm.printjoints()

### STEP 1: Denavit Hartenberg representation -- UR5e
# 1.1: fill up the DHmatrix function in the DH class
# 1.2: create a function that moves the transform objects where they should according to DH
arm.moveTransforms()

# run one time step of the robot in order to read the joint positions
# Time step is two seconds so that we can see the difference between STEP 1 and STEP 2
supervisor.step(TIME_STEP)
arm.printSensorValues()


### STEP 2: Forward Kinematics
posEE=fk(arm.DH)
# 2.1: create a fk function in the Kin.py file
#supposed to see =>
print("**************************************");
print("STEP 1: The initial pose of posEE is:\n",posEE);

# 2.2: check that the position above matches with the "position" field of the robotiq gripper
# 2.3: Move the robot: Fill up the "moveJoints" function in the Chain class
# 2.4: Check that moveJoints works
print("Setting next joint position for step 2.")
arm.moveJoints([np.pi/2.0, np.pi/2.0, np.pi/2.0, -np.pi/2.0, np.pi/2.0, np.pi/2.0, np.pi/2.0])
arm.printDH()
arm.moveTransforms()
supervisor.step(TIME_STEP)
arm.printSensorValues()


# 2.5: Check that the position of the end effector in the kinematics matches with the position of the gripper
posEE=fk(arm.DH)
#supposed to see =>
print("**************************************");
print("STEP 2: The pose of posEE is now:\n",posEE);

# # 2.6: Implement a rotational motion on the joints with an update rate of 10ms
start_joints = [np.pi/2.0, -np.pi/2.0, np.pi/2.0, -np.pi/2.0, -np.pi/2.0, np.pi/2.0, np.pi/2.0]
goal_joints = [np.pi/10.0, 0.0, -np.pi/10.0, -np.pi/10.0, -np.pi/10.0, np.pi/10.0, np.pi/10.0]
TIME_STEP = 10
num_steps = 1000
arm.moveInJointIncrements(start_joints, goal_joints, TIME_STEP, num_steps)
arm.printSensorValues()