# -*- coding: utf-8 -*-
'''
Step 7:
在这一步中，实现了
1.ASS_robot 添加传感器模型
2.测试碰撞模型

'''
import os, sys
import Sofa
from stlib3.scene import Scene
import numpy as np
from intestinev1 import Intestinev1, Intestinev2, Intestinev3, Intestinev4

dirPath = os.path.dirname(os.path.abspath(__file__)) + '/'
from stlib3.visuals import VisualModel
from fixingbox import FixingBox
from scipy.spatial.transform import Rotation as R


def quat_vector(quat):
    '''
    四元数转换为旋转矩阵，并左乘方向向量[0,0,1]
    :param quat: [qx,qy,qz,w]
    :return: [x,y,z]
    '''
    # 构造四元数
    r = R.from_quat(quat)
    # 转化为旋转矩阵
    rotation_matrix = r.as_matrix()
    # 旋转矩阵左乘方向向量
    vector = np.matmul(rotation_matrix, np.array([0, 0, -1]))
    # euler0 = r.as_euler('xyz', degrees=True)
    # theta=2*math.acos(quat[6])
    # vector = np.zeros((1, 3))
    # vector = quat[3:6]/math.sin(theta/2)
    # vector=quat[3:7].as_as_euler('xyz', thetadegrees=False)
    return vector


def quat_rot(quat):
    '''
    四元数转换为旋转矩阵
    :param quat: [qx,qy,qz,w]
    :return: [x,y,z]
    '''
    # 构造四元数
    r = R.from_quat(quat)
    # 转化为旋转矩阵
    rotation_matrix = r.as_matrix()
    return rotation_matrix


class EmptyController(Sofa.Core.Controller):

    # F_collies = np.zeros((8, 3))

    def __init__(self, *args, **kwargs):
        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        # 指针获取节点
        self.ServoMotor = kwargs["ServoMotor"]
        self.scene = kwargs["scene"]
        if "intestineCollision" in kwargs:
            self.intestineCollision = kwargs["intestineCollision"]
        if "intestineCollisionInner" in kwargs:
            self.intestineCollisionInner = kwargs["intestineCollisionInner"]

        # 获取拉格朗日约束
        self.GenericConstraintSolver = kwargs['GenericConstraintSolver']

        # 传感器列表
        self.Sensorslist = kwargs['Sensors']

        # 获取所有传感器的约束
        self.Constraints = []
        for i in range(1, 11, 1):
            self.Constraints.append(
                self.Sensorslist.getChild("Sensor" + str(i)).CollisionModel.MechanicalObject.constraint)

        # 获取所有传感器的自由度
        self.Sensors = []
        for i in range(1, 11, 1):
            self.Sensors.append(self.Sensorslist.getChild("Sensor" + str(i)).MechanicalModel.dofs)

        #
        self.stepsize = 0.01
        self.d = 1
        self.steppressure = 20
        self.initFlag=1;

    # Default BaseObject functions********************************
    def init(self):
        # self.intestineCollision.SurfacePressureForceField.pressure.value = -300
        pass

    def onAnimateBeginEvent(self, event):  # called at each begin of animation step

        #初始加持状态生成
        if self.initFlag==1:
            self.intestineCollision.SurfacePressureForceField.pressure.value -= 1
            if self.intestineCollision.SurfacePressureForceField.pressure.value<-360:
                self.initFlag=0

        # 计算传感器的力
        # 获取constraint每个约束的法向力（对刚性接触面的方向未知）\
        constraintLambda = self.GenericConstraintSolver.constraintForces.value
        # print(constraintlambda)

        # 对每个传感器进行计算
        f_tmp = np.zeros((10, 3))
        for n in range(0, 10, 1):
            constraint = self.Constraints[n].value
            Sensor_pos = quat_rot(self.Sensors[n].position.value[0][3:7])
            # print(self.Sensors[n].position.value[0][3:7])
            # constraint格式整理成一行
            constraint_oneline = constraint.split('\n')
            # 每个节点的法向约束力

            # 对每一个约束进行计算
            for i in range(len(constraint_oneline)):
                # 第i个约束
                constraint_mat = list(map(eval, constraint_oneline[i].split()))
                if len(constraint_mat) > 0:
                    # 第i个约束的个数j，即第二列
                    # print("约束id：", i, "，约束个数", constraint_mat[1])
                    for j in range(constraint_mat[1]):
                        # 第i个约束影响的节点id
                        id_index = 2 + j * 4
                        ID = constraint_mat[id_index]
                        # 第i个约束影响的节点id的方向
                        left = id_index + 1
                        right = left + 3
                        array_one = np.array([constraint_mat[left:right]]).T
                        # 计算在世界坐标系下的力
                        force = np.dot(array_one, constraintLambda[i])
                        # 左乘旋转矩阵
                        f_tmp[n] += np.matmul(Sensor_pos, force).T[0]
                        # 不左乘旋转矩阵
                        # f_tmp[n] += force.T[0]

        print(f_tmp)
        print("\n")

        pass

    def onKeypressedEvent(self, event):
        key = event['key']
        print(ord(key))
        # print(self.scene.gravity[1])
        if ord(key) == 49:
            print("You pressed the 1 key")
            self.scene.gravity[1] = -9810
        if ord(key) == 50:
            self.scene.gravity[1] = 0
        if ord(key) == 51:
            self.intestineCollision.SurfacePressureForceField.pressure.value += self.steppressure
            print(self.intestineCollision.SurfacePressureForceField.pressure.value)
            print("You pressed the 3 key")
        if ord(key) == 52:
            self.intestineCollision.SurfacePressureForceField.pressure.value -= self.steppressure
            print(self.intestineCollision.SurfacePressureForceField.pressure.value)
            print("You pressed the 4 key")
        if ord(key) == 53:
            self.intestineCollisionInner.SurfacePressureForceField.pressure.value += self.steppressure
            self.intestineCollisionInner.pressure1.pressure.value += self.steppressure

            print(self.intestineCollisionInner.SurfacePressureForceField.pressure.value)
            print("You pressed the 5 key")
        if ord(key) == 54:
            self.intestineCollisionInner.SurfacePressureForceField.pressure.value -= self.steppressure
            self.intestineCollisionInner.pressure1.pressure.value -= self.steppressure

            print(self.intestineCollisionInner.SurfacePressureForceField.pressure.value)
            print("You pressed the 6 key")
        if ord(key) == 55:
            self.intestineCollisionInner.pressure1.pressure.value += self.steppressure

            print(self.intestineCollisionInner.SurfacePressureForceField.pressure.value)
            print("You pressed the 7 key")
        if ord(key) == 56:
            self.intestineCollisionInner.pressure1.pressure.value -= self.steppressure

            print(self.intestineCollisionInner.SurfacePressureForceField.pressure.value)
            print("You pressed the 8 key")
        if ord(key) == 19:  # up
            print("You pressed the Up key")
            # wa[1] = abs(math.cos(factor * 2 * math.pi)) + 0.2  # create angle
            self.ServoMotor.angleIn[1] = self.ServoMotor.angleIn[1] + self.stepsize
            theta1 = np.pi - self.ServoMotor.angleIn[1]
            theta24 = getAngle(11.01 / 2, 30.58, 26.41, np.pi - self.ServoMotor.angleIn[1])
            self.ServoMotor.angleIn[0] = -theta24 + np.pi / 2
            self.ServoMotor.angleIn[2] = np.pi - 2 * (np.pi * 2 - np.pi / 2 - theta24 - theta1)
            self.ServoMotor.angleIn[3] = self.ServoMotor.angleIn[1]

        if ord(key) == 21:  # down
            print("You pressed the Down key")
            self.ServoMotor.angleIn[1] = self.ServoMotor.angleIn[1] - self.stepsize
            theta1 = np.pi - self.ServoMotor.angleIn[1]
            theta24 = getAngle(11.01 / 2, 30.58, 26.41, np.pi - self.ServoMotor.angleIn[1])
            self.ServoMotor.angleIn[0] = -theta24 + np.pi / 2
            self.ServoMotor.angleIn[2] = np.pi - 2 * (np.pi * 2 - np.pi / 2 - theta24 - theta1)
            self.ServoMotor.angleIn[3] = self.ServoMotor.angleIn[1]

        if ord(key) == 91:  # {[
            print("###")
            # print(self.scene.Simulation.Robot.Articulation.ArmWheel.dofs.position.value[0][0])
            with self.scene.Simulation.Robot.Articulation.ArmWheel.dofs.position.writeableArray() as pos:
                for p in pos:
                    p[2] -= 0.1

        if ord(key) == 93:  # }]
            with self.scene.Simulation.Robot.Articulation.ArmWheel.dofs.position.writeableArray() as pos:
                for p in pos:
                    p[2] += 0.1


def getAngle(x1, x2, x3, theta1):
    if theta1 < np.pi:
        x4_2 = x2 * x2 + x3 * x3 - 2 * x2 * x3 * np.cos((theta1))
        x4 = np.sqrt(x4_2)
        costheta2 = (x2 * x2 + x4_2 - x3 * x3) / (2 * x2 * x4)
        theta2 = np.arccos(costheta2)
        theta4 = np.arccos(x1 / x4)
        ans = theta2 + theta4
    else:
        x4_2 = x2 * x2 + x3 * x3 - 2 * x2 * x3 * np.cos((theta1))
        x4 = np.sqrt(x4_2)
        costheta2 = (x2 * x2 + x4_2 - x3 * x3) / (2 * x2 * x4)
        theta2 = np.arccos(costheta2)
        theta4 = np.arccos(x1 / x4)
        ans = -theta2 + theta4
    return ans


def CreateAxis(name="Intestine", filepath='', position=None, translation=None, rotation=None, visiontranslation=None,
               visionrotation=None, color=None):
    if color is None:
        color = [0.5, 0.45, 0.75, 0.7]
    if translation is None:
        translation = [0, 0, 0]
    if rotation is None:
        rotation = [0, 0, 0]
    if visionrotation is None:
        visionrotation = [0, 0, 0]
    if position is None:
        position = [0, 0, 0]

    self = Sofa.Core.Node(name)
    mechanicalModel = self.addChild("MechanicalModel")
    mechanicalModel.addObject('MechanicalObject',
                              name='dofs',
                              size=1,
                              template='Rigid3d',
                              # showObject=True,
                              # showObjectScale=10,
                              translation=translation,
                              rotation=rotation,
                              position=position,
                              )
    # upperArmLongMechanicalModel.addObject('FixedConstraint')
    mechanicalModel.addObject('UniformMass', totalMass=0.1)
    # mechanicalModel.addObject('RigidRigidMapping', name='mapping',
    #                           input="@../../Articulation/ArmWheel/dofs",
    #                           output="@./",
    #                           index=index,  # input frame index,不能改
    #                           )
    # visual model
    visualModel = self.addChild('VisualModel')
    visualModel.addObject('MeshSTLLoader', name='loader', filename=filepath,
                          translation=visiontranslation,
                          rotation=visionrotation)
    visualModel.addObject('MeshTopology', src='@loader')
    visualModel.addObject('OglModel', name='renderer', color=color,
                          writeZTransparent=True)
    visualModel.addObject('RigidMapping',
                          input=mechanicalModel.dofs.getLinkPath(),
                          output=visualModel.renderer.getLinkPath(), )

    return self


def CreateArm(mappingIndex, name="Intestine", filepath='', file1path=None, position=None, translation=None,
              rotation=None,
              visiontranslation=None,
              visionrotation=None, color=None):
    if color is None:
        color = [0.5, 0.45, 0.75, 0.7]
    if translation is None:
        translation = [0, 0, 0]
    if rotation is None:
        rotation = [0, 0, 0]
    if visionrotation is None:
        visionrotation = [0, 0, 0]
    if position is None:
        position = [0, 0, 0]

    self = Sofa.Core.Node(name)
    mechanicalModel = self.addChild("MechanicalModel")
    mechanicalModel.addObject('MechanicalObject',
                              name='dofs',
                              size=1,
                              template='Rigid3d',
                              # showObject=True,
                              # showObjectScale=10,
                              translation=translation,
                              rotation=rotation,
                              position=position,
                              )
    # upperArmLongMechanicalModel.addObject('FixedConstraint')
    mechanicalModel.addObject('UniformMass', totalMass=0.1)

    # visual model
    visualModel = self.addChild('VisualModel')
    visualModel.addObject('MeshSTLLoader', name='loader', filename=filepath,
                          translation=visiontranslation,
                          rotation=visionrotation)
    visualModel.addObject('MeshTopology', src='@loader')
    visualModel.addObject('OglModel', name='renderer', color=color,
                          writeZTransparent=True)
    visualModel.addObject('RigidMapping',
                          input=mechanicalModel.dofs.getLinkPath(),
                          output=visualModel.renderer.getLinkPath(), )

    # collision model
    if file1path is not None:
        collisionmodel = self.addChild("CollisionModel1")
        collisionmodel.addObject('MeshSTLLoader', name="loader1", filename=file1path,
                                 rotation=visionrotation, translation=visiontranslation
                                 )
        collisionmodel.addObject('MeshTopology', src="@loader1")
        collisionmodel.addObject('MechanicalObject')
        # 传感器的碰撞组为1
        collisionmodel.addObject('PointCollisionModel', group=1)
        collisionmodel.addObject('LineCollisionModel', group=1)
        collisionmodel.addObject('TriangleCollisionModel', group=1)
        collisionmodel.addObject('RigidMapping',
                                 input=mechanicalModel.dofs.getLinkPath()
                                 )

    return self


def CreateSensor(name="Sensor", filepath='', rotation=None, translation=None,
                 collisionrotation=None, collisiontranslation=None,
                 color=None):
    if color is None:
        color = [0.5, 0.8, 0.1, 0.3]
    if translation is None:
        translation = [0., 0., 0.]
    if collisionrotation is None:
        collisionrotation = [0., 0., 0.]
    if collisiontranslation is None:
        collisiontranslation = [0., 0., 0.]
    self = Sofa.Core.Node(name)
    self.addObject('EulerImplicitSolver')
    # self.addObject('CGLinearSolver')
    self.addObject('SparseLDLSolver')
    # self.addObject('AsyncSparseLDLSolver')

    mechanicalModel = self.addChild("MechanicalModel")
    # upperArmLongMechanicalModel = upperArmLong.addChild("MechanicalModel")
    mechanicalModel.addObject('MechanicalObject',
                              name='dofs',
                              size=1,
                              template='Rigid3d',
                              # showObject=True,
                              # showObjectScale=10,
                              rotation=rotation,
                              translation=translation, )
    # upperArmLongMechanicalModel.addObject('FixedConstraint')
    mechanicalModel.addObject('UniformMass', totalMass=0.1)

    # collision model
    collisionmodel = self.addChild("CollisionModel")
    collisionmodel.addObject('MeshSTLLoader', name="loader", filename=filepath,
                             rotation=collisionrotation, translation=collisiontranslation
                             )
    collisionmodel.addObject('MeshTopology', src="@loader")
    collisionmodel.addObject('MechanicalObject')
    # 传感器的碰撞组为1
    # collisionmodel.addObject('PointCollisionModel', group=1)
    collisionmodel.addObject('LineCollisionModel', group=1)
    collisionmodel.addObject('TriangleCollisionModel', group=1)
    collisionmodel.addObject('RigidMapping',
                             input=mechanicalModel.dofs.getLinkPath()
                             )

    return self


class ServoMotor(Sofa.Prefab):
    '''A S90 servo motor

    This prefab is implementing a S90 servo motor.
    https://servodatabase.com/servo/towerpro/sg90

    The prefab ServoMotor is composed of:
    - a visual model
    - a mechanical model composed two rigids. One rigid is for the motor body
      while the other is implementing the servo rotating wheel.

    The prefab has the following parameters:
    - translation           to change default location of the servo (default [0.0,0.0,0.0])
    - rotation              to change default rotation of the servo (default [0.0,0.0,0.0,1])
    - scale                 to change default scale of the servo (default 1)
    - showServo             to control wether a visual model of the motor is added (default True)
    - showWheel             to control wether the rotation axis of the motor is displayed (default False)

    The prefab have the following property:
    - angle         use this to specify the angle of rotation of the servo motor
    - angleLimits   use this to set a min and max value for the servo angle rotation
    - position      use this to specify the position of the servo motor

    Example of use in a Sofa scene:

    def addScene(root):
        ...
        servo = ServoMotor(root)

        ## Direct access to the components
        servo.angle.value = 1.0
    '''
    prefabData = [
        {'name': 'name', 'type': 'string', 'help': 'Node name', 'default': 'ServoMotor'},
        {'name': 'rotation', 'type': 'Vec3d', 'help': 'Rotation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'translation', 'type': 'Vec3d', 'help': 'Translation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'scale3d', 'type': 'Vec3d', 'help': 'Scale 3d', 'default': [1.0, 1.0, 1.0]}]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):
        # 数据参数初始化
        self.addData(name='minAngle', group='S90Properties', help='min angle of rotation (in radians)', type='float',
                     value=0)
        self.addData(name='maxAngle', group='S90Properties', help='max angle of rotation (in radians)', type='float',
                     value=100)
        self.addData(name='angleIn', group='S90Properties', help='angle of rotation (in radians)', type='vector<float>',
                     value=[-0.43282318, 1.2, 1.6072389, 1.2])
        self.addData(name='armsList', group='S90Properties', help='The name of arms',
                     type='vector<string>',
                     value=['upperArm_Long', 'upperArm_Short', 'middleArm_Short', 'middleArm_Long', 'lowerArm_Long',
                            'lowerArm_Short'])
        self.addData(name='visionTranslation', group='S90Properties', help='visionTranslation of arms',
                     type='vector<Vec3d>',
                     value=[[0.0, 0.0, -30.58], [0.0, 0.0, -26.41],
                            [0.0, 0.0, -26.41], [0.0, 0.0, -30.58],
                            [0.0, -22.6, -30.58],[0.0, -22.6, -26.41],
                            ])
        self.addData(name='sensorRotation', group='S90Properties', help='Rotation of Sensor',
                     type='vector<Vec3d>',
                     value=[
                         [180.0, (np.pi / 2 + np.arcsin(2.25 / 30.5)) / np.pi * 180, 180.0],
                         [-90.0, (np.pi / 2 - np.arcsin(2.25 / 30.5)) / np.pi * 180, 0.0],
                         [0.0, (np.pi / 2 - np.arcsin(2.76 / 26.27)) / np.pi * 180, 0.0],
                         [-90.0, (np.pi / 2 - np.arcsin(2.76 / 26.27)) / np.pi * 180, 0.0],
                         [0.0, (-np.pi / 2 - np.arcsin(2.25 / 30.5)) / np.pi * 180, 180.0],
                         [90.0, (-np.pi / 2 - np.arcsin(2.25 / 30.5)) / np.pi * 180, 180.0],
                         [0.0, (-np.pi / 2 - np.arcsin(2.76 / 26.27)) / np.pi * 180, 180.0],
                         [90.0, (-np.pi / 2 - np.arcsin(2.76 / 26.27)) / np.pi * 180, 180.0],
                         [0.0, (np.pi / 2 - np.arcsin(2.25 / 30.5)) / np.pi * 180, 0.0],
                         [0.0, (np.pi / 2 - np.arcsin(2.76 / 26.27)) / np.pi * 180, 0.0],
                     ])
        self.addData(name='sensorCollisionTranslation', group='S90Properties', help='Rotation of Sensor',
                     type='vector<Vec3d>',
                     value=[
                         [5.67, -4.5 + 8, 3.05],
                         [5.67, -2.25 + 8, 4.3],
                         [6.67, -4.5 + 8, 0.29],
                         [6.77, 0.51 + 8, 4.3],
                         [0 - 24.83, -4.5 + 9, -5.3 - 0.21 + 11],
                         [0 - 24.83, -4.5 + 9, -4.3 - 0.21 + 9],
                         [6.77 - 26.27, -4.3 + 8.7, -0.51 - 2.76 + 6.5],
                         [6.77 - 26.27, 0.4 - 2.76 + 4.7, -4.5 + 9],
                         [5.67, -4.5 - 22.6 + 8, 3.05],
                         [6.67, -4.5 - 22.6 + 8, 0.29],
                     ])
        self.addData(name='posOnChildList', group='S90Properties', help='Rotation of Sensor', type='vector<Vec3d>',
                     value=[
                         [0., 0., -30.58],
                         [0., 0., -26.41],
                         [0., 11.3, -26.41],
                         [0., 0., -30.58],
                     ])


        # Articulation0
        angle = self.addChild('Articulation')
        angle.addObject('MechanicalObject', name='dofs', template='Vec1',
                        position=[[-0.43282318], [1.2], [1.6072389], [1.2]],
                        rest_position=self.getData('angleIn').getLinkPath(),
                        # rest_position=self.angleIn.value(),
                        )
        angle.addObject('RestShapeSpringsForceField', points=[0, 1, 2, 3], stiffness=1e11)
        angle.addObject('UniformMass', totalMass=0.01)

        # 关节轮 armWheel0 (非实体部分)
        armWheel = angle.addChild('ArmWheel')
        armWheel.addObject('MechanicalObject', name='dofs', template='Rigid3d',
                           position=[[0., 0., 0., 0., 0., 0., 1.], [0., 0., 0., 0., 0., 0., 1.],
                                     [0., 0., 0., 0., 0., 0., 1.], [0., 0., 0., 0., 0., 0., 1.],
                                     [0., 0., 0., 0., 0., 0., 1.]],
                           # showObjectScale=10,
                           # showObject=True,
                           translation=list(self.translation.value),
                           rotation=list(self.rotation.value),
                           scale3d=list(self.scale3d.value))
        armWheel.addObject('ArticulatedSystemMapping',
                           input1="@../dofs",
                           # input2='@../../Axis1/MechanicalModel/dofs',
                           output="@./dofs")

        angle.addObject('ArticulatedHierarchyContainer', printLog=False)
        articulationCenters = angle.addChild('ArticulationCenters')

        # 关节
        for i in range(0, 4, 1):
            articulationCenter = articulationCenters.addChild('ArticulationCenter' + str(i + 1))
            articulationCenter.addObject('ArticulationCenter', parentIndex=i, childIndex=i + 1,
                                         posOnParent=[0., 0., 0.], posOnChild=self.posOnChildList.value[i],
                                         articulationProcess=2,
                                         )
            articulations = articulationCenter.addChild('Articulations')
            articulations.addObject('Articulation', translation=False, rotation=True, rotationAxis=[0, 1, 0],
                                    articulationIndex=i)

        ##########
        # 轴1 Axis1
        # axis1 = self.addChild(
        #     CreateAxis(name='Axis1', filepath='data/Ass_robot/sofa_model/Axis_1.STL',
        #               translation=list(self.translation.value), rotation=list(self.rotation.value),
        #               position=[[11.01, -11.3, 0., 0., 0., 0., 1.]],
        #               # visiontranslation=[0.0, 0.0, -30.58]
        #               color=[0.15, 0.45, 0.75, 0.7],
        #               ))
        # axis1.MechanicalModel.addObject('FixedConstraint')
        # # 轴2 Axis2
        # axis2 = self.addChild(
        #     CreateAxis(name='Axis2', filepath='data/Ass_robot/sofa_model/Axis_2.STL',
        #               translation=list(self.translation.value), rotation=list(self.rotation.value),
        #               # position=[[11.01, -11.3, 0., 0., 0., 0., 1.]],
        #               # visiontranslation=[0.0, 0.0, -30.58]
        #               color=[0.15, 0.45, 0.75, 0.7],
        #               ))
        # axis2.MechanicalModel.addObject('FixedConstraint')

        # 机械臂部分 arms
        for i in range(0, 6, 1):
            arm = self.addChild(
                CreateArm(name=self.armsList.value[i],
                          filepath='data/Ass_robot/sofa_model/' + self.armsList.value[i] + '.STL',
                          file1path='data/Ass_robot/sofa_model/' + self.armsList.value[i] + '_p1.STL',
                          visiontranslation=self.visionTranslation.value[i], mappingIndex=1))
            # 1-1 2-2 3-3 4-4 5-1 5-2
            mappingindex = i + 1
            if mappingindex > 4:
                mappingindex -= 4
            arm.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
                                          input="@../../Articulation/ArmWheel/dofs",
                                          output="@./",
                                          index=mappingindex,  # input frame index,不能改
                                          )

        # 传感器部分 Sensors
        sensors = self.addChild("Sensors")
        for i in range(1, 11, 1):
            sensor = sensors.addChild(
                CreateSensor(name='Sensor' + str(i),
                             filepath='data/Ass_robot/sofa_model/sensor_plane.stl',
                             # translation=list(self.translation.value),
                             rotation=self.sensorRotation.value[i - 1],
                             collisiontranslation=self.sensorCollisionTranslation.value[i - 1],
                             # translation=[3.05, -4.5, -5.67]
                             ))
            mappingIndex = 0
            if i == 1 or i == 2:
                mappingIndex = 1
            elif i == 3 or i == 4:
                mappingIndex = 2
            elif i == 5 or i == 6:
                mappingIndex = 4
            elif i == 7 or i == 8:
                mappingIndex = 3
            elif i == 9:
                mappingIndex = 1
            elif i == 10:
                mappingIndex = 2
            sensor.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
                                             input="@../../../Articulation/ArmWheel/dofs",
                                             output="@./",
                                             index=mappingIndex,  # input frame index,不能改
                                             )


def createScene(rootNode):
    import math
    from splib3.animation import animate
    scene = Scene(rootNode, gravity=[0.0, 0.0, 0.0], dt=0.0001,
                  plugins=['SofaSparseSolver', 'SofaOpenglVisual', 'SofaSimpleFem', 'SofaDeformable', 'SofaEngine',
                           'SofaGraphComponent', 'SofaRigid', 'SoftRobots'],
                  iterative=False
                  )
    scene.addMainHeader()
    # Add ContactHeader
    # choice 1
    # ContactHeader(rootNode, alarmDistance=4, contactDistance=0.1, frictionCoef=0.2)
    # choice 2
    scene.addObject('DefaultPipeline')
    scene.addObject('BruteForceBroadPhase')
    scene.addObject('BVHNarrowPhase')
    frictionCoef = 0.8
    scene.addObject('RuleBasedContactManager', responseParams="mu=" + str(frictionCoef),
                    name='Response', response='FrictionContactConstraint')
    scene.addObject('LocalMinDistance',
                    alarmDistance=1, contactDistance=0.2,
                    angleCone=0.01)
    scene.addObject('FreeMotionAnimationLoop',parallelCollisionDetectionAndFreeMotion=True,parallelODESolving=True)
    scene.addObject('GenericConstraintSolver', tolerance=1e-7, maxIterations=2000,
                    computeConstraintForces=True,
                    multithreading=True
                    )

    scene.Simulation.addObject('GenericConstraintCorrection')
    # scene.Simulation.addObject('LinearSolverConstraintCorrection')

    scene.Settings.mouseButton.stiffness = 0.1
    scene.VisualStyle.displayFlags = "showBehavior showCollision"
    # scene.Modelling.addChild(ServoMotor(name="ServoMotor"))

    # simulation model
    scene.Simulation.addChild(
        Intestinev2(rotation=[90.0, 0.0, 0.0], translation=[5, 50, 28], color=[1.0, 1.0, 1.0, 0.5]))
    scene.Simulation.addChild(ServoMotor(name="Robot", translation=[0, 0, 1.5], rotation=[0, 0, 0]))
    # animate(animation, {'target': scene.Simulation.ServoMotor}, duration=10., mode='loop')
    # scene.Simulation.ServoMotor.Articulation.ServoWheel.dofs.showObject = True

    box1 = FixingBox(scene.Simulation,
                     scene.Simulation.Intestine.MechanicalModel,
                     name="box1",
                     translation=[5, 65, 28],
                     scale=[30., 30., 30.])
    box1.BoxROI.drawBoxes = True

    box2 = FixingBox(scene.Simulation,
                     scene.Simulation.Intestine.MechanicalModel,
                     name="box2",
                     translation=[5, -60, 28],
                     scale=[30., 30., 30.])
    box2.BoxROI.drawBoxes = True

    # scene.Simulation.addChild(scene.Modelling.ServoMotor.box1)
    scene.addObject(EmptyController(name='controller', ServoMotor=scene.Simulation.Robot, scene=scene,
                                    intestineCollision=scene.Simulation.Intestine.CollisionModel,
                                    intestineCollisionInner=scene.Simulation.Intestine.CollisionModel_inner,
                                    GenericConstraintSolver=scene.GenericConstraintSolver,
                                    Sensors=scene.Simulation.Robot.Sensors
                                    ))
    return scene


if __name__ == '__main__':
    # runSofa.exe路径
    path = "D:/Software_download/sofa_22/SOFA_robosoft2022_python-3.8_Windows/bin/runSofa"
    path1 = "F:/code_repo/sofa/build/bin/RelWithDebInfo/runSofa"
    # 使用Sofa运行该文件
    os.system(path1 + " " + sys.argv[0])
