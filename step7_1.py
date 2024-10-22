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
from intestinev1 import Intestinev1,Intestinev2,Intestinev3

dirPath = os.path.dirname(os.path.abspath(__file__)) + '/'
from stlib3.visuals import VisualModel
from fixingbox import FixingBox


class EmptyController(Sofa.Core.Controller):

    # F_collies = np.zeros((8, 3))

    def __init__(self, *args, **kwargs):
        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        # 指针获取节点
        self.ServoMotor = kwargs["ServoMotor"]
        self.scene=kwargs["scene"]
        self.intestineCollision=kwargs["intestineCollision"]
        self.intestineCollisionInner = kwargs["intestineCollisionInner"]

        self.stepsize = 0.01
        self.steppressure = 5

    def onKeypressedEvent(self, event):
        key = event['key']
        # print(self.scene.gravity[1])
        if ord(key)==49:
            print("You pressed the 1 key")
            self.scene.gravity[1]=-9810
        if ord(key)==50:
            self.scene.gravity[1] =0
        if ord(key)==51:
            self.intestineCollision.SurfacePressureForceField.pressure.value=self.intestineCollision.SurfacePressureForceField.pressure.value+self.steppressure
            print(self.intestineCollision.SurfacePressureForceField.pressure.value)
            print("You pressed the 3 key")
        if ord(key)==52:
            self.intestineCollision.SurfacePressureForceField.pressure.value = self.intestineCollision.SurfacePressureForceField.pressure.value - self.steppressure
            print(self.intestineCollision.SurfacePressureForceField.pressure.value)
            print("You pressed the 4 key")
        if ord(key)==53:
            self.intestineCollisionInner.SurfacePressureForceField.pressure.value=self.intestineCollisionInner.SurfacePressureForceField.pressure.value+self.steppressure
            print(self.intestineCollisionInner.SurfacePressureForceField.pressure.value)
            print("You pressed the 5 key")
        if ord(key)==54:
            self.intestineCollisionInner.SurfacePressureForceField.pressure.value = self.intestineCollisionInner.SurfacePressureForceField.pressure.value - self.steppressure
            print(self.intestineCollisionInner.SurfacePressureForceField.pressure.value)
            print("You pressed the 6 key")
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
    # upperArmLongMechanicalModel = upperArmLong.addChild("MechanicalModel")
    mechanicalModel.addObject('MechanicalObject',
                              name='dofs',
                              size=1,
                              template='Rigid3d',
                              showObject=True,
                              showObjectScale=10,
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
def CreateArm(name="Intestine", filepath='', file1path=None,position=None, translation=None, rotation=None, visiontranslation=None,
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
    # upperArmLongMechanicalModel = upperArmLong.addChild("MechanicalModel")
    mechanicalModel.addObject('MechanicalObject',
                              name='dofs',
                              size=1,
                              template='Rigid3d',
                              showObject=True,
                              showObjectScale=10,
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
                              showObjectScale=10,
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
    properties = [
        {'name': 'name', 'type': 'string', 'help': 'Node name', 'default': 'ServoMotor'},
        {'name': 'rotation', 'type': 'Vec3d', 'help': 'Rotation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'translation', 'type': 'Vec3d', 'help': 'Translation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'scale3d', 'type': 'Vec3d', 'help': 'Scale 3d', 'default': [1.0, 1.0, 1.0]}]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):
        # The inputs
        self.addData(name='minAngle', group='S90Properties', help='min angle of rotation (in radians)', type='float',
                     value=0)
        self.addData(name='maxAngle', group='S90Properties', help='max angle of rotation (in radians)', type='float',
                     value=100)
        self.addData(name='angleIn', group='S90Properties', help='angle of rotation (in radians)', type='vector<float>',
                     value=[-0.43282318, 1.2, 1.6072389, 1.2])
        # self.addData(name='angleIn1', group='S90Properties', help='angle of rotation (in radians)', type='float',
        #              value=np.pi/3)

        # Articulation0

        angle = self.addChild('Articulation')
        angle.addObject('MechanicalObject', name='dofs', template='Vec1',
                        position=[[-0.43282318], [1.2], [1.6072389], [1.2]],
                        rest_position=self.getData('angleIn').getLinkPath(),
                        )
        angle.addObject('RestShapeSpringsForceField', points=[0, 1, 2, 3], stiffness=1e11)
        angle.addObject('UniformMass', totalMass=0.01)

        # 关节轮 armWheel0 (非实体部分)
        armWheel = angle.addChild('ArmWheel')
        armWheel.addObject('MechanicalObject', name='dofs', template='Rigid3',
                           position=[[0., 0., 0., 0., 0., 0., 1.], [0., 0., 0., 0., 0., 0., 1.],
                                     [0., 0., 0., 0., 0., 0., 1.], [0., 0., 0., 0., 0., 0., 1.],
                                     [0., 0., 0., 0., 0., 0., 1.]],
                           showObjectScale=10,
                           showObject=True,
                           translation=list(self.translation.value),
                           rotation=list(self.rotation.value),
                           scale3d=list(self.scale3d.value))
        armWheel.addObject('ArticulatedSystemMapping',
                           input1="@../dofs",
                           # input2='@../../Axis1/MechanicalModel/dofs',
                           output="@./dofs")

        angle.addObject('ArticulatedHierarchyContainer', printLog=False)
        articulationCenters = angle.addChild('ArticulationCenters')
        #
        articulationCenter0 = articulationCenters.addChild('ArticulationCenter0')
        articulationCenter0.addObject('ArticulationCenter', parentIndex=0, childIndex=1,
                                      posOnParent=[0., 0., 0.], posOnChild=[0., 0., -30.58],
                                      articulationProcess=2,
                                      )
        articulations0 = articulationCenter0.addChild('Articulations')
        articulations0.addObject('Articulation', translation=False, rotation=True, rotationAxis=[0, 1, 0],
                                 articulationIndex=0)
        #
        articulationCenter1 = articulationCenters.addChild('ArticulationCenter1')
        articulationCenter1.addObject('ArticulationCenter', parentIndex=1, childIndex=2,
                                      posOnParent=[0., 0., 0.], posOnChild=[0., 0., -26.41],
                                      articulationProcess=2,
                                      )
        articulations1 = articulationCenter1.addChild('Articulations')
        articulations1.addObject('Articulation', translation=False, rotation=True, rotationAxis=[0, 1, 0],
                                 articulationIndex=1)
        #
        articulationCenter2 = articulationCenters.addChild('ArticulationCenter2')
        articulationCenter2.addObject('ArticulationCenter', parentIndex=2, childIndex=3,
                                      posOnParent=[0., 0., 0.], posOnChild=[0., 11.3, -26.41],
                                      articulationProcess=2,
                                      )
        articulations2 = articulationCenter2.addChild('Articulations')
        articulations2.addObject('Articulation', translation=False, rotation=True, rotationAxis=[0, 1, 0],
                                 articulationIndex=2)
        #
        articulationCenter3 = articulationCenters.addChild('ArticulationCenter3')
        articulationCenter3.addObject('ArticulationCenter', parentIndex=3, childIndex=4,
                                      posOnParent=[0., 0., 0.], posOnChild=[0., 0., -30.58],
                                      articulationProcess=2,
                                      )
        articulations3 = articulationCenter3.addChild('Articulations')
        articulations3.addObject('Articulation', translation=False, rotation=True, rotationAxis=[0, 1, 0],
                                 articulationIndex=3)

        ##########
        # 轴1 Axis1
        axis1 = self.addChild(
            CreateAxis(name='Axis1', filepath='data/Ass_robot/sofa_model/Axis_1.STL',
                      translation=list(self.translation.value), rotation=list(self.rotation.value),
                      position=[[11.01, -11.3, 0., 0., 0., 0., 1.]],
                      # visiontranslation=[0.0, 0.0, -30.58]
                      color=[0.15, 0.45, 0.75, 0.7],
                      ))
        axis1.MechanicalModel.addObject('FixedConstraint')
        # 轴2 Axis2
        axis2 = self.addChild(
            CreateAxis(name='Axis2', filepath='data/Ass_robot/sofa_model/Axis_2.STL',
                      translation=list(self.translation.value), rotation=list(self.rotation.value),
                      # position=[[11.01, -11.3, 0., 0., 0., 0., 1.]],
                      # visiontranslation=[0.0, 0.0, -30.58]
                      color=[0.15, 0.45, 0.75, 0.7],
                      ))
        axis2.MechanicalModel.addObject('FixedConstraint')

        # 上臂长 upperArmLong
        upperArmLong = self.addChild(
            CreateArm(name='UpperArmLong', filepath='data/Ass_robot/sofa_model/upperArm_Long.STL',
                      file1path="data/Ass_robot/sofa_model/upperArm_Long_p1.STL",
                      visiontranslation=[0.0, 0.0, -30.58]))
        upperArmLong.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
                                               input="@../../Articulation/ArmWheel/dofs",
                                               output="@./",
                                               index=1,  # input frame index,不能改
                                               )

        # 上臂短 upperArmShort
        upperArmShort = self.addChild(
            CreateArm(name='UpperArmShort', filepath='data/Ass_robot/sofa_model/upperArm_Short.STL',
                      file1path="data/Ass_robot/sofa_model/upperArm_Short_p1.STL",
                      visiontranslation=[0.0, 0.0, -26.41]))
        upperArmShort.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
                                                input="@../../Articulation/ArmWheel/dofs",
                                                output="@./",
                                                index=2,  # input frame index,不能改
                                                )

        # 中臂短 middleArmShort
        middleArmShort = self.addChild(
            CreateArm(name='MiddleArmShort', filepath='data/Ass_robot/sofa_model/middleArm_Short.STL',
                      file1path="data/Ass_robot/sofa_model/middleArm_Short_p1.STL",
                      visiontranslation=[0.0, 0.0, -26.41]))
        middleArmShort.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
                                                 input="@../../Articulation/ArmWheel/dofs",
                                                 output="@./",
                                                 index=3,  # input frame index,不能改
                                                 )
        # 中臂长 middleArmLong
        middleArmLong = self.addChild(
            CreateArm(name='MiddleArmLong', filepath='data/Ass_robot/sofa_model/middleArm_Long.STL',
                      file1path="data/Ass_robot/sofa_model/middleArm_Long_p1.STL",
                      visiontranslation=[0.0, 0.0, -30.58]))
        middleArmLong.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
                                                input="@../../Articulation/ArmWheel/dofs",
                                                output="@./",
                                                index=4,  # input frame index,不能改
                                                )
        # 下臂长 lowerArmLong
        lowerArmLong = self.addChild(
            CreateArm(name='LowerArmLong', filepath='data/Ass_robot/sofa_model/lowerArm_Long.STL',
                      # file1path="data/Ass_robot/sofa_model/upperArm_Long_p1.STL",
                      visiontranslation=[0.0, -22.6, -30.58]))
        lowerArmLong.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
                                               input="@../../Articulation/ArmWheel/dofs",
                                               output="@./",
                                               index=1,  # input frame index,不能改
                                               )
        # 下臂短 lowerArmShort
        lowerArmShort = self.addChild(
            CreateArm(name='LowerArmShort', filepath='data/Ass_robot/sofa_model/lowerArm_Short.STL',
                      # file1path="data/Ass_robot/sofa_model/upperArm_Long_p1.STL",
                      visiontranslation=[0.0, -22.6, -26.41]))
        lowerArmShort.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
                                                input="@../../Articulation/ArmWheel/dofs",
                                                output="@./",
                                                index=2,  # input frame index,不能改
                                                )

        # 传感器部分
        # sensor in upperArmLong
        sensors = self.addChild("Sensors")
        sensor1 = sensors.addChild(CreateSensor(name='Sensor1', filepath='data/Ass_robot/sofa_model/sensor_plane.stl',
                                                rotation=[180.0, (np.pi / 2 + np.arcsin(2.25 / 30.5)) / np.pi * 180, 180.0],
                                                collisiontranslation=[5.67, -4.5+8, 3.05],
                                                # translation=[3.05, -4.5, -5.67]
                                                ))
        sensor1.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
                                          input="@../../../Articulation/ArmWheel/dofs",
                                          output="@./",
                                          index=1,  # input frame index,不能改
                                          )
        sensor2 = sensors.addChild(CreateSensor(name='Sensor2', filepath='data/Ass_robot/sofa_model/sensor_plane.stl',
                                                rotation=[-90.0, (np.pi / 2 - np.arcsin(2.25 / 30.5)) / np.pi * 180,
                                                          0.0],
                                                # translation=[0, 4.3, -5.67]),
                                                collisiontranslation=[5.67, -2.25+8, 4.3],
                                                ))
        sensor2.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
                                          input="@../../../Articulation/ArmWheel/dofs",
                                          output="@./",
                                          index=1,  # input frame index,不能改
                                          )
        # sensor in upperArmShort
        sensor3 = sensors.addChild(CreateSensor(name='Sensor3', filepath='data/Ass_robot/sofa_model/sensor_plane.stl',
                                                rotation=[0.0, (np.pi / 2 - np.arcsin(2.76 / 26.27)) / np.pi * 180,
                                                          0.0],
                                                collisiontranslation=[6.67, -4.5+8, 0.29],
                                                # translation=[3.05, -4.5, -5.67]
                                                ))
        sensor3.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
                                          input="@../../../Articulation/ArmWheel/dofs",
                                          output="@./",
                                          index=2,  # input frame index,不能改
                                          )
        sensor4 = sensors.addChild(CreateSensor(name='Sensor4', filepath='data/Ass_robot/sofa_model/sensor_plane.stl',
                                                rotation=[-90.0, (np.pi / 2 - np.arcsin(2.76 / 26.27)) / np.pi * 180,
                                                          0.0],
                                                # translation=[0, 4.3, -5.67]),
                                                collisiontranslation=[6.77, 0.51+8, 4.3],
                                                ))
        sensor4.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
                                          input="@../../../Articulation/ArmWheel/dofs",
                                          output="@./",
                                          index=2,  # input frame index,不能改
                                          )

        # sensor in middleArmShort



        sensor5 = sensors.addChild(CreateSensor(name='Sensor5', filepath='data/Ass_robot/sofa_model/sensor_plane.stl',
                                                rotation=[0.0, (-np.pi / 2 - np.arcsin(2.25 / 30.5)) / np.pi * 180,
                                                          180.0],
                                                collisiontranslation=[0 - 24.83, -4.5+9, -5.3 - 0.21+11],
                                                # translation=[3.05, -4.5, -5.67]
                                                ))
        sensor5.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
                                          input="@../../../Articulation/ArmWheel/dofs",
                                          output="@./",
                                          index=4,  # input frame index,不能改
                                          )
        sensor6 = sensors.addChild(CreateSensor(name='Sensor6', filepath='data/Ass_robot/sofa_model/sensor_plane.stl',
                                                rotation=[90.0, (-np.pi / 2 - np.arcsin(2.25 / 30.5)) / np.pi * 180,
                                                          180.0],
                                                # translation=[0, 4.3, -5.67]),
                                                collisiontranslation=[0 - 24.83, -4.5+9, -4.3 - 0.21+9],
                                                ))
        sensor6.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
                                          input="@../../../Articulation/ArmWheel/dofs",
                                          output="@./",
                                          index=4,  # input frame index,不能改
                                          )

        sensor7 = sensors.addChild(CreateSensor(name='Sensor7', filepath='data/Ass_robot/sofa_model/sensor_plane.stl',
                                                rotation=[0.0, (-np.pi / 2 - np.arcsin(2.76 / 26.27)) / np.pi * 180,
                                                          180.0],
                                                collisiontranslation=[6.77 - 26.27, -4.3 + 8.7,
                                                                      -0.51 - 2.76 + 6.5],
                                                # translation=[3.05, -4.5, -5.67]
                                                ))
        sensor7.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
                                          input="@../../../Articulation/ArmWheel/dofs",
                                          output="@./",
                                          index=3,  # input frame index,不能改
                                          )
        sensor8 = sensors.addChild(CreateSensor(name='Sensor8', filepath='data/Ass_robot/sofa_model/sensor_plane.stl',
                                                rotation=[90.0, (-np.pi / 2 - np.arcsin(2.76 / 26.27)) / np.pi * 180,
                                                          180.0],
                                                # translation=[0, 4.3, -5.67]),
                                                collisiontranslation=[6.77 - 26.27, 0.4 - 2.76 + 4.7, -4.5 + 9],
                                                ))
        sensor8.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
                                          input="@../../../Articulation/ArmWheel/dofs",
                                          output="@./",
                                          index=3,  # input frame index,不能改
                                          )

        # sensor in middleArmLong
        sensor9 = sensors.addChild(CreateSensor(name='Sensor9', filepath='data/Ass_robot/sofa_model/sensor_plane.stl',
                                                rotation=[0.0, (np.pi / 2 - np.arcsin(2.25 / 30.5)) / np.pi * 180, 0.0],
                                                collisiontranslation=[5.67, -4.5 - 22.6+8, 3.05],
                                                # translation=[3.05, -4.5, -5.67]
                                                ))
        sensor9.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
                                          input="@../../../Articulation/ArmWheel/dofs",
                                          output="@./",
                                          index=1,  # input frame index,不能改
                                          )

        sensor10 = sensors.addChild(CreateSensor(name='Sensor10', filepath='data/Ass_robot/sofa_model/sensor_plane.stl',
                                                rotation=[0.0, (np.pi / 2 - np.arcsin(2.76 / 26.27)) / np.pi * 180,
                                                          0.0],
                                                collisiontranslation=[6.67, -4.5 - 22.6+8, 0.29],
                                                # translation=[3.05, -4.5, -5.67]
                                                ))
        sensor10.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
                                          input="@../../../Articulation/ArmWheel/dofs",
                                          output="@./",
                                          index=2,  # input frame index,不能改
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
                    alarmDistance=1, contactDistance=0.4,
                    angleCone=0.01)
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('GenericConstraintSolver', tolerance=1e-6, maxIterations=1000,
                    computeConstraintForces=True,
                    multithreading=True
                    )

    scene.Simulation.addObject('GenericConstraintCorrection')
    # scene.Simulation.addObject('LinearSolverConstraintCorrection')

    scene.Settings.mouseButton.stiffness = 0.1
    scene.VisualStyle.displayFlags = "showBehavior showCollision"
    # scene.Modelling.addChild(ServoMotor(name="ServoMotor"))

    # simulation model
    scene.Simulation.addChild(Intestinev2(rotation=[90.0, 0.0, 0.0],translation=[5,50,28], color=[1.0, 1.0, 1.0, 0.5]))
    scene.Simulation.addChild(ServoMotor(name="ServoMotor", translation=[0, 0, 0], rotation=[0, 0, 0]))
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
    scene.addObject(EmptyController(name='controller', ServoMotor=scene.Simulation.ServoMotor,scene=scene,
                                    intestineCollision=scene.Simulation.Intestine.CollisionModel,
                                    intestineCollisionInner=scene.Simulation.Intestine.CollisionModel_inner
                                    ))
    return scene


if __name__ == '__main__':
    # runSofa.exe路径
    path = "D:/Software_download/sofa_22/SOFA_robosoft2022_python-3.8_Windows/bin/runSofa"
    path1 ="F:/code_repo/sofa/build/bin/RelWithDebInfo/runSofa"
    # 使用Sofa运行该文件
    os.system(path1 + " " + sys.argv[0])
