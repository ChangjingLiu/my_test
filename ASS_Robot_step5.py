import os, sys
import Sofa
from stlib3.scene import Scene
import numpy as np

dirPath = os.path.dirname(os.path.abspath(__file__)) + '/'
from stlib3.visuals import VisualModel
from fixingbox import FixingBox


def CreateArm(name="Intestine", filepath='', rotation=None, visiontranslation=None, color=None):
    if color is None:
        color=[0.5, 0.45, 0.75, 0.7]
    self = Sofa.Core.Node(name)
    mechanicalModel = self.addChild("MechanicalModel")
    # upperArmLongMechanicalModel = upperArmLong.addChild("MechanicalModel")
    mechanicalModel.addObject('MechanicalObject',
                              name='dofs',
                              size=1,
                              template='Rigid3d',
                              showObject=True,
                              showObjectScale=10,
                              translation=[0, 0, 0])
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
                          rotation=[0.0, 0.0, 0.0])
    visualModel.addObject('MeshTopology', src='@loader')
    visualModel.addObject('OglModel', name='renderer', color=color,
                          writeZTransparent=True)
    visualModel.addObject('RigidMapping',
                          input=mechanicalModel.dofs.getLinkPath(),
                          output=visualModel.renderer.getLinkPath(), )

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
                     value=-100)
        self.addData(name='maxAngle', group='S90Properties', help='max angle of rotation (in radians)', type='float',
                     value=100)
        self.addData(name='angleIn', group='S90Properties', help='angle of rotation (in radians)', type='float',
                     value=0)

        # Articulation0

        angle = self.addChild('Articulation')
        angle.addObject('MechanicalObject', name='dofs', template='Vec1', position=[[-np.pi/5], [0],[np.pi/2],[0],[np.pi]],
                        # rest_position=self.getData('angleIn').getLinkPath()
                        )
        # angle.addObject('RestShapeSpringsForceField', points=0, stiffness=1e9)
        angle.addObject('UniformMass', totalMass=0.01)

        # 关节轮 armWheel0 (非实体部分)
        armWheel = angle.addChild('ArmWheel')
        armWheel.addObject('MechanicalObject', name='dofs', template='Rigid3',
                           position=[[0., 0., 0., 0., 0., 0., 1.], [0., 0., 0., 0., 0., 0., 1.],
                                     [0., 0., 0., 0., 0., 0., 1.],[0., 0., 0., 0., 0., 0., 1.],
                                     [0., 0., 0., 0., 0., 0., 1.],[0., 0., 0., 0., 0., 0., 1.]],
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
                                      # globalPosition=[0,0,50],
                                      posOnParent=[0., 0., 0.], posOnChild=[0., 0, -10],
                                      articulationProcess=2,
                                      )
        articulations2 = articulationCenter2.addChild('Articulations')
        articulations2.addObject('Articulation', translation=False, rotation=True, rotationAxis=[0, 1, 0],
                                 articulationIndex=2)



        # #
        articulationCenter3 = articulationCenters.addChild('ArticulationCenter3')
        articulationCenter3.addObject('ArticulationCenter', parentIndex=0, childIndex=4,
                                      posOnParent=[0., 0., 0.], posOnChild=[0., 0., -30],
                                      # articulationProcess=2,
                                      )
        articulations3 = articulationCenter3.addChild('Articulations')
        articulations3.addObject('Articulation', translation=True, rotation=False, rotationAxis=[0, 0, 1],
                                 # translationAxis=[0,0,1],
                                 articulationIndex=3)

        articulationCenter4 = articulationCenters.addChild('ArticulationCenter4')
        articulationCenter4.addObject('ArticulationCenter', parentIndex=2, childIndex=3,
                                      posOnParent=[0., 0., 0.], posOnChild=[0., 0., 0],
                                      # articulationProcess=2,
                                      )
        articulations4 = articulationCenter4.addChild('Articulations')
        articulations4.addObject('Articulation', translation=False, rotation=True,
                                 rotationAxis=[0, 1, 0],
                                 # translationAxis=[0,0,1],
                                 articulationIndex=4)



        ##########

        # 轴2 Axis2
        axis1 = self.addChild('Axis1')
        axis1mechanicalModel = axis1.addChild("MechanicalModel")
        axis1mechanicalModel.addObject('MechanicalObject', name='dofs',
                                       template='Rigid3d',
                                       position=[[11.01, 0., 0., 0., 0., 0., 1.]],
                                       translation=list(self.translation.value),
                                       rotation=list(self.rotation.value),
                                       scale3d=list(self.scale3d.value), )
        axis1mechanicalModel.addObject('FixedConstraint')
        axis1mechanicalModel.addObject('UniformMass', name="mass", totalMass=0.1)

        axis1visualModel = axis1.addChild('VisualModel')
        axis1visualModel.addObject('MeshSTLLoader', name='loader', filename='data/Ass_robot/Axis_2.STL')
        axis1visualModel.addObject('MeshTopology', src='@loader')
        axis1visualModel.addObject('OglModel', name='renderer', color=[0.15, 0.45, 0.75, 0.7], writeZTransparent=True)
        axis1visualModel.addObject('RigidMapping',
                              input=axis1mechanicalModel.dofs.getLinkPath(),
                              output=axis1visualModel.renderer.getLinkPath(), )

        # 轴2 Axis2
        axis2 = self.addChild('Axis2')
        axis2mechanicalModel = axis2.addChild("MechanicalModel")
        axis2mechanicalModel.addObject('MechanicalObject', name='dofs',
                                       template='Rigid3d',
                                       position=[[0., 0., 0., 0., 0., 0., 1.]],
                                       translation=list(self.translation.value),
                                       rotation=list(self.rotation.value),
                                       scale3d=list(self.scale3d.value), )
        axis2mechanicalModel.addObject('FixedConstraint')
        axis2mechanicalModel.addObject('UniformMass', name="mass", totalMass=0.1)

        visualModel = axis2.addChild('VisualModel')
        visualModel.addObject('MeshSTLLoader', name='loader', filename='data/Ass_robot/Axis_2.STL')
        visualModel.addObject('MeshTopology', src='@loader')
        visualModel.addObject('OglModel', name='renderer', color=[0.15, 0.45, 0.75, 0.7], writeZTransparent=True)
        visualModel.addObject('RigidMapping',
                              input=axis2mechanicalModel.dofs.getLinkPath(),
                              output=visualModel.renderer.getLinkPath(), )

        # 上臂长 upperArmLong
        upperArmLong = self.addChild(CreateArm(name='UpperArmLong', filepath='data/Ass_robot/upperArm_Long.STL',
                                               visiontranslation=[0.0, 0.0, -30.58]))
        upperArmLong.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
                                                            input="@../../Articulation/ArmWheel/dofs",
                                                            output="@./",
                                                            index=1,  # input frame index,不能改
                                                            )


        # 上臂短 upperArmShort
        upperArmShort = self.addChild(CreateArm(name='UpperArmShort', filepath='data/Ass_robot/upperArm_Short.STL',
                                               visiontranslation=[0.0, 0.0, -26.41]))
        upperArmShort.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
                                               input="@../../Articulation/ArmWheel/dofs",
                                               output="@./",
                                               index=2,  # input frame index,不能改
                                               )

        # # 中臂短 middleArmShort
        # middleArmShort = self.addChild(CreateArm(name='MiddleArmShort', filepath='data/Ass_robot/middleArm_Short.STL',
        #                                         visiontranslation=[0.0, 0.0, -26.41]))
        # middleArmShort.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
        #                                         input="@../../Articulation/ArmWheel/dofs",
        #                                         output="@./",
        #                                         index=3,  # input frame index,不能改
        #                                         )
        # # 中臂长 middleArmLong
        # middleArmLong = self.addChild(CreateArm(name='MiddleArmLong', filepath='data/Ass_robot/middleArm_Long.STL',
        #                                          visiontranslation=[0.0, 0.0, -30.58]))
        # middleArmLong.MechanicalModel.addObject('RigidRigidMapping', name='mapping',
        #                                          input="@../../Articulation/ArmWheel/dofs",
        #                                          output="@./",
        #                                          index=4,  # input frame index,不能改
        #                                          )
        # 无效

def createScene(rootNode):
    import math
    from splib3.animation import animate

    def animation(target, factor):
        target.angleIn.value = math.cos(factor * 2 * math.pi)

    scene = Scene(rootNode, plugins=['SofaConstraint', 'SofaGeneralRigid', 'SofaOpenglVisual', 'SofaRigid',
                                     ], iterative=False)
    scene.addMainHeader()
    scene.addObject('DefaultVisualManagerLoop')
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('GenericConstraintSolver', maxIterations=1e3, tolerance=1e-5)
    scene.Simulation.addObject('GenericConstraintCorrection')
    scene.Settings.mouseButton.stiffness = 0.001
    scene.dt = 0.01
    scene.gravity = [0., -9810., 0.]
    scene.Modelling.addChild(ServoMotor(name="ServoMotor"))
    scene.Simulation.addChild(scene.Modelling.ServoMotor)
    # animate(animation, {'target': scene.Simulation.ServoMotor}, duration=10., mode='loop')
    # scene.Simulation.ServoMotor.Articulation.ServoWheel.dofs.showObject = True

    # box1 = FixingBox(scene.Modelling.ServoMotor,
    #                  scene.Modelling.ServoMotor.ServoBody.MechanicalModel,
    #                  name="box1",
    #                  translation=[0.0, 0.0, 0.0],
    #                  scale=[5., 5., 5.])
    # box1.BoxROI.drawBoxes = True
    # scene.Simulation.addChild(scene.Modelling.ServoMotor.box1)
    return scene


if __name__ == '__main__':
    # runSofa.exe路径
    path = "D:/Software_download/sofa_22/SOFA_robosoft2022_python-3.8_Windows/bin/runSofa"
    # 使用Sofa运行该文件
    os.system(path + " " + sys.argv[0])
