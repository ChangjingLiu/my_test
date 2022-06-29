import os
import sys
import Sofa
from stlib3.scene import Scene
from stlib3.physics.collision import CollisionMesh
dirPath = os.path.dirname(os.path.abspath(__file__)) + '/'


class ASSRobot(Sofa.Prefab):

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

        # Axis
        # mechanical model, index:0
        Axis2 = self.addChild('Axis2')
        Axis2.addObject('MechanicalObject', name='dofs', template='Rigid3',
                            position=[[0., 0., 0., 0., 0., 0., 1.]],
                            translation=[0.,0.,0.],
                            rotation=[0.,0.,0.],
                            scale3d=[1.,1.,1.])
        Axis2.addObject('FixedConstraint', indices=0)
        Axis2.addObject('UniformMass', totalMass=0.01)
        # visual model
        visual_Axis2 = Axis2.addChild('VisualModel')
        visual_Axis2.addObject('MeshSTLLoader', name='loader', filename='data/Ass_robot/Axis_2.STL')
        visual_Axis2.addObject('MeshTopology', src='@loader')
        visual_Axis2.addObject('OglModel', color=[0.15, 0.45, 0.75, 0.7], writeZTransparent=True)
        visual_Axis2.addObject('RigidMapping', index=0)

        # # Articulation:0->1
        # Articulation01 = self.addChild('Articulation01')
        # Articulation01.addObject('MechanicalObject', name='dofs',
        #                          template='Vec1', position=[[0]],
        #                 # rest_position=self.getData('angleIn').getLinkPath()
        #                          )
        # Articulation01.addObject('RestShapeSpringsForceField', points=0, stiffness=1e9)
        # Articulation01.addObject('UniformMass', totalMass=0.01)

        # angle0
        # 链接Axis2和upperArmLong
        angle0 = self.addChild('Articulation')
        angle0.addObject('MechanicalObject', name='dofs', template='Vec1', position=[[0]])
        angle0.addObject('RestShapeSpringsForceField', points=0, stiffness=1e9)
        angle0.addObject('UniformMass', totalMass=0.01)

        # upperArmLong
        upperArmLong = angle0.addChild('upperArmLong')
        upperArmLong.addObject('MechanicalObject', name='dofs', template='Rigid3',
                             position=[[0., 0., 0., 0., 0., 0., 1.], [0., 0., 0., 0., 0., 0., 1.]], showObjectScale=20,
                             translation=[0.,0.,0.], rotation=[0.,0.,0.],
                             scale3d=[1.,1.,1.])
        upperArmLong.addObject('ArticulatedSystemMapping', input1="@../dofs", input2="@../../Axis2/dofs", output="@./")




        # ArticulatedSystemMapping 建立系统的关节和全局运动的对应关系。
        # input1链接到包含关节自由度 (Vec1d) 的 MechanicalObject
        # output链接到 MechanicalObject，其中包含每个刚体 (Rigid3d) 的全局坐标系中的映射自由度，即关节系统的每个部分
        # angle0.addObject('ArticulatedSystemMapping', input1="@../dofs", input2="@../../Axis2/dofs",
        #                      output="@./")



        # # visual model
        # visual_upperArmLong = upperArmLong.addChild('VisualModel')
        # visual_upperArmLong.addObject('MeshSTLLoader', name='loader', filename='data/Ass_robot/upperArm_Long.STL')
        # visual_upperArmLong.addObject('MeshTopology', src='@loader')
        # visual_upperArmLong.addObject('OglModel',
        #                               color=[0.15, 0.45, 0.75, 0.7],
        #                               writeZTransparent=True)
        # visual_upperArmLong.addObject('RigidMapping', index=0)
        #
        # # # Collision Object for the Cube
        # # CollisionMesh(upperArmLong,
        # #               surfaceMeshFileName='data/Ass_robot/upperArm_Long.STL', name='CollisionModel', rotation=[0.0, 0.0, 0.0],
        # #               collisionGroup=1)
        # collisionmodel = upperArmLong.addChild('CollisionModel')
        # collisionmodel.addObject("MeshSTLLoader", name="loader", filename='data/Ass_robot/upperArm_Long.STL',
        #                          rotation=[0,0,0], translation=[0,0,0])
        # collisionmodel.addObject('MeshTopology', src="@loader")
        # collisionmodel.addObject('MechanicalObject')
        # collisionmodel.addObject('PointCollisionModel')
        # collisionmodel.addObject('LineCollisionModel')
        # collisionmodel.addObject('TriangleCollisionModel')
        # collisionmodel.addObject('RigidMapping')



        # articulationCenter
        articulationCenter = angle0.addChild('ArticulationCenter')
        articulationCenter.addObject('ArticulationCenter', parentIndex=0, childIndex=1, posOnParent=[0., 0., 0.],
                                     posOnChild=[0., 0., 0.])
        articulation = articulationCenter.addChild('Articulations')
        articulation.addObject('Articulation', translation=False, rotation=True, rotationAxis=[1, 0, 0],
                               articulationIndex=0)
        angle0.addObject('ArticulatedHierarchyContainer', printLog=False)




###########################################################################
        # Servo body
        # servoBody = self.addChild('ServoBody')
        # servoBody.addObject('MechanicalObject', name='dofs', template='Rigid3', position=[[0., 0., 0., 0., 0., 0., 1.]],
        #                             translation=list(self.translation.value),rotation=list(self.rotation.value),scale3d=list(self.scale3d.value))
        # servoBody.addObject('FixedConstraint', indices=0)
        # servoBody.addObject('UniformMass', totalMass=0.01)
        #
        # visual = servoBody.addChild('VisualModel')
        # visual.addObject('MeshSTLLoader', name='loader', filename=dirPath+'data/mesh/SG90_servomotor.stl')
        # visual.addObject('MeshTopology', src='@loader')
        # visual.addObject('OglModel', color=[0.15, 0.45, 0.75, 0.7], writeZTransparent=True)
        # visual.addObject('RigidMapping', index=0)

        # Servo wheel
        # angle = self.addChild('Articulation')
        # angle.addObject('MechanicalObject', name='dofs', template='Vec1', position=[[0]], rest_position=self.getData('angleIn').getLinkPath())
        # angle.addObject('RestShapeSpringsForceField', points=0, stiffness=1e9)
        # angle.addObject('UniformMass', totalMass=0.01)
        #
        # servoWheel = angle.addChild('ServoWheel')
        # servoWheel.addObject('MechanicalObject', name='dofs', template='Rigid3', position=[[0., 0., 0., 0., 0., 0., 1.],[0., 0., 0., 0., 0., 0., 1.]], showObjectScale=20,
        #                             translation=list(self.translation.value),rotation=list(self.rotation.value),scale3d=list(self.scale3d.value))
        # servoWheel.addObject('ArticulatedSystemMapping', input1="@../dofs", input2="@../../ServoBody/dofs", output="@./")
        #
        # articulationCenter = angle.addChild('ArticulationCenter')
        # articulationCenter.addObject('ArticulationCenter', parentIndex=0, childIndex=1, posOnParent=[0., 0., 0.], posOnChild=[0., 0., 0.])
        # articulation = articulationCenter.addChild('Articulations')
        # articulation.addObject('Articulation', translation=False, rotation=True, rotationAxis=[1, 0, 0], articulationIndex=0)
        # angle.addObject('ArticulatedHierarchyContainer', printLog=False)

        # The output
        # self.addData(name='angleOut', group='S90Properties', help='angle of rotation (in degree)', type='float', value=angle.dofs.getData('position').getLinkPath())


def createScene(rootNode):
    import math
    from splib3.animation import animate

    # def animation(target, factor):
    #     target.angleIn.value = math.cos(factor * 2 * math.pi)

    scene = Scene(rootNode, plugins=['SofaConstraint', 'SofaGeneralRigid', 'SofaOpenglVisual', 'SofaRigid'],
                  iterative=False)
    scene.addMainHeader()
    scene.addObject('DefaultVisualManagerLoop')
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('GenericConstraintSolver', maxIterations=1e3, tolerance=1e-5)
    scene.Simulation.addObject('GenericConstraintCorrection')

    scene.dt = 0.01
    scene.gravity = [0., -9810., 0.]
    scene.Modelling.addChild(ASSRobot(name="AssRobot"))
    scene.Simulation.addChild(scene.Modelling.AssRobot)
    # animate(animation, {'target': scene.Simulation.ServoMotor}, duration=10., mode='loop')
    # scene.Simulation.AssRobot.Articulation.upperArmLong.dofs.showObject = True

    return scene


if __name__ == '__main__':
    # runSofa.exe路径
    path = "D:/Software_download/sofa_22/SOFA_robosoft2022_python-3.8_Windows/bin/runSofa"
    # 使用Sofa运行该文件
    os.system(path + " " + sys.argv[0])
