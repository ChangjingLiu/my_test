# -*- coding: utf-8 -*-
'''
Step 7:
在这一步中，实现了
1.重建更细致的肠道模型
2.超粘性弹性体
3.加压
4.加入机器人模型
'''
import os
import sys
import Sofa
from stlib3.physics.collision import CollisionMesh
from stlib3.scene import Scene
# from fixingbox import FixingBox
from splib3.numerics import getOrientedBoxFromTransform
from stlib3.physics.rigid import Floor, Cube


def FixingBox(parent, target, name='FixingBox',
              translation=None, eulerRotation=None, scale=None):
    '''Fix a set of 'dofs' according to a translation & orientation'''

    if scale is None:
        scale = [1.0, 1.0, 1.0]
    if eulerRotation is None:
        eulerRotation = [0.0, 0.0, 0.0]
    if translation is None:
        translation = [0.0, 0.0, 0.0]
    ob = getOrientedBoxFromTransform(translation=translation, eulerRotation=eulerRotation, scale=scale)

    self = parent.addChild(name)
    self.addObject('BoxROI',
                   orientedBox=ob,
                   name='BoxROI',
                   position=target.dofs.getData('rest_position').getLinkPath(),
                   drawBoxes=False,
                   # template="CudaVec3f"
                   )

    c = self.addChild('Constraint')
    target.addChild(c)

    c.addObject('RestShapeSpringsForceField',
                # template="CudaVec3f",  # cuda组件
                points=self.BoxROI.getData('indices').getLinkPath(),
                stiffness=1e12)
    return self


def Intestine_linux_cuda(name="Intestine", rotation=None, translation=None, color=None):
    # 为了建立肠道模型，我们需要 To simulate an intestine object, we need:
    # 1.变形定律（暂使用线性弹性体）a deformation law (here linear elasticity)
    # 2.求解方法（使用FRM）a solving method (here FEM)
    # 3.当我们使用FEM时，我们需要空间离散化（这里是四面体） as we are using FEM we need a space discretization (here tetrahedron)

    # 参数处理
    if color is None:
        color = [1.0, 1., 01.0, 1.0]
    if translation is None:
        translation = [0, 0, 0]
    if rotation is None:
        rotation = [90.0, 0.0, 0.0]

    self = Sofa.Core.Node(name)

    mechanicalmodel = self.addChild("MechanicalModel")
    # mechanicalmodel.addObject('EulerImplicitSolver')
    # mechanicalmodel.addObject('CGLinearSolver')
    mechanicalmodel.addObject('MeshGmshLoader',
                              name='loader',
                              rotation=rotation,
                              translation=translation,
                              filename='data/Intestine/IntestineV1.msh')
    mechanicalmodel.addObject('MechanicalObject',
                              name='dofs',
                              position=mechanicalmodel.loader.position.getLinkPath(),
                              showObject=True,
                              showObjectScale=5.0,
                              template="Vec3d",
                              # template="CudaVec3f",  # cuda组件
                              )
    mechanicalmodel.addObject('TetrahedronSetTopologyContainer',
                              template="Vec3d",
                              src='@loader',
                              name='container')

    mechanicalmodel.addObject('UniformMass',
                              # template="CudaVec3f",
                              name="mass",
                              totalMass=5,
                              )

    # 有限元组件FEM ForceField components
    mechanicalmodel.addObject('TetrahedronSetGeometryAlgorithms',
                              template="CudaVec3f",  # cuda组件
                              )
    mechanicalmodel.addObject('TetrahedronFEMForceField',
                              # template="Vec3d",
                              template="CudaVec3f",
                              name="linearElasticBehavior",
                              youngModulus=500,
                              poissonRatio=0.4)
    # mechanicalmodel.addObject("MeshSpringForceField",name="Springs",stiffness=10,damping=1 )
    # mechanicalmodel.addObject("LinearSolverConstraintCorrection")
    mechanicalmodel.addObject("UncoupledConstraintCorrection")


    # Visual model 视觉模型用stl会好看些
    visualmodel = Sofa.Core.Node("VisualModel")
    # Specific loader for the visual model
    visualmodel.addObject('MeshSTLLoader',
                          name='loader',
                          filename='data/Intestine/IntestineV1.stl',
                          rotation=rotation,
                          translation=translation,
                          )
    visualmodel.addObject('OglModel',
                          # template="CudaVec3f",
                          src=visualmodel.loader.getLinkPath(),
                          name='renderer',
                          color=[1.0, 1.0, 1.0, 0.5])
    self.addChild(visualmodel)

    visualmodel.addObject('BarycentricMapping',
                          input=mechanicalmodel.dofs.getLinkPath(),
                          output=visualmodel.renderer.getLinkPath())
    # 碰撞模型
    # Collision Object for the Cube
    collisionmodel = self.addChild("CollisionModel")
    # translation[1]=translation[1]-40
    collisionmodel.addObject('MeshSTLLoader', name="loader", filename="data/Intestine/IntestineV1.stl",
                             rotation=rotation, translation=translation
                             )
    collisionmodel.addObject('MeshTopology', src="@loader")
    collisionmodel.addObject('MechanicalObject', src="@loader",
                             # template="CudaVec3f",
                             position="@[-1].position"
                             )

    # 碰撞组为
    collisionmodel.addObject('PointCollisionModel')
    collisionmodel.addObject('LineCollisionModel')
    collisionmodel.addObject('TriangleCollisionModel')
    collisionmodel.addObject("SphereCollisionModel",name = "CollisionModel",
    listRadius = "@[-2].listRadius",
    )
    collisionmodel.addObject('BarycentricMapping',
                             input=mechanicalmodel.dofs.getLinkPath()
                             )
    collisionmodel.addObject("SurfacePressureForceField", pressure=0,
                             # pulseMode="true",
                             # pressureSpeed=0.1
                             # template="CudaVec3f",
                             )
    # collisionmodel.addObject("LinearSolverConstraintCorrection")
    collisionmodel.addObject("UncoupledConstraintCorrection")
    # self.addObject("LinearSolverConstraintCorrection")
    return self


def createScene(rootNode):
    scene = Scene(rootNode, gravity=[0.0, -9810.0, 0.0], dt=0.001,
                  plugins=['SofaSparseSolver', 'SofaOpenglVisual', 'SofaSimpleFem', 'SofaDeformable', 'SofaEngine',
                           'SofaGraphComponent', 'SofaRigid', 'SoftRobots'],
                  # iterative=False
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
                    alarmDistance=2, contactDistance=0.1,
                    angleCone=0.01)
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('GenericConstraintSolver', tolerance=1e-6, maxIterations=1000,
                    computeConstraintForces=True, multithreading=True)
    # scene.Simulation.addObject("GenericConstraintCorrection")
    # scene.Simulation.addObject("LinearSolverConstraintCorrection")

    # scene.Settings.mouseButton.stiffness = 1
    scene.VisualStyle.displayFlags = "showBehavior showCollision"

    scene.Simulation.addChild(Intestine_linux_cuda())
    Floor1=Floor(scene.Modelling,
          color=[1.0, 0.0, 0.0],
          translation=[0.0, -160.0, 0.0],
          rotation=[0., 0., 10.],
          isAStaticObject=True)
    box1 = FixingBox(scene.Simulation,
                     scene.Simulation.Intestine.MechanicalModel,
                     name="box1",
                     translation=[0.0, 10.0, 0.0],
                     scale=[50., 50., 50.])
    # box2 = FixingBox(scene.Simulation,
    #                  scene.Simulation.Intestine.MechanicalModel,
    #                  name="box2",
    #                  translation=[0.0, -110.0, 0.0],
    #                  scale=[50., 50., 50.])

    box1.BoxROI.drawBoxes = True
    # box2.BoxROI.drawBoxes = True
    # scene.Simulation.addChild(scene.Modelling.Floor)
    scene.Simulation.addChild(scene.Simulation.box1)
    # scene.Simulation.addChild(scene.Simulation.box2)


if __name__ == '__main__':
    # runSofa.exe路径
    path = "/home/lcj/sofa/build/master/bin/runSofa"
    path1 = "F:/code_repo/sofa/build/bin/RelWithDebInfo/runSofa"
    # 使用Sofa运行该文件
    os.system(path + " " + sys.argv[0])
