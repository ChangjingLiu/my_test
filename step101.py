# -*- coding: utf-8 -*-
'''
Step 10:
在这一步中，实现了
1.粒子仿真与平板碰撞，用于仿真实现肠道容积物

'''

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
from stlib3.scene.contactheader import ContactHeader
from stlib3.physics.rigid import Floor, Cube
import numpy as np
from intestinev1 import Intestinev1

dirPath = os.path.dirname(os.path.abspath(__file__)) + '/'
from stlib3.visuals import VisualModel
from fixingbox import FixingBox


def Particles(name="Particles", rotation=None, translation=None, color=None):
    self = Sofa.Core.Node(name)

    # self.addObject("EulerExplicitSolver", symplectic=1)
    self.addObject('EulerImplicitSolver')
    self.addObject('CGLinearSolver', name='LinearSolver')
    mechanicalModel = self.addChild("MechanicalModel")
    mechanicalModel.addObject("MechanicalObject", name="MModel")
    mechanicalModel.addObject("RegularGridTopology",
                        nx=5, ny=40, nz=5,
                        xmin=-1.5, xmax=0,
                        ymin=-3, ymax=12,
                        zmin=-1.5, zmax=0,
                        drawEdges=1
                              )
    mechanicalModel.addObject("UniformMass", name="M1", vertexMass=1)
    mechanicalModel.addObject("SpatialGridContainer", cellWidth=0.75)
    mechanicalModel.addObject("SPHFluidForceField", radius=0.745,
                        density=15, kernelType=1, viscosityType=2,
                        viscosity=10, pressure=1000, surfaceTension=-1000)

    mechanicalModel.addObject("PlaneForceField", name='p1', normal=[1, 0, 0], d=-4, showPlane=1)
    mechanicalModel.addObject("PlaneForceField", name='p2', normal=[-1, 0, 0], d=-4, showPlane=1)
    mechanicalModel.addObject("PlaneForceField", name='p3', normal=[0.5, 1, 0.1], d=-4, showPlane=1)
    mechanicalModel.addObject("PlaneForceField", name='p4', normal=[0, 0, 1], d=-4, showPlane=1)
    mechanicalModel.addObject("PlaneForceField", name='p5', normal=[0, 0, -1], d=-4, showPlane=1)

    # 碰撞模型
    collisionModel = self.addChild("CollisionModel")
    collisionModel.addObject('MechanicalObject', template='Vec3d', )
    collisionModel.addObject("RegularGridTopology",
                              nx=5, ny=40, nz=5,
                              xmin=-1.5, xmax=0,
                              ymin=-3, ymax=12,
                              zmin=-1.5, zmax=0,
                              # drawEdges=1
                              )
    # 碰撞组为
    collisionModel.addObject('PointCollisionModel',
                             # selfCollision=True
                             )
    collisionModel.addObject('BarycentricMapping',
                             input=mechanicalModel.MModel.getLinkPath()
                             )
    return self


def createScene(rootNode):
    # rootNode.findData('gravity').value = [0.0, -10.0, 0.0];
    # rootNode.findData('dt').value = 0.01
    # rootNode.addObject('DefaultPipeline')
    # rootNode.addObject('DefaultAnimationLoop')
    #
    # rootNode.addObject('RequiredPlugin', name="SofaOpenglVisual")
    # rootNode.addObject('RequiredPlugin', name="SofaPython3")
    # rootNode.addObject('RequiredPlugin', name="SofaSphFluid")
    # rootNode.addObject('RequiredPlugin', name="SofaExplicitOdeSolver")
    # rootNode.addObject('RequiredPlugin', name="SofaBoundaryCondition")
    #
    # # Collision function
    #
    #
    # rootNode.addObject('GenericConstraintSolver', tolerance="1e-6", maxIterations="1000")
    # rootNode.addObject('BruteForceBroadPhase')
    # rootNode.addObject('BVHNarrowPhase')
    # rootNode.addObject('RuleBasedContactManager', responseParams="mu=" + str(0.0), name='Response')
    # rootNode.addObject('LocalMinDistance', alarmDistance=10, contactDistance=5, angleCone=0.01)
    # rootNode.addObject('GenericConstraintCorrection')
    # #
    # rootNode.addObject('VisualStyle')
    # rootNode.VisualStyle.displayFlags = "showBehaviorModels showForceFields showCollisionModels"
    #


    scene = Scene(rootNode, gravity=[0.0, -10.0, 0.0], dt=0.01,
                  plugins=['SofaSparseSolver', 'SofaOpenglVisual', 'SofaSimpleFem', 'SofaDeformable', 'SofaEngine',
                           'SofaGraphComponent', 'SofaRigid', 'SoftRobots',"SofaSphFluid","SofaExplicitOdeSolver"],
                  iterative=False
                  )
    scene.addMainHeader()
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
                    computeConstraintForces=True,
                    multithreading=True
                    )

    scene.Simulation.addObject('GenericConstraintCorrection')
    # scene.Simulation.addObject('LinearSolverConstraintCorrection')

    scene.Settings.mouseButton.stiffness = 0.1
    scene.VisualStyle.displayFlags = "showBehaviorModels showForceFields showCollisionModels"

    scene.Simulation.addChild(Particles())
    # Floor(scene.Modelling,
    #       color=[1.0, 0.0, 0.0],
    #       translation=[0.0, -10.0, 0.0],
    #       rotation=[0., 0., 10.],
    #       isAStaticObject=True)
    # rootNode.addChild(Particles())

    return rootNode


if __name__ == '__main__':
    # runSofa.exe路径
    path = "D:/Software_download/sofa_22/SOFA_robosoft2022_python-3.8_Windows/bin/runSofa"
    path1 = "F:/code_repo/sofa/build/bin/RelWithDebInfo/runSofa"
    # 使用Sofa运行该文件
    os.system(path1 + " " + sys.argv[0])
