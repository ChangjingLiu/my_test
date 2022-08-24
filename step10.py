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
from stlib3.physics.rigid import Floor, Cube
import numpy as np
from intestinev1 import Intestinev1

dirPath = os.path.dirname(os.path.abspath(__file__)) + '/'
from stlib3.visuals import VisualModel
from fixingbox import FixingBox


def Particles(name="Particles", rotation=None, translation=None, color=None):

    self = Sofa.Core.Node(name)

    self.addObject('EulerImplicitSolver')
    self.addObject('CGLinearSolver')

    # mechanicalmodel
    mechanicalModel = self.addChild("MechanicalModel")
    mechanicalModel.addObject('MechanicalObject',
                              name='dofs',
                              # position=mechanicalModel.loader.position.getLinkPath(),
                              # showObject=True,
                              # showObjectScale=5.0,
                              # template="Vec3d",
                              # template="CudaVec3f",  # cuda组件
                              )
    mechanicalModel.addObject('ParticleSource',
                              name="ParticlesSource",
                              radius=[0.1, 0.1, 0.1],
                              velocity=[0, -1, 0],
                              delay=0.5,
                              start=0,
                              stop=10,
                              center=[0.5, 1, 0.25,-1, 1, 0.5],
                              )
    mechanicalModel.addObject('UniformMass',
                              name="mass",
                              src="@ParticlesSource",
                              totalMass="1")

    mechanicalModel.addObject('SpatialGridContainer')
    mechanicalModel.addObject('SPHFluidForceField',
                              radius=0.1,
                              density=15,
                              kernelType=1,
                              viscosityType=2,
                              viscosity=10,
                              pressure=1000,
                              surfaceTension=-1000,
                              )

    visualmodel = self.addChild("VisualModel")
    visualmodel.addObject('OglModel')
    visualmodel.addObject('SPHFluidSurfaceMapping',
                          name="ParticlesMapping",
                          input="@../MechanicalModel/dofs",
                          output="@OglModel",
                          isoValue=0.5,
                          radius=0.1,
                          step=0.02
                          )
    visualmodel.addObject('OglGrid',
                             plane="Z",
                             size=10,
                             nbSubdiv=10
                             )
    visualmodel.addObject('OglLineAxis',
                          axis="x y z",
                          )


    collisionmodel = self.addChild("CollisionModel")
    collisionmodel.addObject('MechanicalObject')
    collisionmodel.addObject('PointCollisionModel')
    # collisionmodel.addObject('LineCollisionModel')
    # collisionmodel.addObject('TriangleCollisionModel')
    collisionmodel.addObject("PlaneForceField",
                             normal=[0,1,1],
                             d=0,
                             showPlane=1
                             )


    return self


def createScene(rootNode):
    scene = Scene(rootNode, gravity=[0.0, 0.0, 0.0], dt=0.01,
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
    scene.VisualStyle.displayFlags = "showBehavior showCollision"
    # scene.Modelling.addChild(ServoMotor(name="ServoMotor"))

    Floor(scene.Modelling,
          color=[1.0, 0.0, 0.0],
          translation=[0.0, -20.0, 0.0],
          rotation=[0., 0., 10.],
          isAStaticObject=True)

    # simulation model
    scene.Simulation.addChild(Particles())
    # scene.Simulation.addChild(ServoMotor(name="ServoMotor", translation=[0, 0, 0], rotation=[0, 0, 0]))
    # animate(animation, {'target': scene.Simulation.ServoMotor}, duration=10., mode='loop')
    # scene.Simulation.ServoMotor.Articulation.ServoWheel.dofs.showObject = True
    return scene


if __name__ == '__main__':
    # runSofa.exe路径
    path = "D:/Software_download/sofa_22/SOFA_robosoft2022_python-3.8_Windows/bin/runSofa"
    path1 = "F:/code_repo/sofa/build/bin/RelWithDebInfo/runSofa"
    # 使用Sofa运行该文件
    os.system(path1 + " " + sys.argv[0])
