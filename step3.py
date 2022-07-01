# -*- coding: utf-8 -*-
'''
Step 3:
在这一步中，实现了
1.root节点的控制器emptyController.py封装
2.在控制器下可获取接触碰撞面每个点的法向力
3.主要思路为通过root.GenericConstraintSolver.constraintForces获得每个约束的法向力，在root.Modelling.Cube.collision.MechanicalObject.constraint获得对象物体的约束力方向，
此外需要判断法向力方向
'''
import os
import sys
import Sofa
from stlib3.scene import Scene
from stlib3.scene.contactheader import ContactHeader
from stlib3.physics.rigid import Sphere
from stlib3.physics.rigid import Floor, Cube
from stlib3.physics.rigid import Cube
# A prefab to fix some part of an object at its rest position.
from fixingbox import FixingBox

#  A prefab to fix
from intestine import Intestine
from mycube import myCube

from emptyController import EmptyController


def createScene(rootNode):
    # 根节点设置 root configuration
    root = Scene(rootNode, gravity=[0.0, -9810, 0.0], dt=0.001,
                 plugins=['SofaSparseSolver', 'SofaOpenglVisual', 'SofaSimpleFem', 'SofaDeformable', 'SofaEngine',
                          'SofaGraphComponent', 'SofaRigid', 'SoftRobots'],
                 iterative=False)
    root.addMainHeader()

    # Add ContactHeader
    # choice 1
    # ContactHeader(rootNode, alarmDistance=4, contactDistance=0.1, frictionCoef=0.2)
    # choice 2
    root.addObject('DefaultPipeline')
    root.addObject('BruteForceBroadPhase')
    root.addObject('BVHNarrowPhase')
    frictionCoef = 0.5
    root.addObject('RuleBasedContactManager', responseParams="mu=" + str(frictionCoef),
                   name='Response', response='FrictionContactConstraint')
    root.addObject('LocalMinDistance',
                   alarmDistance=4, contactDistance=0.2,
                   angleCone=0.01)
    root.addObject('FreeMotionAnimationLoop')
    root.addObject('GenericConstraintSolver', tolerance=1e-6, maxIterations=1000,
                   computeConstraintForces=True, multithreading=True)
    # root.addObject('LCPConstraintSolver',tolerance="0.001", maxIt=1000)


    # 仿真节点设置 root.Simulation configuration
    root.Simulation.addObject('GenericConstraintCorrection')
    root.Simulation.TimeIntegrationSchema.rayleighStiffness = 0.01

    # 设置节点的设置 setting configuration
    root.Settings.mouseButton.stiffness = 10

    # 可视化设置 VisualStyle configuration
    root.VisualStyle.displayFlags = 'showBehavior'
    root.VisualStyle.displayFlags = "showCollisionModels"

    # 添加肠道模型 Add the intestine model
    # root.Modelling.addChild(Intestine(rotation=[90.0, 0.0, 0.0], color=[1.0, 1.0, 1.0, 0.5]))
    # 添加一个方块 Add Cube
    # scene.Modelling.addChild(
    #     myCube(translation=[0, 200, 0], rotation=[0.0, 0.0, 0.0], color=[1.0, 1.0, 1.0, 0.5], scale=2))
    Cube1 = Cube(root.Modelling, name="Cube",
                 translation=[0.0, 80.0, 0.0],
                 uniformScale=15.,
                 totalMass=0.1,
                 isAStaticObject=False, )
    Cube1.addObject('UncoupledConstraintCorrection')
    # root.Modelling.Cube.addObject('Monitor', template="Vec3d", name="123",listening="1", indices="0", showForces="1" )

    Floor(root.Modelling,
          color=[1.0, 0.0, 0.0],
          translation=[0.0, -160.0, 0.0],
          rotation=[0., 0., 10.],
          isAStaticObject=True)

    # Instanciating the FixingBox prefab into the graph, constraining the mechanical object of the ElasticBody.
    box1 = FixingBox(root.Modelling,
                     root.Modelling.Intestine.MechanicalModel,
                     name="box1",
                     translation=[0.0, 10.0, 0.0],
                     scale=[50., 50., 50.])
    box2 = FixingBox(root.Modelling,
                     root.Modelling.Intestine.MechanicalModel,
                     name="box2",
                     translation=[0.0, -60.0, 0.0],
                     scale=[50., 50., 50.])
    # # Changing the property of the Box ROI so that the constraint area appears on screen.
    box1.BoxROI.drawBoxes = True
    box2.BoxROI.drawBoxes = True

    # 添加Simulation模块
    # root.Simulation.addChild(root.Modelling.Intestine)
    root.Simulation.addChild(root.Modelling.Cube)
    root.Simulation.addChild(root.Modelling.box1)
    root.Simulation.addChild(root.Modelling.box2)

    # 添加全局控制器 Add Controller
    # 参数为节点
    root.addObject(EmptyController(node=root, GenericConstraintSolver=root.GenericConstraintSolver,
                                   cubeConstraint=root.Modelling.Cube.collision.MechanicalObject.constraint,
                                   Cube=root.Modelling.Cube))


if __name__ == '__main__':
    # runSofa.exe路径
    path = "D:/Software_download/sofa_22/SOFA_robosoft2022_python-3.8_Windows/bin/runSofa"
    # 使用Sofa运行该文件
    os.system(path + " " + sys.argv[0])
