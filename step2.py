# -*- coding: utf-8 -*-
'''
Step 2:
在这一步中，实现了
1.肠道模型的封装
2.添加了方块用于测试肠道的碰撞模型
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


def createScene(rootNode):
    scene = Scene(rootNode, gravity=[0.0, -9810, 0.0],
                  plugins=['SofaSparseSolver', 'SofaOpenglVisual', 'SofaSimpleFem', 'SofaDeformable', 'SofaEngine',
                           'SofaGraphComponent', 'SofaRigid', 'SoftRobots'],
                  iterative=False)
    ContactHeader(rootNode, alarmDistance=4, contactDistance=0.1, frictionCoef=0.2)
    scene.addMainHeader()
    # scene.addObject('DefaultAnimationLoop')
    # scene.addObject('DefaultVisualManagerLoop')

    scene.Simulation.addObject('GenericConstraintCorrection')
    scene.Simulation.TimeIntegrationSchema.rayleighStiffness = 0.005

    scene.Settings.mouseButton.stiffness = 10

    scene.VisualStyle.displayFlags = 'showBehavior'
    scene.VisualStyle.displayFlags = "showCollisionModels"
    scene.dt = 0.001

    # 添加肠道模型 Add the intestine model
    scene.Modelling.addChild(Intestine(rotation=[90.0, 0.0, 0.0], color=[1.0, 1.0, 1.0, 0.5]))
    # 添加一个方块
    # scene.Modelling.addChild(
    #     myCube(translation=[0, 200, 0], rotation=[0.0, 0.0, 0.0], color=[1.0, 1.0, 1.0, 0.5], scale=2))
    Cube1 = Cube(scene.Modelling, translation=[0.0, 80.0, 0.0],
                    uniformScale=13.,
                    totalMass=0.032,
                    isAStaticObject=False,)
    Cube1.addObject('UncoupledConstraintCorrection')

    Floor(scene.Modelling,
          color=[1.0, 0.0, 0.0],
          translation=[0.0, -160.0, 0.0],
          isAStaticObject=True)

    # Instanciating the FixingBox prefab into the graph, constraining the mechanical object of the ElasticBody.
    box1 = FixingBox(scene.Modelling,
                     scene.Modelling.Intestine.MechanicalModel,
                     name="box1",
                     translation=[0.0, 10.0, 0.0],
                     scale=[50., 50., 50.])
    box2 = FixingBox(scene.Modelling,
                     scene.Modelling.Intestine.MechanicalModel,
                     name="box2",
                     translation=[0.0, -60.0, 0.0],
                     scale=[50., 50., 50.])
    # # Changing the property of the Box ROI so that the constraint area appears on screen.
    box1.BoxROI.drawBoxes = True
    box2.BoxROI.drawBoxes = True

    # Simulation模块
    scene.Simulation.addChild(scene.Modelling.Intestine)
    scene.Simulation.addChild(scene.Modelling.Cube)
    scene.Simulation.addChild(scene.Modelling.box1)
    scene.Simulation.addChild(scene.Modelling.box2)




if __name__ == '__main__':
    # runSofa.exe路径
    path = "D:/Software_download/sofa_22/SOFA_robosoft2022_python-3.8_Windows/bin/runSofa"
    # 使用Sofa运行该文件
    os.system(path + " " + sys.argv[0])
