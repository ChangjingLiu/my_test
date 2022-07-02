import os
import sys
import Sofa
from stlib3.physics.collision import CollisionMesh
from stlib3.scene import Scene
from fixingbox import FixingBox
def Intestine(name="Intestine", rotation=None, translation=None, color=None):
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
    mechanicalmodel.addObject('MeshGmshLoader',
                              name='loader',
                              rotation=rotation,
                              translation=translation,
                              filename='data/myBody.msh')
    mechanicalmodel.addObject('TetrahedronSetTopologyContainer',
                              src='@loader',
                              name='container')
    mechanicalmodel.addObject('MechanicalObject',
                              name='dofs',
                              position=mechanicalmodel.loader.position.getLinkPath(),
                              showObject=True,
                              showObjectScale=5.0)
    mechanicalmodel.addObject('UniformMass',
                              name="mass",
                              totalMass=10)
    # 有限元组件FEM ForceField components
    mechanicalmodel.addObject('TetrahedronFEMForceField',
                              name="linearElasticBehavior",
                              youngModulus=200,
                              poissonRatio=0.45)

    # Visual model 视觉模型用stl会好看些
    visualmodel = Sofa.Core.Node("VisualModel")
    # Specific loader for the visual model
    visualmodel.addObject('MeshSTLLoader',
                          name='loader',
                          filename='data/myBody.stl',
                          rotation=rotation,
                          translation=translation,
    )
    visualmodel.addObject('OglModel',
                          src=visualmodel.loader.getLinkPath(),
                          name='renderer',
                          color=[1.0, 1.0, 1.0, 0.5])
    self.addChild(visualmodel)

    visualmodel.addObject('BarycentricMapping',
                          input=mechanicalmodel.dofs.getLinkPath(),
                          output=visualmodel.renderer.getLinkPath())

    # 碰撞模型
    # Collision Object for the Cube
    self.addCollision=CollisionMesh(mechanicalmodel,
                  surfaceMeshFileName="data/myBody.stl", name="CollisionModel",
                                    rotation=rotation,translation=translation,
                  # collisionGroup=1
                                    )

    return self


def createScene(rootNode):
    scene = Scene(rootNode, gravity=[0.0, -9810, 0.0], plugins=['SofaSparseSolver', 'SofaOpenglVisual'],
                  iterative=False)
    scene.addMainHeader()
    scene.addObject('DefaultAnimationLoop')
    scene.addObject('DefaultVisualManagerLoop')
    scene.Simulation.addChild(Intestine())
    box1 = FixingBox(scene.Simulation,
                     scene.Simulation.Intestine.MechanicalModel,
                     name="box1",
                     translation=[0.0, 10.0, 0.0],
                     scale=[50., 50., 50.])
    box1.BoxROI.drawBoxes = True
    scene.Simulation.addChild(scene.Simulation.box1)

if __name__ == '__main__':
    # runSofa.exe路径
    path = "D:/Software_download/sofa_22/SOFA_robosoft2022_python-3.8_Windows/bin/runSofa"
    # 使用Sofa运行该文件
    os.system(path + " " + sys.argv[0])
