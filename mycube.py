import os
import sys
import Sofa
from stlib3.scene import Scene
from stlib3.physics.collision import CollisionMesh
from stlib3.scene import MainHeader, ContactHeader
from stlib3.physics.rigid import Floor


def myCube(name="MyCube", rotation=None, translation=None, color=None, scale=1):
    # 参数处理
    if color is None:
        color = [1.0, 1., 01.0, 1.0]
    if translation is None:
        translation = [0, 0, 0]
    if rotation is None:
        rotation = [0.0, 0.0, 0.0]

    # Mechanical model
    totalMass = 1.0
    volume = 1.0
    inertiaMatrix = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    self = Sofa.Core.Node(name)
    cube = self.addChild("Cube")

    cube.addObject('MechanicalObject', name="DOF", template="Rigid3", translation=translation,
                   rotation=rotation)
    cube.addObject('UniformMass', name="vertexMass", vertexMass=[totalMass, volume, inertiaMatrix[:]])

    # Material behaviour when submitted to constraints
    cube.addObject('UncoupledConstraintCorrection')

    # Time integration and solver

    cube.addObject('EulerImplicitSolver', name='odesolver')
    cube.addObject('CGLinearSolver', name='Solver')

    # Visual Object of the Cube

    visual = cube.addChild("CubeVisual")
    # Graphic model based on a mesh
    visual.addObject('MeshObjLoader', name="loader", filename="data/smCube27.obj", triangulate=True)
    visual.addObject('OglModel', name="Visual", src="@loader", color=[0.1, 0.0, 1.0], scale=scale)
    # Building a correspondence between the mechanical and the graphical representation
    visual.addObject('RigidMapping')

    # Collision Object for the Cube

    collision = cube.addChild("CubeCollisionModel")
    collision.addObject('MeshObjLoader', name="loader", filename="data/smCube27.obj", triangulate=True, scale=scale)

    collision.addObject('MeshTopology', src="@loader")
    collision.addObject('MechanicalObject')

    collision.addObject('TriangleCollisionModel')
    collision.addObject('LineCollisionModel')
    collision.addObject('PointCollisionModel')

    collision.addObject('RigidMapping')
    # CollisionMesh(cube,
    #               surfaceMeshFileName="data/smCube27.obj", name="CollisionModel", rotation=[0.0, 0.0, 0.0],
    #               collisionGroup=1)
    return self


def createScene(rootNode):
    scene = Scene(rootNode, gravity=[0.0, -9810, 0.0], plugins=['SofaSparseSolver', 'SofaOpenglVisual'],
                  iterative=False)
    scene.addMainHeader()
    scene.addObject('DefaultAnimationLoop')
    scene.addObject('DefaultVisualManagerLoop')
    scene.Simulation.addChild(myCube())


if __name__ == '__main__':
    # runSofa.exe路径
    path = "D:/Software_download/sofa_22/SOFA_robosoft2022_python-3.8_Windows/bin/runSofa"
    # 使用Sofa运行该文件
    os.system(path + " " + sys.argv[0])
