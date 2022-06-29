import Sofa
import os, sys
from stlib3.scene import Scene, MainHeader, ContactHeader
from fixingbox import FixingBox
from stlib3.physics.collision import CollisionMesh


def createScene(rootNode):
    # Setting the gravity, assuming the length unit is in millimeters
    scene = Scene(rootNode, gravity=[0.0, -9810, 0.0], plugins=['SofaSparseSolver', 'SofaOpenglVisual'],
                  iterative=False)

    # Collision handling built-in function (already used in Step 1)
    ContactHeader(rootNode, alarmDistance=10, contactDistance=5)

    scene.addMainHeader()
    scene.addObject('DefaultAnimationLoop')
    scene.addObject('DefaultVisualManagerLoop')

    # Setting the timestep in seconds
    rootNode.dt = 0.01

    # It is possible to visualize the 'forcefields' by doing
    scene.VisualStyle.displayFlags = 'showForceFields'

    # Tool to load the mesh file of the silicone piece. It will be used for both the mechanical and the visual models.
    # scene.Modelling.addObject('MeshVTKLoader', name='loader', filename='data/myBody.vtk')

    ##############################################
    # 模型
    # Basic mechanical modelling of the silicone piece
    elasticbody = scene.Modelling.addChild('ElasticBody')
    elasticbody.addObject('MeshGmshLoader',
                          name='loader',
                          filename='data/myBody.msh')
    elasticbody.addObject('TetrahedronSetTopologyContainer',
                          src='@loader',
                          name='tetras')

    # Time integration and solver
    elasticbody.addObject('EulerImplicit', name='odesolver')
    elasticbody.addObject('CGLinearSolver', name='Solver', iterations=25, tolerance=1e-05, threshold=1e-05)
    ##############################################################
    # 物理模型
    # mechanicalmodel
    mechanicalmodel = elasticbody.addChild("MechanicalModel")
    mechanicalmodel.addObject('MechanicalObject',
                              name='dofs',
                              position=elasticbody.loader.position.getLinkPath(),
                              rotation=[90.0, 0.0, 0.0],
                              showObject=True,
                              showObjectScale=5.0)
    mechanicalmodel.addObject('UniformMass',
                              name="mass",
                              totalMass=10)
    # ForceField components
    mechanicalmodel.addObject('TetrahedronFEMForceField',
                              name="linearElasticBehavior",
                              youngModulus=200,
                              poissonRatio=0.45)

    # Material behaviour when submitted to constraints
    mechanicalmodel.addObject('UncoupledConstraintCorrection')

    #####################################################
    # 视觉模型
    # Visual model 视觉模型用stl会好看些
    visual = Sofa.Core.Node("VisualModel")
    # Specific loader for the visual model
    visual.addObject('MeshSTLLoader',
                     name='loader',
                     filename='data/myBody.stl',
                     rotation=[90.0, 0.0, 0])
    visual.addObject('OglModel',
                     src=visual.loader.getLinkPath(),
                     name='renderer',
                     color=[1.0, 1.0, 1.0, 0.5])
    scene.Modelling.ElasticBody.addChild(visual)

    visual.addObject('BarycentricMapping',
                     input=mechanicalmodel.dofs.getLinkPath(),
                     output=visual.renderer.getLinkPath())

    #################################
    # 碰撞模型
    # Collision Object for the Cube
    CollisionMesh(mechanicalmodel,
                  surfaceMeshFileName="data/myBody.stl", name="CollisionModel", rotation=[90.0, 0.0, 0.0],
                  collisionGroup=1)

    # Instanciating the FixingBox prefab into the graph, constraining the mechanical object of the ElasticBody.
    box1 = FixingBox(scene.Modelling,
                     scene.Modelling.ElasticBody.MechanicalModel,
                     name="box1",
                     translation=[0.0, 60.0, 0.0],
                     scale=[50., 50., 50.])
    box2 = FixingBox(scene.Modelling,
                     scene.Modelling.ElasticBody.MechanicalModel,
                     name="box2",
                     translation=[0.0, -60.0, 0.0],
                     scale=[50., 50., 50.])
    # Changing the property of the Box ROI so that the constraint area appears on screen.
    box1.BoxROI.drawBoxes = True
    box2.BoxROI.drawBoxes = True

    scene.Simulation.addChild(elasticbody)
    scene.Simulation.addChild(scene.Modelling.box1)
    scene.Simulation.addChild(scene.Modelling.box2)
    return rootNode


if __name__ == '__main__':
    # runSofa.exe路径
    path = "D:/Software_download/sofa_22/SOFA_robosoft2022_python-3.8_Windows/bin/runSofa"
    # 使用Sofa运行该文件
    os.system(path + " " + sys.argv[0])
