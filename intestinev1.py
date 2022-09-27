# -*- coding: utf-8 -*-
'''
intestinev1:
在这一步中，实现了
1.基于普通FEM的肠道物理模型，碰撞模型和视觉模型
为提高仿真速度，同时不牺牲过多的精度，可参考 https://github.com/sofa-framework/doc/blob/master/15_Using_SOFA/32_Performances/20_Improve_performances.md
为此有如下操作：
1.FEM模型降阶
2.将 TetrahedronFEMForceField 改为 TriangularFEMForceFieldOptim
3.减少碰撞类型
'''
import os
import sys
import Sofa
import copy
from stlib3.physics.collision import CollisionMesh
from stlib3.scene import Scene
from fixingbox import FixingBox


def Intestinev1(name="Intestine", rotation=None, translation=None, color=None):
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
                              filename='data/Intestine/IntestineV1-Tube.msh')
    mechanicalmodel.addObject('TetrahedronSetTopologyContainer',
                              src='@loader',
                              name='container')
    mechanicalmodel.addObject('MechanicalObject',
                              name='dofs',
                              template='Vec3d',
                              position=mechanicalmodel.loader.position.getLinkPath(),
                              showObject=True,
                              showObjectScale=5.0)

    mechanicalmodel.addObject('UniformMass',
                              name="mass",
                              totalMass=0.5)
    # 有限元组件FEM ForceField components
    mechanicalmodel.addObject(
        # 'TetrahedronFEMForceField',
        'TriangularFEMForceFieldOptim',
        # 'FastTetrahedralCorotationalForceField',
        name="linearElasticBehavior",
        youngModulus=500,
        poissonRatio=0.4)
    # mechanicalmodel.addObject("MeshSpringForceField",name="Springs",stiffness=10,damping=1 )

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
                          src=visualmodel.loader.getLinkPath(),
                          name='renderer',
                          color=[1.0, 1.0, 1.0, 0.5])
    self.addChild(visualmodel)

    visualmodel.addObject('BarycentricMapping',
                          input=mechanicalmodel.dofs.getLinkPath(),
                          output=visualmodel.renderer.getLinkPath())

    # 碰撞模型
    # Collision Object for the Cube
    # self.addCollision = CollisionMesh(mechanicalmodel,
    #                                   surfaceMeshFileName="data/Intestine/IntestineV1.stl", name="CollisionModel",
    #                                   rotation=rotation, translation=translation,
    #                                   # collisionGroup=1
    #                                   )
    # collision model
    collisionmodelInner = self.addChild("CollisionModel_inner")
    collisionmodelInner.addObject('MeshSTLLoader', name="loader",
                                  filename="data/Intestine/IntestineV1_inner_collision.stl",
                                  rotation=rotation, translation=translation
                                  )
    collisionmodelInner.addObject('MeshTopology', src="@loader")
    collisionmodelInner.addObject('MechanicalObject', src="@loader", template='Vec3d', )

    # 碰撞组为
    collisionmodelInner.addObject('PointCollisionModel',
                                  # selfCollision=True
                                  )
    collisionmodelInner.addObject('LineCollisionModel',
                                  # selfCollision=True
                                  )
    collisionmodelInner.addObject('TriangleCollisionModel',
                                  # selfCollision=True
                                  )
    collisionmodelInner.addObject('BarycentricMapping',
                                  input=mechanicalmodel.dofs.getLinkPath()
                                  )
    collisionmodelInner.addObject("SurfacePressureForceField", pressure=0,
                                  # pulseMode="true",
                                  # pressureSpeed=0.1
                                  )
    # collisionmodelInner.addObject('TaitSurfacePressureForceField', name='pressure',
    #                           gamma=5, B=1000, injectedVolume=5,
    #                           )

    c_translation = translation
    c_translation[1] = translation[1] - 25;
    collisionmodel = self.addChild("CollisionModel")
    collisionmodel.addObject('MeshSTLLoader', name="loader", filename="data/Intestine/IntestineV1_outer_collision.stl",
                             rotation=rotation, translation=translation
                             )
    collisionmodel.addObject('MeshTopology', src="@loader")
    collisionmodel.addObject('MechanicalObject', src="@loader", template='Vec3d', )

    # 碰撞组为
    collisionmodel.addObject('PointCollisionModel',
                             # selfCollision=True
                             )
    collisionmodel.addObject('LineCollisionModel',
                             # selfCollision=True
                             )
    collisionmodel.addObject('TriangleCollisionModel',
                             # selfCollision=True
                             )
    collisionmodel.addObject('BarycentricMapping',
                             input=mechanicalmodel.dofs.getLinkPath()
                             )
    collisionmodel.addObject("SurfacePressureForceField", pressure=0,
                             # pulseMode="true",
                             # pressureSpeed=0.1
                             )

    return self


def Intestinev2(name="Intestine", rotation=None, translation=None, color=None):
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
                              filename='data/Intestine/IntestineV1-Tube.msh')

    mechanicalmodel.addObject('TetrahedronSetTopologyContainer',
                              src='@loader',
                              name='container')

    mechanicalmodel.addObject('MechanicalObject',
                              name='dofs',
                              template='Vec3d',
                              position=mechanicalmodel.loader.position.getLinkPath(),
                              showObject=True,
                              showObjectScale=5.0)

    mechanicalmodel.addObject('UniformMass',
                              name="mass",
                              totalMass=0.5)
    # 有限元组件FEM ForceField components
    # mechanicalmodel.addObject(
    #     # 'TetrahedronFEMForceField',
    #     'TriangularFEMForceFieldOptim',
    #     # 'FastTetrahedralCorotationalForceField',
    #     name="linearElasticBehavior",
    #     youngModulus=500,
    #     poissonRatio=0.4)
    mechanicalmodel.addObject(
        # 'TetrahedronFEMForceField',
        # 'TriangularFEMForceFieldOptim',
        # 'FastTetrahedralCorotationalForceField',
        'TetrahedronHyperelasticityFEMForceField',
        name="HyperElasticMaterial",
        materialName="NeoHookean",
        AnisotropyDirections=[0,0,0],
        ParameterSet=[27104, 149900]
        # ParameterSet = [27104, 149900]
    )
    # mechanicalmodel.addObject("MeshSpringForceField",name="Springs",stiffness=10,damping=1 )

    # Visual model 视觉模型用stl会好看些
    # visualmodel = Sofa.Core.Node("VisualModel")
    # # Specific loader for the visual model
    # visualmodel.addObject('MeshSTLLoader',
    #                       name='loader',
    #                       filename='data/Intestine/IntestineV1.stl',
    #                       rotation=rotation,
    #                       translation=translation,
    #                       )
    # visualmodel.addObject('OglModel',
    #                       src=visualmodel.loader.getLinkPath(),
    #                       name='renderer',
    #                       color=[1.0, 1.0, 1.0, 0.5])
    # self.addChild(visualmodel)
    #
    # visualmodel.addObject('BarycentricMapping',
    #                       input=mechanicalmodel.dofs.getLinkPath(),
    #                       output=visualmodel.renderer.getLinkPath())

    # 碰撞模型
    # collision model
    translation_1 = copy.deepcopy(translation)
    translation_1[1]=translation_1[1]+0
    collisionmodelInner = self.addChild("CollisionModel_inner")
    collisionmodelInner.addObject('MeshSTLLoader', name="loader",
                                  filename="data/Intestine/IntestineV1_inner_collision.stl",
                                  rotation=rotation, translation=translation_1
                                  )
    collisionmodelInner.addObject('MeshTopology', src="@loader")
    collisionmodelInner.addObject('MechanicalObject', src="@loader", template='Vec3d', )

    # 碰撞组为
    # collisionmodelInner.addObject('PointCollisionModel',
    #                               # selfCollision=True
    #                               )
    # collisionmodelInner.addObject('LineCollisionModel',
    #                               # selfCollision=True
    #                               )
    # collisionmodelInner.addObject('TriangleCollisionModel',
    #                               # selfCollision=True
    #                               )
    collisionmodelInner.addObject('BarycentricMapping',
                                  input=mechanicalmodel.dofs.getLinkPath()
                                  )
    collisionmodelInner.addObject("SurfacePressureForceField", pressure=0,
                                  # pulseMode="true",
                                  # pressureSpeed=0.1
                                  mainDirection=[0,0,0],
                                  drawForceScale=0.0001,
                                  )
    collisionmodelInner.addObject("SurfacePressureForceField",
                                  name="pressure1",
                                  pressure=0,
                                  # pulseMode="true",
                                  # pressureSpeed=0.1
                                  mainDirection=[0, -1, 0],
                                  drawForceScale=0.0001,
                                  )
    # collisionmodelInner.addObject('TaitSurfacePressureForceField', name='pressure',
    #                           gamma=7, B=5000, injectedVolume=1000,
    #                           )

    c_translation = translation
    c_translation[1] = translation[1] - 25;
    collisionmodel = self.addChild("CollisionModel")
    collisionmodel.addObject('MeshSTLLoader', name="loader", filename="data/Intestine/IntestineV1_outer_collision.stl",
                             rotation=rotation, translation=translation
                             )
    collisionmodel.addObject('MeshTopology', src="@loader")
    collisionmodel.addObject('MechanicalObject', src="@loader", template='Vec3d', )

    # 碰撞组为
    collisionmodel.addObject('PointCollisionModel',
                             # selfCollision=True
                             )
    collisionmodel.addObject('LineCollisionModel',
                             # selfCollision=True
                             )
    collisionmodel.addObject('TriangleCollisionModel',
                             # selfCollision=True
                             )
    collisionmodel.addObject('BarycentricMapping',
                             input=mechanicalmodel.dofs.getLinkPath()
                             )
    collisionmodel.addObject("SurfacePressureForceField", pressure=0,
                             # pulseMode="true",
                             # pressureSpeed=0.1
                             mainDirection=[1, 0, 0],
                             )

    return self

def Intestinev3(name="Intestine", rotation=None, translation=None, color=None):
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
                              filename='data/Intestine/IntestineV3-Tube.msh')

    mechanicalmodel.addObject('TetrahedronSetTopologyContainer',
                              src='@loader',
                              name='container')

    mechanicalmodel.addObject('MechanicalObject',
                              name='dofs',
                              template='Vec3d',
                              position=mechanicalmodel.loader.position.getLinkPath(),
                              showObject=True,
                              showObjectScale=5.0)

    mechanicalmodel.addObject('UniformMass',
                              name="mass",
                              totalMass=0.5)

    mechanicalmodel.addObject(
        # 'TetrahedronFEMForceField',
        # 'TriangularFEMForceFieldOptim',
        # 'FastTetrahedralCorotationalForceField',
        'TetrahedronHyperelasticityFEMForceField',
        name="HyperElasticMaterial",
        materialName="NeoHookean",
        AnisotropyDirections=[0,0,0],
        ParameterSet=[27104, 149900]
    )

    # Visual model 视觉模型用stl会好看些
    # visualmodel = Sofa.Core.Node("VisualModel")
    # # Specific loader for the visual model
    # visualmodel.addObject('MeshSTLLoader',
    #                       name='loader',
    #                       filename='data/Intestine/IntestineV1.stl',
    #                       rotation=rotation,
    #                       translation=translation,
    #                       )
    # visualmodel.addObject('OglModel',
    #                       src=visualmodel.loader.getLinkPath(),
    #                       name='renderer',
    #                       color=[1.0, 1.0, 1.0, 0.5])
    # self.addChild(visualmodel)
    #
    # visualmodel.addObject('BarycentricMapping',
    #                       input=mechanicalmodel.dofs.getLinkPath(),
    #                       output=visualmodel.renderer.getLinkPath())

    # 碰撞模型
    # collision model
    collisionmodelInner = self.addChild("CollisionModel_inner")
    collisionmodelInner.addObject('MeshGmshLoader', name="loader",
                                  filename="data/Intestine/IntestineV3-Tube_inner.msh",
                                  rotation=rotation, translation=translation
                                  )
    collisionmodelInner.addObject('MeshTopology', src="@loader")
    collisionmodelInner.addObject('MechanicalObject', src="@loader", template='Vec3d', )

    # 碰撞组为
    collisionmodelInner.addObject('PointCollisionModel',
                                  # selfCollision=True
                                  )
    collisionmodelInner.addObject('LineCollisionModel',
                                  # selfCollision=True
                                  )
    collisionmodelInner.addObject('TriangleCollisionModel',
                                  # selfCollision=True
                                  )
    collisionmodelInner.addObject('BarycentricMapping',
                                  input=mechanicalmodel.dofs.getLinkPath()
                                  )
    collisionmodelInner.addObject("SurfacePressureForceField", pressure=0,
                                  # pulseMode="true",
                                  # pressureSpeed=0.1
                                  mainDirection=[0,0,0],
                                  drawForceScale=0.0001,
                                  )
    #

    collisionmodel = self.addChild("CollisionModel")
    collisionmodel.addObject('MeshGmshLoader', name="loader", filename="data/Intestine/IntestineV3-Tube_outer.msh",
                             rotation=rotation, translation=translation
                             )
    collisionmodel.addObject('MeshTopology', src="@loader")
    collisionmodel.addObject('MechanicalObject', src="@loader", template='Vec3d', )

    # 碰撞组为
    collisionmodel.addObject('PointCollisionModel',
                             # selfCollision=True
                             )
    collisionmodel.addObject('LineCollisionModel',
                             # selfCollision=True
                             )
    collisionmodel.addObject('TriangleCollisionModel',
                             # selfCollision=True
                             )
    collisionmodel.addObject('BarycentricMapping',
                             input=mechanicalmodel.dofs.getLinkPath()
                             )
    collisionmodel.addObject("SurfacePressureForceField", pressure=0,
                             # pulseMode="true",
                             # pressureSpeed=0.1
                             )

    return self

def Intestinev4(name="Intestine", rotation=None, translation=None, color=None):
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
                              filename='data/Intestine/IntestineV5-Tube.msh')

    mechanicalmodel.addObject('TetrahedronSetTopologyContainer',
                              src='@loader',
                              name='container')

    mechanicalmodel.addObject('MechanicalObject',
                              name='dofs',
                              template='Vec3d',
                              position=mechanicalmodel.loader.position.getLinkPath(),
                              showObject=True,
                              showObjectScale=5.0)

    mechanicalmodel.addObject('UniformMass',
                              name="mass",
                              totalMass=0.5)
    # 有限元组件FEM ForceField components
    # mechanicalmodel.addObject(
    #     # 'TetrahedronFEMForceField',
    #     'TriangularFEMForceFieldOptim',
    #     # 'FastTetrahedralCorotationalForceField',
    #     name="linearElasticBehavior",
    #     youngModulus=500,
    #     poissonRatio=0.4)
    mechanicalmodel.addObject(
        # 'TetrahedronFEMForceField',
        # 'TriangularFEMForceFieldOptim',
        # 'FastTetrahedralCorotationalForceField',
        'TetrahedronHyperelasticityFEMForceField',
        name="HyperElasticMaterial",
        materialName="NeoHookean",
        AnisotropyDirections=[0,0,0],
        ParameterSet=[27104, 149900]
    )
    # mechanicalmodel.addObject("MeshSpringForceField",name="Springs",stiffness=10,damping=1 )

    # Visual model 视觉模型用stl会好看些
    # visualmodel = Sofa.Core.Node("VisualModel")
    # # Specific loader for the visual model
    # visualmodel.addObject('MeshSTLLoader',
    #                       name='loader',
    #                       filename='data/Intestine/IntestineV4.stl',
    #                       rotation=rotation,
    #                       translation=translation,
    #                       )
    # visualmodel.addObject('OglModel',
    #                       src=visualmodel.loader.getLinkPath(),
    #                       name='renderer',
    #                       color=[1.0, 1.0, 1.0, 0.5])
    # self.addChild(visualmodel)
    #
    # visualmodel.addObject('BarycentricMapping',
    #                       input=mechanicalmodel.dofs.getLinkPath(),
    #                       output=visualmodel.renderer.getLinkPath())
    #
    # # 碰撞模型
    # # collision model
    translation_1 = copy.deepcopy(translation)
    translation_1[1]=translation_1[1]+0
    collisionmodelInner = self.addChild("CollisionModel_inner")
    collisionmodelInner.addObject('MeshSTLLoader', name="loader",
                                  filename="data/Intestine/IntestineV1_inner_collision.stl",
                                  rotation=rotation, translation=translation_1
                                  )
    collisionmodelInner.addObject('MeshTopology', src="@loader")
    collisionmodelInner.addObject('MechanicalObject', src="@loader", template='Vec3d', )

    # 碰撞组为
    # collisionmodelInner.addObject('PointCollisionModel',
    #                               # selfCollision=True
    #                               )
    # collisionmodelInner.addObject('LineCollisionModel',
    #                               # selfCollision=True
    #                               )
    # collisionmodelInner.addObject('TriangleCollisionModel',
    #                               # selfCollision=True
    #                               )
    collisionmodelInner.addObject('BarycentricMapping',
                                  input=mechanicalmodel.dofs.getLinkPath()
                                  )
    collisionmodelInner.addObject("SurfacePressureForceField", pressure=0,
                                  # pulseMode="true",
                                  # pressureSpeed=0.1
                                  mainDirection=[0,0,0],
                                  drawForceScale=0.0001,
                                  )
    # collisionmodelInner.addObject('TaitSurfacePressureForceField', name='pressure',
    #                           gamma=7, B=5000, injectedVolume=1000,
    #                           )

    c_translation = translation
    c_translation[1] = translation[1] - 25;
    collisionmodel = self.addChild("CollisionModel")
    collisionmodel.addObject('MeshSTLLoader', name="loader", filename="data/Intestine/IntestineV1_outer_collision.stl",
                             rotation=rotation, translation=translation
                             )
    collisionmodel.addObject('MeshTopology', src="@loader")
    collisionmodel.addObject('MechanicalObject', src="@loader", template='Vec3d', )

    # 碰撞组为
    collisionmodel.addObject('PointCollisionModel',
                             # selfCollision=True
                             )
    collisionmodel.addObject('LineCollisionModel',
                             # selfCollision=True
                             )
    collisionmodel.addObject('TriangleCollisionModel',
                             # selfCollision=True
                             )
    collisionmodel.addObject('BarycentricMapping',
                             input=mechanicalmodel.dofs.getLinkPath()
                             )
    collisionmodel.addObject("SurfacePressureForceField", pressure=0,
                             # pulseMode="true",
                             # pressureSpeed=0.1
                             )

    return self

def Intestinev5(name="Intestine", rotation=None, translation=None, color=None):
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
                              filename='data/Intestine/IntestineV5-Tube.msh')

    mechanicalmodel.addObject('TetrahedronSetTopologyContainer',
                              src='@loader',
                              name='container')

    mechanicalmodel.addObject('MechanicalObject',
                              name='dofs',
                              template='Vec3d',
                              position=mechanicalmodel.loader.position.getLinkPath(),
                              showObject=True,
                              showObjectScale=5.0)

    mechanicalmodel.addObject('UniformMass',
                              name="mass",
                              totalMass=10)
    # 有限元组件FEM ForceField components
    # mechanicalmodel.addObject(
    #     # 'TetrahedronFEMForceField',
    #     'TriangularFEMForceFieldOptim',
    #     # 'FastTetrahedralCorotationalForceField',
    #     name="linearElasticBehavior",
    #     youngModulus=500,
    #     poissonRatio=0.4)
    mechanicalmodel.addObject(
        # 'TetrahedronFEMForceField',
        # 'TriangularFEMForceFieldOptim',
        # 'FastTetrahedralCorotationalForceField',
        'TetrahedronHyperelasticityFEMForceField',
        name="HyperElasticMaterial",
        materialName="NeoHookean",
        AnisotropyDirections=[0,0,0],
        ParameterSet=[27104, 149900]
        # ParameterSet = [27104, 149900]
    )
    # mechanicalmodel.addObject("MeshSpringForceField",name="Springs",stiffness=10,damping=1 )

    # Visual model 视觉模型用stl会好看些
    # visualmodel = Sofa.Core.Node("VisualModel")
    # # Specific loader for the visual model
    # visualmodel.addObject('MeshSTLLoader',
    #                       name='loader',
    #                       filename='data/Intestine/IntestineV1.stl',
    #                       rotation=rotation,
    #                       translation=translation,
    #                       )
    # visualmodel.addObject('OglModel',
    #                       src=visualmodel.loader.getLinkPath(),
    #                       name='renderer',
    #                       color=[1.0, 1.0, 1.0, 0.5])
    # self.addChild(visualmodel)
    #
    # visualmodel.addObject('BarycentricMapping',
    #                       input=mechanicalmodel.dofs.getLinkPath(),
    #                       output=visualmodel.renderer.getLinkPath())

    # 碰撞模型
    # collision model
    translation_1 = copy.deepcopy(translation)
    translation_1[1]=translation_1[1]+0
    collisionmodelInner = self.addChild("CollisionModel_inner")
    collisionmodelInner.addObject('MeshSTLLoader', name="loader",
                                  filename="data/Intestine/IntestineV5_inner_collision.stl",
                                  rotation=rotation, translation=translation_1
                                  )
    collisionmodelInner.addObject('MeshTopology', src="@loader")
    collisionmodelInner.addObject('MechanicalObject', src="@loader", template='Vec3d', )

    # 碰撞组为
    # collisionmodelInner.addObject('PointCollisionModel',
    #                               # selfCollision=True
    #                               )
    # collisionmodelInner.addObject('LineCollisionModel',
    #                               # selfCollision=True
    #                               )
    # collisionmodelInner.addObject('TriangleCollisionModel',
    #                               # selfCollision=True
    #                               )
    collisionmodelInner.addObject('BarycentricMapping',
                                  input=mechanicalmodel.dofs.getLinkPath()
                                  )
    collisionmodelInner.addObject("SurfacePressureForceField", pressure=0,
                                  # pulseMode="true",
                                  # pressureSpeed=0.1
                                  mainDirection=[0,0,0],
                                  drawForceScale=0.0001,
                                  )
    collisionmodelInner.addObject("SurfacePressureForceField",
                                  name="pressure1",
                                  pressure=0,
                                  # pulseMode="true",
                                  # pressureSpeed=0.1
                                  mainDirection=[0, -1, 0],
                                  drawForceScale=0.0001,
                                  )
    # collisionmodelInner.addObject('TaitSurfacePressureForceField', name='pressure',
    #                           gamma=7, B=5000, injectedVolume=1000,
    #                           )

    c_translation = translation
    c_translation[1] = translation[1] - 25;
    collisionmodel = self.addChild("CollisionModel")
    collisionmodel.addObject('MeshSTLLoader', name="loader", filename="data/Intestine/IntestineV5_outer_collision.stl",
                             rotation=rotation, translation=translation
                             )
    collisionmodel.addObject('MeshTopology', src="@loader")
    collisionmodel.addObject('MechanicalObject', src="@loader", template='Vec3d', )

    # 碰撞组为
    collisionmodel.addObject('PointCollisionModel',
                             # selfCollision=True
                             )
    collisionmodel.addObject('LineCollisionModel',
                             # selfCollision=True
                             )
    collisionmodel.addObject('TriangleCollisionModel',
                             # selfCollision=True
                             )
    collisionmodel.addObject('BarycentricMapping',
                             input=mechanicalmodel.dofs.getLinkPath()
                             )
    collisionmodel.addObject("SurfacePressureForceField", pressure=0,
                             # pulseMode="true",
                             # pressureSpeed=0.1
                             mainDirection=[1, 0, 0],
                             )

    return self
def createScene(rootNode):
    scene = Scene(rootNode, gravity=[0.0, 0.0, 0.0], dt=0.0001,
                  plugins=['SofaSparseSolver', 'SofaOpenglVisual'],
                  iterative=False
                  )
    scene.addMainHeader()
    scene.addObject('DefaultAnimationLoop')
    scene.addObject('DefaultVisualManagerLoop')
    # scene.Settings.mouseButton.stiffness = 1
    scene.VisualStyle.displayFlags = "showBehavior showCollision"

    scene.Simulation.addChild(Intestinev2())
    box1 = FixingBox(scene.Simulation,
                     scene.Simulation.Intestine.MechanicalModel,
                     name="box1",
                     translation=[0.0, 10.0, 0.0],
                     scale=[50., 50., 50.])
    box2 = FixingBox(scene.Simulation,
                     scene.Simulation.Intestine.MechanicalModel,
                     name="box2",
                     translation=[0.0, -110.0, 0.0],
                     scale=[50., 50., 50.])

    box1.BoxROI.drawBoxes = True
    box2.BoxROI.drawBoxes = True
    scene.Simulation.addChild(scene.Simulation.box1)
    scene.Simulation.addChild(scene.Simulation.box2)


if __name__ == '__main__':
    # runSofa.exe路径
    path = "D:/Software_download/sofa_22/SOFA_robosoft2022_python-3.8_Windows/bin/runSofa"
    path1 = "/home/lcj/sofa/build/master/bin/runSofa"
    # 使用Sofa运行该文件
    os.system(path1 + " " + sys.argv[0])
