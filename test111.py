import Sofa
import Sofa.Gui
import SofaRuntime
import matplotlib.pyplot as plt

USE_GUI = False


def main():
    root = Sofa.Core.Node('root')
    createScene(root)
    Sofa.Simulation.init(root)
    plt.figure()

    if not USE_GUI:
        while root.getTime() < 5:  # This is done to be sure that the results will not explode
            Sofa.Simulation.animate(root, root.dt.value)
    else:
        Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
        Sofa.Gui.GUIManager.createGUI(root, __file__)
        Sofa.Gui.GUIManager.SetDimension(1080, 1080)
        Sofa.Gui.GUIManager.MainLoop(root)
        Sofa.Gui.GUIManager.closeGUI()


def createScene(root):
    root.addObject('RequiredPlugin', name='SofaSparseSolver')
    root.addObject('RequiredPlugin', name='SofaDeformable')
    root.addObject('RequiredPlugin', name='SofaEngine')
    root.addObject('RequiredPlugin', name='SofaImplicitOdeSolver')
    root.addObject('RequiredPlugin', name='SofaRigid')
    root.addObject('RequiredPlugin', name='SofaSimpleFem')
    root.addObject('VisualStyle', displayFlags='showForceFields')
    root.gravity = [0, -10, 0]
    root.addObject('RegularGridTopology', name='topology', n=[3, 3, 3], min=[-1, -1, -1], max=[1, 1, 1])
    root.addObject('MeshTopology', name='mesh_topology', src='@topology')
    mech_obj = root.addObject('MechanicalObject', template='Vec3d')
    root.addObject('EulerImplicitSolver')
    root.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixd')
    root.addObject('HexahedronFEMForceField', src='@mesh_topology')
    root.addObject('UniformMass', totalMass=.1)

    root.addObject(MyController(mech=mech_obj, root=root))

    return root


def update_graph(x, y):
    figure = plt.gcf()
    figure.clf()

    plt.plot(x, y)
    plt.show(block=False)
    plt.pause(.0001)


class MyController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.mech = kwargs['mech']
        self.root = kwargs['root']
        self.position = []
        self.time = []
        self.rate_update = 0

    def onAnimateEndEvent(self, event):
        self.position.append(self.mech.position.value[0][1])
        self.time.append(self.root.time.value)
        if self.rate_update == 100:
            update_graph(self.time, self.position)
            self.rate_update = 0
        self.rate_update += 1


if __name__ == "__main__":
    main()