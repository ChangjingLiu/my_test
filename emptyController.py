import Sofa
import os
import sys
import numpy as np
import math
from scipy.spatial.transform import Rotation as R


def quat_vector(quat):
    '''
    四元数转换为旋转矩阵，并左乘方向向量[0,1,0]
    :param quat: [qx,qy,qz,w]
    :return: [x,y,z]
    '''
    # 构造四元数
    r = R.from_quat(quat)
    # 转化为旋转矩阵
    rotation_matrix = r.as_matrix()
    # 旋转矩阵左乘方向向量
    vector = np.matmul(rotation_matrix, np.array([0, 1, 0]))
    # euler0 = r.as_euler('xyz', degrees=True)
    # theta=2*math.acos(quat[6])
    # vector = np.zeros((1, 3))
    # vector = quat[3:6]/math.sin(theta/2)
    # vector=quat[3:7].as_as_euler('xyz', thetadegrees=False)
    return vector

def quat_rot(quat):
    '''
    四元数转换为旋转矩阵
    :param quat: [qx,qy,qz,w]
    :return: [x,y,z]
    '''
    # 构造四元数
    r = R.from_quat(quat)
    # 转化为旋转矩阵
    rotation_matrix = r.as_matrix()
    return rotation_matrix


# This python script shows the functions to be implemented
# in order to create your Controller in python
class EmptyController(Sofa.Core.Controller):

    # F_collies = np.zeros((8, 3))

    def __init__(self, *args, **kwargs):
        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        # 指针获取节点
        print('Controller initialized')
        self.node = kwargs["node"]
        self.GenericConstraintSolver = kwargs['GenericConstraintSolver']
        # self.Modelling = self.node.getChild("Modelling")
        # self.Simulation = self.node.getChild("Simulation")
        # self.Intestine = self.Modelling.getObject("Intestine")
        # self.Cube = self.Modelling.getObject("Cube")
        self.CubeConstraint = kwargs['cubeConstraint']
        self.Cube = kwargs['Cube']
        # self.F_collies = np.zeros((40, 3))

    # Default BaseObject functions********************************
    def init(self):
        pass

    def bwdInit():
        pass

    def reinit():
        pass

    # Default Events *********************************************
    def onAnimateBeginEvent(self, event):  # called at each begin of animation step
        # 获取Cube的姿态
        pos = self.Cube.mstate.position.value[0]
        cube_pos = quat_vector(pos[3:7])
        # print(cube_pos)

        # 获取constraint每个约束的法向力（对刚性接触面的方向未知）
        constraintLambda = self.GenericConstraintSolver.constraintForces.value
        # print(constraintlambda)
        constraint = self.CubeConstraint.value
        # print(constraint)
        constraint_oneline = constraint.split('\n')
        # len(self.constraintForcesLambda)
        # 每个节点的法向约束力
        F_collies = np.zeros((8, 3))

        # 对每一个约束进行计算
        for i in range(len(constraint_oneline)):
            # 第i个约束
            constraint_mat = list(map(eval, constraint_oneline[i].split()))
            if len(constraint_mat) > 0:
                # 第i个约束的个数j，即第二列
                # print("约束id：", i, "，约束个数", constraint_mat[1])
                for j in range(constraint_mat[1]):
                    # 第i个约束影响的节点id
                    id_index = 2 + j * 4
                    ID = constraint_mat[id_index]
                    # 第i个约束影响的节点id的方向
                    left = id_index + 1
                    right = left + 3

                    # print(ID)
                    # print(constraint_mat[left:right])
                    # print("点id：", ID, "矩阵", constraint_mat[left:right])
                    # # 1第一种力在世界坐标系下
                    # F_collies[ID] += np.array(constraint_mat[left:right]) * constraintLambda[i] / 0.001

                    # 2先计算世界坐标系下的力，再左乘旋转矩阵转换到局部坐标系
                    # print("旋转矩阵")
                    # print(quat_rot(pos[3:7]))
                    # print("力")
                    # print(np.dot(constraint_mat[left:right],constraintLambda[i]))
                    # print("final")
                    F_collies[ID] += np.matmul(quat_rot(pos[3:7]),np.dot(constraint_mat[left:right],constraintLambda[i]).T).T/0.001
                    # F_collies[ID] += np.array([1,1,1])*abs(constraintLambda[i]) / 0.001
                    # 计算约束力向量与刚性接触面的法向向量的内积
                    # ans = np.dot(np.array(constraint_mat[left:right]) * constraintLambda[i], cube_pos)
                    # if ans >= 0:
                    #     print("1\n")
                    #     F_collies[ID] += np.array([1, 1, 1]) * abs(constraintLambda[i]) / 0.001
                    # elif ans < 0:
                    #     print("2\n")
                    #     F_collies[ID] += np.array([1, 1, 1]) * abs(constraintLambda[i]) / 0.001


        print(F_collies)
        # return 0
        # with self.GenericConstraintSolver.constraintForces.writeableArray() as wa:
        #     print(wa)

    def onAnimateEndEvent(self, event):  # called at each end of animation step

        return 0

    def onKeypressedEvent(self, event):
        key = event['key']
        if ord(key) == 19:  # up
            print("You pressed the Up key")

        if ord(key) == 21:  # down
            print("You pressed the Down key")

        if ord(key) == 18:  # left
            print("You pressed the Left key")

        if ord(key) == 20:  # right
            print("You pressed the Right key")

    def onKeyreleasedEvent(self, event):
        key = event['key']
        if ord(key) == 19:  # up
            print("You released the Up key")

        if ord(key) == 21:  # down
            print("You released the Down key")

        if ord(key) == 18:  # left
            print("You released the Left key")

        if ord(key) == 20:  # right
            print("You released the Right key")

    def onMouseEvent(self, event):
        if (event['State'] == 0):  # mouse moving
            print("Mouse is moving (x,y) = " + str(event['mouseX']) + " , " + str(event['mouseY']))

        if (event['State'] == 1):  # left mouse clicked
            print("Left mouse clicked")

        if (event['State'] == 2):  # left mouse released
            print("Left mouse released")

        if (event['State'] == 3):  # right mouse released
            print("Right mouse clicked")

        if (event['State'] == 4):  # right mouse released
            print("Right mouse released")

        if (event['State'] == 5):  # wheel clicked
            print("Mouse wheel clicked")

        if (event['State'] == 6):  # wheel released
            print("Mouse wheel released")

    def onScriptEvent(self, event):
        pass

    def onEvent(self, event):
        pass


def createScene(root):
    root.dt = 0.01
    root.addObject('DefaultVisualManagerLoop')
    root.addObject('DefaultAnimationLoop')

    # Add our python controller in the scene
    root.addObject(EmptyController(name="MyEmptyController"))


def main():
    import SofaRuntime
    import Sofa.Gui
    SofaRuntime.importPlugin("SofaOpenglVisual")
    root = Sofa.Core.Node("root")
    createScene(root)

    Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 1080)
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()

    print("End of simulation.")


def main():
    # runSofa.exe路径
    path = "D:/Software_download/sofa_22/SOFA_robosoft2022_python-3.8_Windows/bin/runSofa"
    # 使用Sofa运行该文件
    os.system(path + " " + sys.argv[0])


if __name__ == '__main__':
    main()
