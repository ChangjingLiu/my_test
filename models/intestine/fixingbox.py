from splib3.objectmodel import SofaPrefab
from splib3.numerics import getOrientedBoxFromTransform



def FixingBox(parent, target, name='FixingBox',
              translation=None, eulerRotation=None, scale=None):
        '''Fix a set of 'dofs' according to a translation & orientation'''

        if scale is None:
            scale = [1.0, 1.0, 1.0]
        if eulerRotation is None:
            eulerRotation = [0.0, 0.0, 0.0]
        if translation is None:
            translation = [0.0, 0.0, 0.0]
        ob = getOrientedBoxFromTransform(translation=translation, eulerRotation=eulerRotation, scale=scale)

        self = parent.addChild(name)
        self.addObject('BoxROI',
                               orientedBox=ob,
                               name='BoxROI',
                               position=target.dofs.getData('rest_position').getLinkPath(),
                               drawBoxes=False)

        c = self.addChild('Constraint')
        target.addChild(c)

        c.addObject('RestShapeSpringsForceField',
                       points=self.BoxROI.getData('indices').getLinkPath(),
                       stiffness=1e12)
        return self
