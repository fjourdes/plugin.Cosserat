# -*- coding: utf-8 -*-

import os
import Sofa
from stlib.scene import MainHeader, ContactHeader

path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'


class Animation(Sofa.PythonScriptController):

    def __init__(self, rigidBaseNode, rateAngularDeformNode):
        self.rigidBaseNode = rigidBaseNode
        self.rateAngularDeformNode = rateAngularDeformNode

        self.rate = 0.2
        self.angularRate = 0.0
        return

    def initGraph(self, nodeRigid):
        self.rigidBaseMO = self.rigidBaseNode.getObject('RigidBaseMO')
        self.rateAngularDeformMO = self.rateAngularDeformNode.getObject(
            'rateAngularDeformMO')

    def onKeyPressed(self, c):

        if ord(c) == 19:  # up
            pos = self.rigidBaseMO.findData('rest_position').value
            pos[0][1] += self.rate
            self.rigidBaseMO.findData('rest_position').value = pos
            print("=======> Position :", pos)

            posA = self.rateAngularDeformMO.findData('position').value
            for i in range(len(posA)):
                posA[i][1] += self.angularRate
            self.rateAngularDeformMO.findData('position').value = posA

        if ord(c) == 21:  # down
            pos = self.rigidBaseMO.findData('rest_position').value
            pos[0][0] -= self.rate
            self.rigidBaseMO.findData('rest_position').value = pos
            # print("=======> Position :",pos)

            posA = self.rateAngularDeformMO.findData('position').value
            for i in range(len(posA)):
                posA[i][1] -= self.angularRate
            self.rateAngularDeformMO.findData('position').value = posA

        if ord(c) == 18:  # left
            pos = self.rigidBaseMO.findData('position').value
            pos[0][2] -= self.rate
            self.rigidBaseMO.findData('position').value = pos
            print("=======> Position :", pos)

            posA = self.rateAngularDeformMO.findData('position').value
            for i in range(len(posA)):
                posA[i][2] -= self.angularRate
            self.rateAngularDeformMO.findData('position').value = posA

        if ord(c) == 20:  # right
            pos = self.rigidBaseMO.findData('position').value
            pos[0][2] += self.rate
            self.rigidBaseMO.findData('position').value = pos
            print("=======> Position :", pos)

            posA = self.rateAngularDeformMO.findData('position').value
            for i in range(len(posA)):
                posA[i][2] += self.angularRate
            self.rateAngularDeformMO.findData('position').value = posA


def createScene(rootNode):

    MainHeader(rootNode, plugins=["SoftRobots", "SoftRobots.Inverse", "SofaPython", "SofaSparseSolver", "SofaPreconditioner", "SofaOpenglVisual", "CosseratPlugin", "BeamAdapter"],
               repositoryPaths=[os.getcwd()])

    rootNode.createObject(
        'VisualStyle', displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields showWireframe')

    rootNode.createObject('FreeMotionAnimationLoop')
    # rootNode.createObject('QPInverseProblemSolver', printLog='0')
    rootNode.createObject('GenericConstraintSolver', tolerance="1e-20", maxIterations="5 00", printLog="0")


    rootNode.gravity = "0 -9180 0"
    # rootNode.createObject('BackgroundSetting', color='0 0.168627 0.211765')
    rootNode.createObject('OglSceneFrame', style="Arrows",
                          alignment="TopRight")

    ##########################################
    # FEM Model                              #
    ##########################################
    finger = rootNode.createChild('finger')
    finger.createObject('EulerImplicitSolver', name='odesolver',
                        firstOrder='0', rayleighMass=0.1, rayleighStiffness=0.1)
    finger.createObject('SparseLDLSolver', name='preconditioner')

    # Add a componant to load a VTK tetrahedral mesh and expose the resulting topology in the scene .
    finger.createObject('MeshVTKLoader', name='loader', filename=path +
                        'finger.vtk', translation="-17.5 -12.5 7.5", rotation="0 180 0")
    finger.createObject('TetrahedronSetTopologyContainer',
                        src='@loader', name='container')
    finger.createObject('TetrahedronSetTopologyModifier')
    finger.createObject(
        'TetrahedronSetTopologyAlgorithms', template='Vec3d')
    finger.createObject(
        'TetrahedronSetGeometryAlgorithms', template='Vec3d')

    # Create a mechanicaobject component to stores the DoFs of the model
    finger.createObject('MechanicalObject', name='tetras', template='Vec3d',
                        showIndices='false', showIndicesScale='4e-5', rx='0', dz='0')

    # Gives a mass to the model
    finger.createObject('UniformMass', totalMass='0.075')

    # Add a TetrahedronFEMForceField componant which implement an elastic material model solved using the Finite Element Method on
    # tetrahedrons.
    finger.createObject('TetrahedronFEMForceField', template='Vec3d',
                        name='FEM', method='large', poissonRatio='0.45',  youngModulus='600')

    finger.createObject('BoxROI', name='ROI1',
                        box='-18 -15 -8 2 -3 8', drawBoxes='true')
    finger.createObject('RestShapeSpringsForceField',
                        points='@ROI1.indices', stiffness='1e12')



    ##########################################
    # Cable points                           #
    ##########################################
    # Mappe points inside the meca, this points will be use for the bilateral mapping
    FEMpos = [" 0.0 0 0 15 0 0 30 0 0 45 0 0 60 0 0 66 0 0 81 0.0 0.0"]
    # FEMpos = [" 81 0.0 0.0"]
    
    femPoints = finger.createChild('femPoints')
    inputFEMCable = femPoints.createObject('MechanicalObject', name="pointsInFEM", position=FEMpos, showObject="0", showIndices="1",showObjectScale='2', showIndicesScale='0.0')    
    femPoints.createObject('BarycentricMapping')

    ##########################################
    # Visualization                          #
    ##########################################
    fingerVisu = finger.createChild('visu')
    fingerVisu.createObject(
        'MeshSTLLoader', filename=path+"finger.stl", name="loader", translation="-17.5 -12.5 7.5",
        rotation="0 180 0",)
    fingerVisu.createObject('OglModel', src="@loader",
                            template='ExtVec3f', color="0.0 0.7 0.7")
    fingerVisu.createObject('BarycentricMapping')


    finger.createObject('LinearSolverConstraintCorrection')

    # ###############
    # New adds to use the sliding Actuator
    ###############
    cableNode = rootNode.createChild('cableNode')
    cableNode.createObject('EulerImplicitSolver', firstOrder="0", rayleighStiffness="1.0", rayleighMass='0.1')
    cableNode.createObject('SparseLUSolver', name='solver')
    cableNode.createObject('GenericConstraintCorrection')

    # ###############
    # RigidBase
    ###############
    rigidBaseNode = cableNode.createChild('rigidBase')
    RigidBaseMO = rigidBaseNode.createObject('MechanicalObject', template='Rigid3d',
                                             name="RigidBaseMO", position="0 0 0  0 0 0 1", showObject='1', showObjectScale='0.1')
    rigidBaseNode.createObject('RestShapeSpringsForceField', name='spring', stiffness="50000",
                               angularStiffness="50000", external_points="0", mstate="@RigidBaseMO", points="0", template="Rigid3d")

    ###############
    # Rate of angular Deformation  (2 sections)
    ###############
    position = ["0 0 0 " + "0 0 0 " + "0 0 0 " +
                "0 0 0 " + "0 0 0 " + "0 0 0 "]
    longeur = '15 15 15 15 6 15'  # beams size
    rateAngularDeformNode = cableNode.createChild('rateAngularDeform')
    rateAngularDeformMO = rateAngularDeformNode.createObject(
        'MechanicalObject', template='Vec3d', name='rateAngularDeformMO', position=position)
    BeamHookeLawForce = rateAngularDeformNode.createObject(
        'BeamHookeLawForceField', crossSectionShape='circular', length=longeur, radius='0.5', youngModulus='5e6')
    # BeamHookeLawForce = rateAngularDeformNode.createObject('CosseratInternalActuation', name="BeamHookeLawForce",  crossSectionShape='circular', radius='0.5', youngModulus='5e6')

    ################################
    # Animation (to move the dofs) #
    ################################
    anim = Animation(rigidBaseNode, rateAngularDeformNode)

    ##############
    #   Frames   #
    ##############
    frames = ["0.0 0 0  0 0 0 1   5 0 0  0 0 0 1  10.0 0 0  0 0 0 1    15.0 0 0  0 0 0 1   20.0 0 0  0 0 0 1" +
              " 30.0 0 0  0 0 0 1  35.0 0 0  0 0 0 1   40.0 0 0  0 0 0 1   45.0 0 0  0 0 0 1 55.0 0 0  0 0 0 1 60.0 0 0  0 0 0 1" +
              " 66.0 0 0  0 0 0 1   71.0 0 0  0 0 0 1   76.0 0 0  0 0 0 1  81.0 0 0  0 0 0 1"]
    # the node of the frame needs to inherit from rigidBaseMO and rateAngularDeform
    mappedFrameNode = rigidBaseNode.createChild('MappedFrames')
    rateAngularDeformNode.addChild(mappedFrameNode)
    framesMO = mappedFrameNode.createObject(
        'MechanicalObject', template='Rigid3d', name="FramesMO", position=frames, showObject='1', showObjectScale='0')

    # The mapping has two inputs: RigidBaseMO and rateAngularDeformMO
    #                 one output: FramesMO
    inputMO = rateAngularDeformMO.getLinkPath()
    inputMO_rigid = RigidBaseMO.getLinkPath()
    outputMO = framesMO.getLinkPath()

    curv_abs_input = '0 15 30 45 60 66 81'
    curv_abs_output = '0.0 5 10 15 20 30 35 40 45 55 60 66 71 76 81'
    mappedFrameNode.createObject('DiscretCosseratMapping', curv_abs_input=curv_abs_input,
                                 curv_abs_output=curv_abs_output, input1=inputMO, input2=inputMO_rigid, output=outputMO, debug='0')

    # actuators = mappedFrameNode.createChild('actuators')
    # actuator0 = actuators.createObject('SlidingActuator', name="SlidingActuator0", template='Rigid3d',
    #                                    direction='0 0 0 1 0 0', indices=1, maxForce='100000', minForce='-30000')
    cable_position = [[0.0, 0.0, 0.0], [5.0, 0.0, 0.0], [10.0, 0.0, 0.0], [15.0, 0.0, 0.0], [20.0, 0.0, 0.0], [30.0, 0.0, 0.0], [35.0, 0.0, 0.0], [40.0, 0.0, 0.0], [45.0, 0.0, 0.0],
                      [55.0, 0.0, 0.0], [60.0, 0.0, 0.0], [66.0, 0.0, 0.0], [71.0, 0.0, 0.0], [76.0, 0.0, 0.0], [81.0, 0.0, 0.0]]
    #  This create a new node in the scene. This node is appended to the finger's node.
    slidingPoint = mappedFrameNode.createChild('slidingPoint')

    # This create a MechanicalObject, a componant holding the degree of freedom of our
    # mechanical modelling. In the case of a cable it is a set of positions specifying
    # the points where the cable is passing by.
    slidingPointMO = slidingPoint.createObject('MechanicalObject', name="cablePos", position=cable_position, showObject="0", showIndices="0")
    slidingPoint.createObject('Line', name="Line")
    slidingPoint.createObject('IdentityMapping')


    mappedPointsNode = slidingPoint.createChild('MappedPoints')
    femPoints.addChild(mappedPointsNode)
    mappedPoints = mappedPointsNode.createObject('MechanicalObject', template='Vec3d', position=FEMpos, name="FramesMO", showObject='0', showObjectScale='0')

    ## Get the tree mstate links for the mapping
    inputCableMO = slidingPointMO.getLinkPath()
    inputFEMCableMO = inputFEMCable.getLinkPath()
    outputPointMO = mappedPoints.getLinkPath()

    mappedPointsNode.createObject('QPSlidingConstraint', nodeame="QPConstraint")

    mappedPointsNode.createObject('DifferenceMultiMapping', name="pointsMulti", input1=inputFEMCableMO, input2=inputCableMO, output=outputPointMO, radius=3)


    # rootNode.createObject('BilateralInteractionConstraint', template='Vec3d', object2=inputCableMO, object1=inputFEMCableMO, first_point='6', second_point='14')



    # rootNode.createObject('ProjectionEngine', name="engine", fromPos="@finger/femPoints/pointsInFEM.position",  destination="@cableNode/rigidBase/MappedFrames/slidingPoint/cablePos.position")

    # rootNode.createObject('ProjectionEngine', name="engine", from="@finger/femPoints/pointsInFEM.position", destination="@cableNode/rigidBase/MappedFrames/slidingPoint/cablePos.position")

    # rootNode.createObject('QPSlidingConstraint', name="constraint2", object1="@finger/femPoints/pointsInFEM",
    #                       object2="@cableNode/rigidBase/MappedFrames/slidingPoint/cablePos", sliding_point="0 1", axis_1="0", axis_2="2", indices={0, 1, 3}, maxForce='100000', minForce='-30000')
    # QPSlidingConstraint
    return rootNode
