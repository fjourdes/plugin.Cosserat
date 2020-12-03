# -*- coding: utf-8 -*-

import sofa
from math import sin,cos, sqrt, pi
import os
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'




def createScene(rootNode):

    sofa.loadPlugin("CosseratPlugin")

    rootNode.createObject('VisualStyle', displayFlags='hideVisualModels showBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields hideInteractionForceFields hideWireframe')

    rootNode.createObject('FreeMotionTaskAnimationLoop')
    rootNode.createObject('GenericConstraintSolver', tolerance="1e-10", printLog='0')
    rootNode.gravity.value = "0 -9810 0"
    rootNode.dt.value ="0.02"
    
    ###############hresho
    ## Solver
    ###############
    
    rootNode = rootNode.createChild("Cosserat")
    
    rootNode.createObject('EulerImplicitSolver', rayleighStiffness="0.0", rayleighMass='0.0')
    rootNode.createObject('SparseLDLSolver', name='solver')
    rootNode.createObject('GenericConstraintCorrection')

    ###############hresho
    ## RigidBase
    ###############
    rigidBaseNode= rootNode.createChild('rigidBase')
    RigidBaseMO = rigidBaseNode.createObject('MechanicalObject', template='Rigid3d', name="RigidBaseMO", position="0 0 0  0 0 0. 1", showObject='1', showObjectScale='0.1', velocity='0 0 0.0 0.0 0 0' )
    rigidBaseNode.createObject("FixedConstraint", template="Rigid3d", name="fixedBase" )

    ###############
    ## Rate of angular Deformation  (2 sections)
    ###############
    #pos = pi
    array1 = [0.0,0.0,0.0]
    array2 = [0.0,0.0,0.0]
    array3 = [0.0,0.0,0.0]
    pos = [array1, array2, array3]

    rateAngularDeformNode = rootNode.createChild('rateAngularDeform')
    rateAngularDeformMO = rateAngularDeformNode.createObject('MechanicalObject', template='Vec3d', name='rateAngularDeformMO', position=pos, velocity='0 0 0.0 0 0 0  0 0 0') # (2 series of 3 angles for 2 sections. we suppose that the lenght is 10 for each)
    BeamHookeLawForce = rateAngularDeformNode.createObject('BeamHookeLawForceField', crossSectionShape='circular', length='10 10 10', radius='0.5', youngModulus='5e3')

    ##############
    ## Frames
    ##############
    # the node of the frame needs to inherit from rigidBaseMO and rateAngularDeform
    mappedFrameNode = rateAngularDeformNode.createChild('MappedFrames')
    framesMO = mappedFrameNode.createObject('MechanicalObject', template='Rigid3d', name="FramesMO", position="0.5 0 0  0 0 0 1  5 0 0 0 0 0 1   10 0 0  0 0 0 1   15 0 0 0 0 0 1  20 0 0  0 0 0 1 25 0 0  0 0 0 1 30 0 0  0 0 0 1", showObject='1', showObjectScale='1' )
    mappedFrameNode.createObject("UniformRigidMass", name="mass" )
    inputMO = rateAngularDeformMO.getPath() 
    inputMO_rigid = RigidBaseMO.getPath()
    outputMO = framesMO.getPath()
    mappedFrameNode.createObject('DiscretCosseratMapping', curv_abs_input='0 10 20 30', curv_abs_output='0.5 5 10 15 20 25 30', input1=inputMO, input2=inputMO_rigid,output=outputMO, debug='0' )

    return rootNode
