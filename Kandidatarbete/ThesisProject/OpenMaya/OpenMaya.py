import maya.OpenMaya as om
import maya.OpenMayaAnim as oma
import maya.cmds as cmds
import pymel.core as pm

sourceAList = om.MDagPathArray()
targetAList = om.MDagPathArray()

sourceRootStr = cmds.ls(sl=True, type = 'joint')[0] 
targetRootStr = cmds.ls(sl=True, type = 'joint')[1]

sourceBindPoseRotation = om.MMatrixArray()
targetBindPoseRotation = om.MMatrixArray()

sourceParentMatrices = om.MMatrixArray()
targetParentMatrices = om.MMatrixArray()	

worldRotation = om.MMatrixArray()	
translatedRotation = om.MMatrixArray()

animationLength = (pm.keyframe(q=True, kc=True)) / 10

def loadList(node, source):      
            
    if source:
        sourceAList.append(node)
    else:        
        targetAList.append(node)
        
    currentJoint = oma.MFnIkJoint(node)            
    if currentJoint.childCount() > 0:               
        for i in range(currentJoint.childCount()):
            childObject = om.MObject(currentJoint.child(i))            
            childDagNode = om.MFnDagNode(childObject)               
            childPath = om.MDagPath()
            childDagNode.getPath(childPath)
               
            loadList(childPath, source)    
 
 
def getParentMatrix(node, parentMatrix):
    jntobject = node.parent(0)  
    if jntobject.hasFn(om.MFn.kJoint):
        parentjnt = oma.MFnIkJoint(jntobject)
    
        parentMatrix = getParentMatrix(parentjnt, parentMatrix)
        
        # Get rotation
        rotation = om.MQuaternion()
        parentjnt.getRotation(rotation)      
        
        # Get orientation
        orient = om.MQuaternion()
        parentjnt.getOrientation(orient)
        
        parentMatrix = (rotation.asMatrix() * orient.asMatrix()) * parentMatrix
    
    return parentMatrix


def loadTarget(node, keys):
    for i in range(node.length()):
        if i > 0:
            jnt = oma.MFnIkJoint(node[i])
            
            if keys == 0:
                rotation = om.MQuaternion()
                jnt.getRotation(rotation)
                targetBindPoseRotation.append(rotation.asMatrix())
                
                par = om.MMatrix().setToIdentity()                
                parmat = getParentMatrix(jnt, par)
                targetParentMatrices.append(parmat)
                
            # Get rotation
            keyframerotation = om.MQuaternion()
            jnt.getRotation(keyframerotation)      
        
            # Get orientation
            keyframeorient = om.MQuaternion()
            jnt.getOrientation(keyframeorient)
            
            om.MGlobal.viewFrame(keys)
            
            b = (keyframeorient.asMatrix() * targetParentMatrices[i-1] * worldRotation[i-1] * targetParentMatrices[i-1].inverse() * keyframeorient.inverse().asMatrix())
            c = targetBindPoseRotation[i-1] * b            
            
            ac = om.MTransformationMatrix(c)              
            jnt.setRotation(ac.eulerRotation())
            
            dag = om.MFnDagNode(node[i].node())
            name = dag.name()
            cmds.select(name)
            target = cmds.ls(sl=True, type ='joint')[0]
                       
            cmds.setKeyframe()
            #pm.setKeyframe(target)
                                            
def loadSource(node, keys):
    for i in range(node.length()):
        if i > 0:
            jnt = oma.MFnIkJoint(node[i])
            
            if keys == 0:               
                rotation = om.MQuaternion()
                jnt.getRotation(rotation)
                sourceBindPoseRotation.append(rotation.asMatrix())
                
                par = om.MMatrix().setToIdentity()                
                parmat = getParentMatrix(jnt, par)
                sourceParentMatrices.append(parmat)
                
                
            # Get rotation
            keyframerotation = om.MQuaternion()
            jnt.getRotation(keyframerotation)      
        
            # Get orientation
            keyframeorient = om.MQuaternion()
            jnt.getOrientation(keyframeorient)
            
            isolatedRotation = sourceBindPoseRotation[i-1].inverse() * keyframerotation.asMatrix()
                        
            om.MGlobal.viewFrame(keys)
            
            worldRot = keyframeorient.inverse().asMatrix() * sourceParentMatrices[i-1].inverse() * isolatedRotation * sourceParentMatrices[i-1] * keyframeorient.asMatrix()    
            worldRotation.append(worldRot) 



def transfer():
    
    selList = om.MSelectionList()
    om.MGlobal.getActiveSelectionList(selList)
    
    sourceRoot = om.MDagPath()
    targetRoot = om.MDagPath()
    
    # Dagpaths for roots
    selList.getDagPath(0, sourceRoot)
    selList.getDagPath(1, targetRoot)    
    
    loadList(sourceRoot, True)
    loadList(targetRoot, False)  
    
    
    for i in range(int(animationLength)):
        om.MGlobal.viewFrame(i)
        
        worldRotation.clear() 
        translatedRotation.clear()
        
        rootObject = oma.MFnIkJoint(sourceRoot)        
                       
        # Get rotation
        rotation = om.MQuaternion()
        rootObject.getRotation(rotation)        
        
        # Get orientation
        orient = om.MQuaternion()
        rootObject.getOrientation(orient)
        
        # Get translatoin
        translation = rootObject.getTranslation(om.MSpace.kTransform)
                    
        loadSource(sourceAList, i)
        loadTarget(targetAList, i)
        
        # Target set
        cmds.select(targetRootStr)
        targetObject = oma.MFnIkJoint(targetRoot)
        targetObject.setRotation(rotation)
        targetObject.setOrientation(orient)
        targetObject.setTranslation(translation, om.MSpace.kTransform)
        
        
        cmds.setKeyframe()
        #pm.setKeyframe(targetRootStr)   
    
    om.MGlobal.viewFrame(0)
    cmds.select(sourceRootStr)
    cmds.select(targetRootStr, add=True) 
   

def testing():   

    nrOfTimes = 1
    
    textfilepath = "C:/Users/Galfi/Documents/OpenMaya.txt"
    textfile = open(textfilepath, "wb") 
    
    for i in range(nrOfTimes):
        
        cmds.timer(s=True)
        
        transfer()   
        
        t = cmds.timer(e = True)
        time = str(t) + "\n"
        timeEncode = time.encode()
        textfile.write(timeEncode)
        
        om.MGlobal.viewFrame(0)
        
        sourceBindPoseRotation.clear()
        targetBindPoseRotation.clear()
        sourceAList.clear()
        targetAList.clear()
        sourceParentMatrices.clear() 
        targetParentMatrices.clear()        
        
    textfile.close() 
      
testing()

 
    
    
