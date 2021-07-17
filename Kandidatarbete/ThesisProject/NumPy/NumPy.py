import maya.cmds as cmds
import pymel.core as pm
import pymel.core.datatypes as dt
import numpy as np

# Root joints for source and target
sourceRoot = pm.ls(sl=True, type='joint')[0]
targetRoot = pm.ls(sl=True, type='joint')[1]

# Source and target lists
pmSource = []
pmTarget = []

animationLength = (pm.keyframe(q=True, kc=True)) / 10
#animationLength = 51
animlength = np.intc(animationLength)

# Source and target rotation/orientation
size = np.intc(len(pm.ls(type = 'joint')) / 2)

sourceBindPoseRotation = np.zeros((size, 4, 4), dtype=np.float32)
targetBindPoseRotation = np.zeros((size, 4, 4), dtype=np.float32)

# Different space matrices
worldRotation = np.zeros((size, 4, 4), dtype=np.float32)
translatedRotation = np.zeros((size, 4, 4), dtype=np.float32)

# Parent matrices
sourceParentMatrices = np.zeros((size, 4, 4), dtype=np.float32)
targetParentMatrices = np.zeros((size, 4, 4), dtype=np.float32)


def loadList(node, string):
    
    if string == "source":
        pmSource.append(node)    
        
    if string == "target":
        pmTarget.append(node)
             
    if node.numChildren() > 0:    
        for child in node.getChildren():
            loadList(child, string)     


# Function to load information from target
def loadSource(node, keys):
    for i, joint in enumerate(node):
        # If we have passed the root joint
        if i > 0:
            # If we are on the first keyframe
            if keys == 0:                
                sourceBindPoseRotation[i-1] = np.matrix(joint.getRotation().asMatrix())                
                sourceParentMatrices[i-1] = getParentsMatrix(joint, np.identity(4))                
            
            keyframeRotation = np.matrix(joint.getRotation().asMatrix())
            keyframeOrientation = np.matrix(joint.getOrientation().asMatrix())
            
            #Isolate rotation
            sourceBindInverse = np.linalg.inv(sourceBindPoseRotation[i-1])
            isolatedRotation = np.matmul(sourceBindInverse,keyframeRotation)
            
            # World rotation
            pm.currentTime(keys)
            keyframeOrientInverse = np.linalg.inv(keyframeOrientation)
            sourceParentInverse = np.linalg.inv(sourceParentMatrices[i-1])
            
            f1 = np.matmul(keyframeOrientInverse, sourceParentInverse)
            f2 = np.matmul(isolatedRotation, sourceParentMatrices[i-1])            
            
            worldRotation[i-1] = np.matmul(np.matmul(f1, f2),keyframeOrientation)
            

def loadTarget(node, keys):
    for i, joint in enumerate(node):
        if i > 0:   
            if keys == 0:                
                targetBindPoseRotation[i-1] = np.matrix(joint.getRotation().asMatrix())                
                targetParentMatrices[i-1] = getParentsMatrix(joint, np.identity(4))
          
                
            keyframeRotation = np.matrix(joint.getRotation().asMatrix())
            keyframeOrientation = np.matrix(joint.getOrientation().asMatrix())
                        
            # Calculate the rotation from the source relative to the target
            pm.currentTime(keys)
            targetparentMatriceInverse = np.linalg.inv(targetParentMatrices[i-1])
            keyframeorientInverse = np.linalg.inv(keyframeOrientation)
            
            f1 = np.matmul(np.matmul(np.matmul(keyframeOrientation, targetParentMatrices[i-1]), np.matmul(worldRotation[i-1], targetparentMatriceInverse)), keyframeorientInverse)
            
            # Set the rotation and keyframe
            joint.setRotation(dt.degrees(dt.EulerRotation(dt.Matrix(np.matmul(targetBindPoseRotation[i-1], f1).tolist()))))
            pm.setKeyframe(joint)
            
            
def getParentsMatrix(child, parentMatrix):    
    
    # If the parent is a joint, we calculates the matrix and then calls the function again, to check if their is another parent
    if type(child.getParent()) == pm.nodetypes.Joint:
        parentMatrix = getParentsMatrix(child.getParent(), parentMatrix)
                
        jointParentRotation = np.matrix(child.getParent().getRotation().asMatrix())               
        jointParentOrient = np.matrix(child.getParent().getOrientation().asMatrix()) 
                
        parentMatrix = np.matmul((jointParentRotation * jointParentOrient), parentMatrix)
  
    return parentMatrix                            
    

# Transfer function
def transferData():
        
    loadList(sourceRoot, "source")
    loadList(targetRoot, "target")
       
    for keys in range(np.intc(animlength)):
        cmds.currentTime(keys)
        
        np.empty_like(worldRotation)
        np.empty_like(translatedRotation)
        
        rootTranslation = np.array(sourceRoot.getTranslation())
        rootOrientation = np.array(sourceRoot.getOrientation())
        rootRotation = np.array(sourceRoot.getRotation()) 
        
        loadSource(pmSource, keys)
        loadTarget(pmTarget, keys)
        
        targetRoot.setOrientation(np.array(sourceRoot.getOrientation()))
        targetRoot.setRotation(np.array(sourceRoot.getRotation()))
        targetRoot.setTranslation(rootTranslation)
       
        pm.setKeyframe(targetRoot)
         
    pm.currentTime(0)

              
def testing():    
    nrOfTimes = 1
   
      
    textfilepath = "C:/Users/Galfi/Documents/NumPy.txt"
    textfile = open(textfilepath, "wb") 
    
    for i in range(nrOfTimes):        

        cmds.timer(s=True)
        
        transferData()  
        
        t = cmds.timer(e = True)
        time = str(t) + "\n"
        timeEncode = time.encode()
        textfile.write(timeEncode)
        pm.currentTime(0)  
        
        np.empty_like(sourceBindPoseRotation)
        np.empty_like(targetBindPoseRotation)
        del pmSource[:]
        del pmTarget[:] 
        #np.empty_like(pmSource)
        #np.empty_like(pmTarget)
        np.empty_like(sourceParentMatrices)
        np.empty_like(targetParentMatrices)        
        
    textfile.close() 

   
testing()