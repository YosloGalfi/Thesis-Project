import maya.cmds as cmds
import pymel.core as pm
import pymel.core.datatypes as dt
import scipy as sp
from scipy import linalg as math

# Root joints for source and target
sourceRoot = pm.ls(sl=True, type='joint')[0]
targetRoot = pm.ls(sl=True, type='joint')[1]

# Animation length 
animationLength = (pm.keyframe(q=True, kc=True)) / 10

# Source and target rotation/orientation
size = int(len(pm.ls(type = 'joint')) / 2)

# Bindpose
sourceBindpose = sp.zeros((size, 4, 4), dtype=sp.float32)
targetBindpose = sp.zeros((size, 4, 4), dtype=sp.float32)

# Different space matrices
worldRotation = sp.zeros((size, 4, 4), dtype=sp.float32)
translatedRotation = sp.zeros((size, 4, 4), dtype=sp.float32)

# Parent matrices
sourceParentMatrices = sp.zeros((size, 4, 4), dtype=sp.float32)
targetParentMatrices = sp.zeros((size, 4, 4), dtype=sp.float32)

# Source and target lists
sourceList = [None] * size
targetList = [None] * size

# Indices for lists
sourceIndex = 0
targetIndex = 0

# Load source hierarchy
def loadSourceList(node, index):
    sourceList[index] = node
    index += 1
    
    if node.numChildren() > 0:
        for child in node.getChildren():
            index = loadSourceList(child, index)
    
    return index

# Load target hierarchy
def loadTargetList(node, index):
    targetList[index] = node
    index += 1
    
    if node.numChildren() > 0:
        for child in node.getChildren():
            index = loadTargetList(child, index)
    
    return index

# Function to load information from target
def loadSource(node, keys):
    for i, joint in enumerate(node):
        # If we have passed the root joint
        if i > 0:
            # If we are on the first keyframe
            if keys == 0:                
                sourceBindpose[i-1] = sp.matrix(joint.getRotation().asMatrix())                
                sourceParentMatrices[i-1] = getParentsMatrix(joint, sp.identity(4))                
            
            keyframeRotation = sp.matrix(joint.getRotation().asMatrix())
            keyframeOrientation = sp.matrix(joint.getOrientation().asMatrix())
            
            #Isolate rotation
            isolatedRotation = sp.matmul(math.inv(sourceBindpose[i-1]),keyframeRotation)
            
            # World rotation
            f1 = sp.matmul(math.inv(keyframeOrientation), math.inv(sourceParentMatrices[i-1]))
            f2 = sp.matmul(isolatedRotation, sourceParentMatrices[i-1])            
            
            worldRotation[i-1] = sp.matmul(sp.matmul(f1, f2),keyframeOrientation)
            

def loadTarget(node, keys):
    for i, joint in enumerate(node):
        if i > 0:   
            if keys == 0:                
                targetBindpose[i-1] = sp.matrix(joint.getRotation().asMatrix())                
                targetParentMatrices[i-1] = getParentsMatrix(joint, sp.identity(4))
          
                
            keyframeRotation = sp.matrix(joint.getRotation().asMatrix())
            keyframeOrientation = sp.matrix(joint.getOrientation().asMatrix())
                        
            # Calculate the rotation from the source relative to the target
            jointSpace = sp.matmul(sp.matmul(sp.matmul(keyframeOrientation, targetParentMatrices[i-1]), sp.matmul(worldRotation[i-1], math.inv(targetParentMatrices[i-1]))), math.inv(keyframeOrientation))
            
            # Set the rotation and keyframe
            joint.setRotation(dt.degrees(dt.EulerRotation(dt.Matrix(sp.matmul(targetBindpose[i-1], jointSpace).tolist()))))
            pm.setKeyframe(joint)
             
def getParentsMatrix(child, parentMatrix):
    if type(child.getParent()) == pm.nodetypes.Joint:
        parentMatrix = getParentsMatrix(child.getParent(), parentMatrix)
                
        jointParentRotation = sp.matrix(child.getParent().getRotation().asMatrix())               
        jointParentOrient = sp.matrix(child.getParent().getOrientation().asMatrix()) 
                
        parentMatrix = sp.matmul((jointParentRotation * jointParentOrient), parentMatrix)
    return parentMatrix 

def transferData(sIndex, tIndex):
     
    sIndex = loadSourceList(sourceRoot, sourceIndex)
    tIndex = loadTargetList(targetRoot, targetIndex)
       
    for keys in range(int(animationLength)):
        cmds.currentTime(keys)
        
        loadSource(sourceList, keys)
        loadTarget(targetList, keys)
        
        targetRoot.setOrientation(sp.array(sourceRoot.getOrientation()))
        targetRoot.setRotation(sp.array(sourceRoot.getRotation()))
        targetRoot.setTranslation(sp.array(sourceRoot.getTranslation()))
       
        pm.setKeyframe(targetRoot)
         
    pm.currentTime(0)

def transferSciPy():    
    nrOfTimes = 1
    
    textfilepath = "C:/Users/Galfi/Documents/scipy.txt"
    textfile = open(textfilepath, "wb") 
    
    for i in range(nrOfTimes):        
        
        sourceIndex = 0
        targetIndex = 0
        
        cmds.timer(s=True)
        
        transferData(sourceIndex, targetIndex)  
        
        t = cmds.timer(e = True)
        time = str(t) + "\n"
        timeEncode = time.encode()
        textfile.write(timeEncode)
        pm.currentTime(0)  
           
    textfile.close() 

   
transferSciPy()
 
    
                                    

       
            
    