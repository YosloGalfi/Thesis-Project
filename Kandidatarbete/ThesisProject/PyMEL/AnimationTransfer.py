import pymel.core as pm
import pymel.core.datatypes as dt
import sys

# Root joints for source and target
sourceRoot = pm.ls(sl = True, type = 'joint')[0]
targetRoot = pm.ls(sl=True, type = 'joint')[1]

# Global list for joints
sourceList = []
targetList = []

# Get the length of the animation
animationLength = (pm.keyframe(q=True, kc=True)) / 10
#animationLength = 630

# Source rotation/orientation
sourceBindPoseRotation = []

# Target rotation/orientation
targetBindPoseRotation = []

# Different space matrices
worldRotation = []
translatedRotation = []

# Parent matrices
sourceParentMatrices = []
targetParentMatrices = []

def loadList(node, string):
    if string == "source":
        sourceList.append(node)
    if string == "target":
        targetList.append(node)
           
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
                sourceBindPoseRotation.append(joint.getRotation().asMatrix())
                parent = 1
                parentMatrix = getParentsMatrix(joint, parent)
                sourceParentMatrices.append(parentMatrix)
            
            keyframeRotation = joint.getRotation().asMatrix()
            keyframeOrientation = joint.getOrientation().asMatrix()
            
            #Isolate rotation
            isolatedRotation = sourceBindPoseRotation[i-1].inverse() * keyframeRotation;
            
            # World rotation
            pm.currentTime(keys)
            worldRotation.append(keyframeOrientation.inverse() * sourceParentMatrices[i-1].inverse() * isolatedRotation * sourceParentMatrices[i-1] * keyframeOrientation)
                       
# Function to load and set information from target
def loadTarget(node, keys):
    # For every joint in the list
    for i, joint in enumerate(node):
        # If we have passed the root joint
        if i > 0:
            # If we are on the first keyframe   
            if keys == 0:
                targetBindPoseRotation.append(joint.getRotation().asMatrix())
                parent = 1
                parentMatrix = getParentsMatrix(joint, parent)
                targetParentMatrices.append(parentMatrix)
            
            keyframeRotation = joint.getRotation().asMatrix()
            keyframeOrientation = joint.getOrientation().asMatrix()
            
            # Calculate the rotation from the source relative to the target
            pm.currentTime(keys)
            b = (keyframeOrientation * targetParentMatrices[i-1] * worldRotation[i-1] * targetParentMatrices[i-1].inverse() * keyframeOrientation.inverse())
            c = targetBindPoseRotation[i-1] * b
            
            # Set the rotation and keyframe
            joint.setRotation(dt.degrees(dt.EulerRotation(c)))
            pm.setKeyframe(joint)
            
# Function to get the matrix/matrices of a joints parent/parents
def getParentsMatrix(child, parentMatrix):
    # Get the first parent
    jointparent = child.getParent()
    # If the parent is a joint, we calculates the matrix and then calls the function again, to check if their is another parent
    if type(jointparent) == pm.nodetypes.Joint:
        parentMatrix = getParentsMatrix(jointparent, parentMatrix)
        parentMatrix = (jointparent.getRotation().asMatrix() * jointparent.getOrientation().asMatrix()) * parentMatrix
  
    return parentMatrix 
   
# Function to transfer data            
def transferData():
    
    loadList(sourceRoot, "source")
    loadList(targetRoot, "target")
    
    # For ever keyframe in the animation
    for keys in range(int(animationLength)):
        pm.currentTime(keys)
        
        del worldRotation[:]
        del translatedRotation[:]
        
        # Get attributes from the source root 
        rootTranslation = sourceRoot.getTranslation()
        rootOrientation = sourceRoot.getOrientation()
        rootRotation = sourceRoot.getRotation()
        
        # Call source and target functions
        loadSource(sourceList, keys)
        loadTarget(targetList, keys)
        
        # Set the attributes for the target root
        targetRoot.setOrientation(rootOrientation)
        targetRoot.setRotation(rootRotation)
        targetRoot.setTranslation(rootTranslation)
        
        # Set keyframe for target root
        pm.setKeyframe(targetRoot)
      
    # print "done"
    pm.currentTime(0)
transferData()