# Thesis-Project
Thesis project for our bachelor's degree. 

For our bachelor's degree project, we chose to implement scripts in Autodesk Maya, to transfer the animation from one skeleton to another. 
The scripts are created with various Python modules, such as NumPy, SciPy, PyMEL and OpenMaya. One version is also created with the C++ API for Maya. 

The scripts traverse the hierarchies of the skeletons, in order to calculate the rotations of each joint, so that the rotations are in relation to the target skeletons space.

The C++ plug in delivers the fastest speed in total, while OpenMaya is the fastest Python based script. PyMEL delivers the slowest script.
