
#include "maya_includes.h"
#include <maya/MTimer.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include <queue>

#include <unordered_map>

using namespace std;

//std::vector<MObject> sourceList;
//std::vector<MObject> targetList;
//
//std::vector<MMatrix> sourceBindPoseRotation;
//std::vector<MMatrix> targetBindPoseRotation;
//
//std::vector<MMatrix> worldRotation;
//std::vector<MMatrix> translatedRotation;
//
//std::vector<MMatrix> sourceParentMatrices;
//std::vector<MMatrix> targetParentMatrices;


//std::vector<MObject> sourceList;
//std::vector<MObject> targetList;
//
//MMatrixArray sourceBindPoseRotation; 
//MMatrixArray  targetBindPoseRotation;
//
//MMatrixArray  worldRotation;
//MMatrixArray  translatedRotation;
//
//MMatrixArray  sourceParentMatrices;
//MMatrixArray  targetParentMatrices;

// Kan radera denna sen
void printList(std::vector<MObject> list)
{
	for (int i = 0; i < list.size(); i++)
	{
		MFnIkJoint joint(list[i]);
		std::cout << "Joint: " << joint.name() << std::endl;
	}
}

void loadList(MObject node, std::vector<MObject>& list)
{
	MFnIkJoint joint(node);

	list.push_back(node);

	if (joint.childCount() > 0)
	{
		for (int i = 0; i < joint.childCount(); i++)
		{
			MObject child = joint.child(i);

			MFnIkJoint joint(joint.child(i));
			if (child.hasFn(MFn::kJoint))
			{
				loadList(child, list);
			}
		}
	}
}

void allocateMemory(MMatrixArray& sourceBindPoseRotation, MMatrixArray& targetBindPoseRotation, MMatrixArray& worldRotation, MMatrixArray& translatedRotation, MMatrixArray& sourceParentMatrices,
	MMatrixArray& targetParentMatrices, std::vector<MObject>& targetList, std::vector<MObject> sourceList)
{
	targetList.reserve(sourceList.size());

	sourceBindPoseRotation.setLength(sourceList.size());
	targetBindPoseRotation.setLength(sourceList.size());
	sourceParentMatrices.setLength(sourceList.size());
	targetParentMatrices.setLength(sourceList.size());
	worldRotation.setLength(sourceList.size());
	translatedRotation.setLength(sourceList.size());
}

MMatrix getParentMatrix(MObject node, MMatrix parentMatrix)
{
	MFnIkJoint joint(node);
	if (joint.parent(0).hasFn(MFn::kJoint))
	{
		double x, y, z, w;
		MQuaternion orientation;
		MMatrix keyframeOrientation;
		MMatrix keyframeRotation;

		MFnIkJoint parent(joint.parent(0));
		parentMatrix = getParentMatrix(joint.parent(0), parentMatrix);

		parent.getRotationQuaternion(x, y, z, w);
		MQuaternion rotation(x, y, z, w);
		parent.getOrientation(orientation);

		keyframeOrientation = orientation.asMatrix();
		keyframeRotation = rotation.asMatrix();

		parentMatrix = (keyframeRotation * keyframeOrientation) * parentMatrix;
	}
	
	return parentMatrix;
}

void calculateSourceSkeleton(std::vector<MObject> skeleton, int keyframe, MMatrixArray& sourceBindPoseRotation, MMatrixArray& sourceParentMatrices, MMatrixArray& worldRotation)
{
	double x, y, z, w;
	MQuaternion orientation;
	MMatrix keyframeOrientation;
	MMatrix keyframeRotation;
	MMatrix identity;
	MStatus status;

	for (int i = 0; i < skeleton.size(); i++)
	{
		MFnIkJoint joint(skeleton[i]);

		// If we have passed the root bone
		if (i > 0)
		{
			// If we are at keyframe 0, save the bindpose
			if (keyframe == 0)
			{
				joint.getRotationQuaternion(x, y, z, w);
				MQuaternion bindPoserotation(x, y, z, w);
				sourceBindPoseRotation.set(bindPoserotation.asMatrix(), i-1);

				identity.setToIdentity();
				sourceParentMatrices.set(getParentMatrix(skeleton[i], identity), i-1);
			}
			//else
			//{
				joint.getRotationQuaternion(x, y, z, w);
				MQuaternion rotation(x, y, z, w);
				joint.getOrientation(orientation);

				keyframeOrientation = orientation.asMatrix();
				keyframeRotation = rotation.asMatrix();

				MMatrix isolatedRotation = sourceBindPoseRotation[i - 1].inverse() * keyframeRotation;

				MGlobal::viewFrame(keyframe);
				worldRotation.append(keyframeOrientation.inverse() * sourceParentMatrices[i - 1].inverse() * isolatedRotation * sourceParentMatrices[i - 1] * keyframeOrientation);
			//}
		}
	}
}

void calculateTargetSkeleton(std::vector<MObject> skeleton, int keyframe, MMatrixArray& targetBindPoseRotation, MMatrixArray& targetParentMatrices, MMatrixArray worldRotation)
{
	double x, y, z, w;
	MQuaternion orientation;
	MMatrix keyframeOrientation;
	MMatrix keyframeRotation;
	MMatrix identity;
	MStatus status;

	for (int i = 0; i < skeleton.size(); i++)
	{
		MFnIkJoint joint(skeleton[i]);

		if (i > 0)
		{
			if (keyframe == 0)
			{
				joint.getRotationQuaternion(x, y, z, w);
				MQuaternion bindPoserotation(x, y, z, w);
				targetBindPoseRotation.set(bindPoserotation.asMatrix(), i-1);

				identity.setToIdentity();
				targetParentMatrices.set(getParentMatrix(skeleton[i], identity), i-1);
			}
			//else
			//{
				joint.getRotationQuaternion(x, y, z, w);
				MQuaternion rotation(x, y, z, w);
				joint.getOrientation(orientation);

				keyframeOrientation = orientation.asMatrix();
				keyframeRotation = rotation.asMatrix();

				MGlobal::viewFrame(keyframe);
				MMatrix b = (keyframeOrientation * targetParentMatrices[i - 1] * worldRotation[i - 1] * targetParentMatrices[i - 1].inverse() * keyframeOrientation.inverse());
				MMatrix c = targetBindPoseRotation[i - 1] * b;

				MQuaternion quat; quat = c;

				double dest[4];
				quat.get(dest);

				//std::string name = joint.name().asChar();
				//std::string des = "setKey('" + name + "')";

				joint.setRotationQuaternion(dest[0], dest[1], dest[2], dest[3]);
				MGlobal::executeCommand("setKeyframe");
				//MGlobal::executePythonCommand(des.c_str());

			//}
		}
	}
}

void start()
{	
	std::vector<MObject> sourceList;
	std::vector<MObject> targetList;

	MMatrixArray sourceBindPoseRotation;
	MMatrixArray  targetBindPoseRotation;

	MMatrixArray  worldRotation;
	MMatrixArray  translatedRotation;

	MMatrixArray  sourceParentMatrices;
	MMatrixArray  targetParentMatrices;

	MStatus status;
	int nrOfKeyframes = 0;

	MSelectionList selected;
	MGlobal::getActiveSelectionList(selected);

	MObject sourceNode;
	MObject targetNode;

	selected.getDependNode(0, sourceNode);
	selected.getDependNode(1, targetNode);

	//MGlobal::executePythonCommand("setTimer('do')");

	// Load source skeleton
	loadList(sourceNode, sourceList);

	allocateMemory(sourceBindPoseRotation, targetBindPoseRotation, worldRotation, translatedRotation, sourceParentMatrices, targetParentMatrices, targetList, sourceList);

	loadList(targetNode, targetList);

	MItDependencyGraph dgIter(sourceNode, MFn::kAnimCurve, MItDependencyGraph::kUpstream, MItDependencyGraph::kBreadthFirst, MItDependencyGraph::kNodeLevel, &status);

	if (status)
	{
		MObject current = dgIter.currentItem();
		MFnAnimCurve animCurve(current, &status);

		if (status == MS::kSuccess)
		{
			nrOfKeyframes = animCurve.numKeys();
		}
	}
	else
	{
		std::cout << status.errorString() << std::endl;
	}

	for (int i = 0; i <= nrOfKeyframes; i++)
	{
		MGlobal::viewFrame(i);

		worldRotation.clear();
		translatedRotation.clear();

		double x, y, z, w;
		MQuaternion orientation;
		MMatrix keyframeOrientation;
		MMatrix keyframeRotation;

		MFnIkJoint rootObject(sourceNode);
		MVector translation = rootObject.getTranslation(MSpace::kTransform);

		rootObject.getRotationQuaternion(x, y, z, w);
		MQuaternion rotation(x, y, z, w);
		rootObject.getOrientation(orientation);

		keyframeOrientation = orientation.asMatrix();
		keyframeRotation = rotation.asMatrix();

		double rot[4];
		rotation.get(rot);

		calculateSourceSkeleton(sourceList, i, sourceBindPoseRotation, sourceParentMatrices, worldRotation);
		calculateTargetSkeleton(targetList, i, targetBindPoseRotation, targetParentMatrices, worldRotation);

		MFnIkJoint targetRoot(targetList[0]);
		targetRoot.setTranslation(translation, MSpace::kTransform);
		targetRoot.setRotateOrientation(orientation, MSpace::kTransform, true);
		targetRoot.setRotationQuaternion(rot[0], rot[1], rot[2], rot[3]);

		//MGlobal::executeCommand("setKeyframe");
		//MGlobal::executePythonCommand("setKey('targetRoot')");
	}

	//MGlobal::executePythonCommand("stopTimer('do')");

}

EXPORT MStatus initializePlugin(MObject obj) 
{		
	MStatus status;

	MFnPlugin myPlugin(obj, "Animation transfer", "1.0", "Any", &status);
	if (MFAIL(status)) 
	{
		CHECK_MSTATUS(status);
		return status;
	}  	

	/* Redirects outputs to mayas output window instead of scripting output */
	std::cout.set_rdbuf(MStreamUtils::stdOutStream().rdbuf());
	std::cerr.set_rdbuf(MStreamUtils::stdErrorStream().rdbuf());

	/* iterate through whole maya scene and check name changing */
	start();

	return status;
}
	
EXPORT MStatus uninitializePlugin(MObject obj) {
	MFnPlugin plugin(obj);

	MTimer gTimer;
	cout << "Plugin unloaded =========================" << endl;
		
	std::cout << "REMOVING CALLBACKS: " << std::endl;

	gTimer.endTimer();
	gTimer.clear();

	return MS::kSuccess;
}