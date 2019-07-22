#ifndef ANIMATION_DATA_STRUCT
#define ANIMATION_DATA_STRUCT
#include <vector>
#include "Quaternion.h"

struct KeyFrames
{
	Matrix4f* jData;
	Quaternion* rotation;
	Vector3f* translate;
};

struct Animations
{
	int noOfKeyFrames;
	KeyFrames* frames;
};

struct Animate
{
	int noOfAnimations;
	std::vector<Animations> animations;
};

#endif

