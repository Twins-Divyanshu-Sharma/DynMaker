#ifndef HYPERSPHERE_UNIT_RADIUS
#define HYPERSPHERE_UNIT_RADIUS

#include "Matrix4f.h"

class Quaternion
{
private :
	float $r$, $i$, $j$, $k$;
public:
	// constructors
	Quaternion();
	Quaternion(float, float, float, float);
	Quaternion(float angle, Vector3f axis);
	
	// calculations
	float length();
	void normalize();
	Quaternion operator*(Quaternion);
	Quaternion operator-();

	// conversion
	Matrix4f toMatrix4f();
	Vector3f toVec3();
	void operator=(Quaternion);
	

	// output
	friend std::ostream& operator << (std::ostream&, Quaternion);
	friend Quaternion toQuaternion(Matrix4f&);
};

#endif