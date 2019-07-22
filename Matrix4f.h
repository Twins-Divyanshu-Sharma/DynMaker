#ifndef MATRIX_4F
#define MATRIX_4F

#include "Vector4f.h"

class Matrix4f
{	
private:
	Vector4f $i$, $j$, $k$, $h$;
public:


	Matrix4f();
	~Matrix4f();
	
	// mathematics
	void operator=(Matrix4f);
	Vector4f operator*(Vector4f);
	Matrix4f operator*(Matrix4f);
	void identity();

	// conditional
	bool operator==(Matrix4f);
	bool operator!=(Matrix4f);

	// indexing
	Vector4f& operator[](int);


	// output
	float* operator!();
	friend std::ostream& operator<< (std::ostream&, Matrix4f);

	// set rotation
	void setRotation(Vector3f&);

};

#endif

