#ifndef VECTOR_4F
#define VECTOR_4F

#include "Vector3f.h"

class Vector4f
{
private:
	float $x$, $y$, $z$, $w$;
public:
	Vector4f();
	Vector4f(float, float, float, float);
	Vector4f(Vector3f, float);
	~Vector4f();

	// getters and setters
	float x(); void x(float);
	float y(); void y(float);
	float z(); void z(float);
	float w(); void w(float);

	// mathematical
	void operator=(Vector4f);
	Vector4f operator+(Vector4f);
	Vector4f operator-(Vector4f);
	float operator*(Vector4f);
	friend Vector4f operator*(float, Vector4f);	//scalar multiplication only from front

	// conditional
	bool operator==(Vector4f);
	bool operator!=(Vector4f);

	// output
	friend std::ostream& operator<< (std::ostream&, Vector4f);	//to console
	float* operator!();	//to float* for glUniform		

	// indexing
	float& operator[](int);

};

#endif
