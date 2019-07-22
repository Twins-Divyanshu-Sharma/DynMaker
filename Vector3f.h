#ifndef VECTOR_3F
#define VECTOR_3F

#include <iostream>
#include <string>
#include <cmath>
#define PI 3.14159265359f

class Vector3f
{
private:
	float $x$, $y$, $z$;
public:
	Vector3f();
	Vector3f(float, float, float);
	~Vector3f();

	// explicit getters setters
	float x();	void x(float);
	float y();	void y(float);
	float z();	void z(float);

	
	//mathematical
	void operator=(Vector3f);
	Vector3f operator+(Vector3f);
	Vector3f operator-(Vector3f);
	float operator*(Vector3f);  // dot product
	Vector3f operator^(Vector3f);  //  cross product
	friend Vector3f operator*(float, Vector3f);	//scalar multiplication only from front
	float magnitude();	// magnitude
	Vector3f dirtection();	// dirtection
	
	//conditionals
	bool operator==(Vector3f);
	bool operator!=(Vector3f);


	//outputs
	friend std::ostream& operator<<(std::ostream&, Vector3f);	 // to console
	float* operator!();	// to float* for glUniform
};

#endif
