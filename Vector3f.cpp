#include "Vector3f.h"

Vector3f::Vector3f() : $x$(0), $y$(0), $z$(0)	// initialize to zero
{
}

Vector3f::Vector3f(float x, float y, float z) : $x$(x) , $y$(y) , $z$(z)	//memeber init
{
}

Vector3f::~Vector3f()
{
}

float Vector3f::x()
{
	return $x$;
}

void Vector3f::x(float x)
{
	$x$ = x;
}

float Vector3f::y()
{
	return $y$;
}

void Vector3f::y(float y)
{
	$y$ = y;
}
float Vector3f::z()
{
	return $z$;
}

void Vector3f::z(float z)
{
	$z$ = z;
}

void Vector3f::operator=(Vector3f vec)
{
	$x$ = vec.$x$;   $y$ = vec.$y$; $z$ = vec.$z$;
}

Vector3f Vector3f::operator+(Vector3f vec)
{
	return Vector3f($x$ + vec.$x$, $y$ + vec.$y$, $z$ + vec.$z$);
}

Vector3f Vector3f::operator-(Vector3f vec)
{
	return Vector3f($x$ - vec.$x$, $y$ - vec.$y$, $z$ - vec.$z$);;
}

float Vector3f::operator*(Vector3f vec)
{
	return ($x$*vec.$x$) + ($y$*vec.$y$) + ($z$*vec.$z$);
}

Vector3f Vector3f::operator^(Vector3f vec)
{
	float resx = $y$ * vec.$z$ - $z$ * vec.$y$;
	float resy = $z$ * vec.$x$ - $x$ * vec.$z$;
	float resz = $x$ * vec.$y$ - $y$ * vec.$x$;
	return Vector3f(resx, resy, resz);
}

float Vector3f::magnitude()
{
	return std::sqrtf($x$*$x$ + $y$*$y$ + $z$*$z$);
}

Vector3f Vector3f::dirtection()
{
	float magn = this->magnitude();
	return Vector3f($x$ / magn, $y$ / magn, $z$ / magn);
}


bool Vector3f::operator==(Vector3f vec)
{
	return ($x$ == vec.$x$ ) && ( $y$ == vec.$y$) && ( $z$ == vec.$z$);
}

bool Vector3f::operator!=(Vector3f vec)
{
	return ($x$ != vec.$x$) || ($y$ != vec.$y$) || ( $z$ != vec.$z$) ;
}

float * Vector3f::operator!()
{
	float* ret = new float[3];
	ret[0] = $x$; ret[1] = $y$; ret[2] = $z$;
	return ret;
}

Vector3f operator*(float scalar, Vector3f vec)
{
	return Vector3f(scalar*vec.$x$, scalar*vec.$y$, scalar*vec.$z$);
}

std::ostream & operator<<(std::ostream & os, Vector3f vec)
{
	//os << " [ " << vec.$x$ << " , " << vec.$y$ << " , " << vec.$z$ << " ] ";
	os << vec.$x$ << " " << vec.$y$ << " " << vec.$z$;
	return os;
}
