#include "Vector4f.h"

Vector4f::Vector4f() : $x$(0), $y$(0), $z$(0), $w$(0)
{
}

Vector4f::Vector4f(float x, float y, float z, float w) : $x$(x), $y$(y), $z$(z), $w$(w)
{}

Vector4f::Vector4f(Vector3f vec, float w) : $x$(vec.x()), $y$(vec.y()), $z$(vec.z()), $w$(w)
{}

Vector4f::~Vector4f()
{
}

float Vector4f::x()
{
	return $x$;
}

void Vector4f::x(float x)
{
	$x$ = x;
}

float Vector4f::y()
{
	return $y$;
}

void Vector4f::y(float y)
{
	$y$ = y;
}
float Vector4f::z()
{
	return $z$;
}

void Vector4f::z(float z)
{
	$z$ = z;
}

float Vector4f::w()
{
	return $w$;
}

void Vector4f::w(float w)
{
	$w$ = w;
}

void Vector4f::operator=(Vector4f vec)
{
	$x$ = vec.$x$;
	$y$ = vec.$y$;
	$z$ = vec.$z$;
	$w$ = vec.$w$;
}

Vector4f Vector4f::operator+(Vector4f vec)
{
	return Vector4f($x$ + vec.$x$, $y$ + vec.$y$, $z$ + vec.$z$, $w$ + vec.$w$);
}

Vector4f Vector4f::operator-(Vector4f vec)
{
	return Vector4f($x$ - vec.$x$, $y$ - vec.$y$, $z$ - vec.$z$, $w$ - vec.$w$);
}

float Vector4f::operator*(Vector4f vec)
{
	return ($x$ * vec.$x$) + ($y$ * vec.$y$) + ($z$ * vec.$z$) + ($w$ * vec.$w$);
}

bool Vector4f::operator==(Vector4f vec)
{
	return ($x$ == vec.$x$) && ($y$ == vec.$y$) && ($z$ == vec.$z$) && ($w$ == vec.$w$) ;
}

bool Vector4f::operator!=(Vector4f vec)
{
	return ($x$ != vec.$x$) || ($y$ != vec.$y$) || ($z$ != vec.$z$) || ($w$ != vec.$w$);
}

float * Vector4f::operator!()
{
	float* ret = new float[4];
	ret[0] = $x$; ret[1] = $y$; ret[2] = $z$; ret[3] = $w$;
	return ret;
}

float & Vector4f::operator[](int n)
{
		switch (n)
		{
		case 0:
			return $x$;
			break;
		case 1:
			return $y$;
			break;
		case 2:
			return $z$;
			break;
		case 3:
			return $w$;
			break;
		default:
			std::cerr << "\a Index must be belong to [0, 3] " << std::endl;
			break;
		};

}

Vector4f operator*(float scalar, Vector4f vec)
{
	return Vector4f(scalar * vec.$x$ , scalar * vec.$y$, scalar * vec.$z$, scalar * vec.$w$);
}

std::ostream & operator<<(std::ostream & os, Vector4f vec)
{
	os << " [ " << vec.$x$ << " , " << vec.$y$ << " , " << vec.$z$ << " , " << vec.$w$ << " ] ";
	return os;
}
