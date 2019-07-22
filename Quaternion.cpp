#include "Quaternion.h"

Quaternion::Quaternion() : $r$(1), $i$(0), $j$(0), $k$(0)
{}

Quaternion::Quaternion(float w, float x, float y, float z) : $r$(w), $i$(x), $j$(y), $k$(z)
{}

Quaternion::Quaternion(float angle , Vector3f axis)
{
	float radAngle = (PI / 180.0f)*angle;
	float w = std::cos(radAngle / 2.0f);
	Vector3f vec = std::sin(radAngle / 2.0f) * axis.dirtection();
	$r$ = w;
	$i$ = vec.x();
	$j$ = vec.y();
	$k$ = vec.z();
}



Quaternion Quaternion::operator*(Quaternion quat)
{
	Vector3f victim(quat.$i$, quat.$j$, quat.$k$);
	Vector3f action($i$, $j$, $k$);
	float victimw = quat.$r$;
	float actionw = $r$;

	float resw = (victimw * actionw) - (action * victim);
	Vector3f resvec = (victimw * action) + (actionw * victim) + (action ^ victim);

	return Quaternion(resw, resvec.x(), resvec.y(), resvec.z());
}

Quaternion Quaternion::operator-()
{
	return Quaternion($r$, -$i$, -$j$, -$k$);
}

float Quaternion::length()
{
	return (float)(std::sqrt($r$*$r$ + $i$ * $i$ + $j$ * $j$ + $k$ * $k$));
}

void Quaternion::normalize()
{
	double length = std::sqrt($r$*$r$ + $i$ * $i$ + $j$ * $j$ + $k$ * $k$);
	$r$ = $r$ / length;
	$i$ = $i$ / length;
	$j$ = $j$ / length;
	$k$ = $k$ / length;
}



Matrix4f Quaternion::toMatrix4f()
{
	Matrix4f mat;
	mat[0][0] = 1 - (2 * $j$*$j$) - (2 * $k$*$k$);		mat[1][0] = 2 * $i$ * $j$ - 2 * $r$ * $k$;		mat[2][0] = 2 * $i$ * $k$ + 2 * $r$ * $j$;
	mat[0][1] = 2 * $i$*$j$ + 2 * $r$*$k$;				mat[1][1] = 1 - 2 * $i$*$i$ - 2 * $k$*$k$;		mat[2][1] = 2 * $j$ * $k$ - 2 * $r$ * $i$;
	mat[0][2] = 2 * $i$*$k$ - 2 * $r$*$j$;				mat[1][2] = 2 * $j$ * $k$ + 2 * $r$ * $i$;		mat[2][2] = 1 - 2*$i$*$i$ - 2*$j$ *$j$;

	return mat;
	
}

Vector3f Quaternion::toVec3()
{
	return Vector3f($i$, $j$, $k$);
}

void Quaternion::operator=(Quaternion q)
{
	$r$ = q.$r$;
	$i$ = q.$i$;
	$j$ = q.$j$; 
	$k$ = q.$k$;
}

std::ostream & operator<<(std::ostream & os, Quaternion q)
{
	//os << " [ " << q.$r$ << " , " << q.$r$ << " , " << q.$r$ << " , " << q.$r$ << " ] ";
	os << q.$r$ << " " << q.$i$ << " " << q.$j$ << " " << q.$k$;
	return os;
}

Quaternion toQuaternion(Matrix4f& matrix)
{
	float sq[4];
	// 11 =0 22=5 33=10
	sq[0] = matrix[0][0] + matrix[1][1] + matrix[2][2];
	sq[1] = matrix[0][0] - matrix[1][1] - matrix[2][2];
	sq[2] = -matrix[0][0] + matrix[1][1] - matrix[2][2];
	sq[3] = -matrix[0][0] - matrix[1][1] + matrix[2][2];
	int gIndex = -1;	float g = -99999;
	for (int i = 0; i < 4; i++)
	{
		if (sq[i] > g)
		{
			g = sq[i];
			gIndex = i;
		}
	}
	float w, x, y, z;
	switch (gIndex)
	{
	case 0:	// w greatest
		w = std::sqrt(sq[0] + 1) / 2;
		x = (matrix[1][2] - matrix[2][1]) / (4 * w);
		y = (matrix[2][0] - matrix[0][2]) / (4 * w);
		z = (matrix[0][1] - matrix[1][0]) / (4 * w);
		break;
	case 1: // x greatest
		x = std::sqrt(sq[1] + 1) / 2;
		w = (matrix[1][2] - matrix[2][1]) / (4 * x);
		y = (matrix[0][1] + matrix[1][0]) / (4 * x);
		z = (matrix[0][2] + matrix[2][0]) / (4 * x);
		break;
	case 2:	// y greatest
		y = std::sqrt(sq[2] + 1) / 2;
		w = (matrix[2][0] - matrix[0][2]) / (4 * y);
		x = (matrix[1][0] + matrix[0][1]) / (4 * y);
		z = (matrix[2][1] + matrix[1][2]) / (4 * y);
		break;
	case 3:
		z = std::sqrt(sq[3] + 1) / 2;
		w = (matrix[0][1] - matrix[1][0]) / (4 * z);
		x = (matrix[2][0] + matrix[0][2]) / (4 * z);
		y = (matrix[2][1] + matrix[1][2]) / (4 * z);
		break;
	}

	return Quaternion(w, x, y, z);
}
