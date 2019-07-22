#include "Matrix4f.h"

Matrix4f::Matrix4f()
{
	this->identity();
}

Matrix4f::~Matrix4f()
{
}

void Matrix4f::operator=(Matrix4f mat)
{
	$i$.x(mat.$i$.x());		$j$.x(mat.$j$.x());		$k$.x(mat.$k$.x());		$h$.x(mat.$h$.x());
	$i$.y(mat.$i$.y());		$j$.y(mat.$j$.y());		$k$.y(mat.$k$.y());		$h$.y(mat.$h$.y());
	$i$.z(mat.$i$.z());		$j$.z(mat.$j$.z());		$k$.z(mat.$k$.z());		$h$.z(mat.$h$.z());
	$i$.w(mat.$i$.w());		$j$.w(mat.$j$.w());		$k$.w(mat.$k$.w());		$h$.w(mat.$h$.w());

}

Vector4f Matrix4f::operator*(Vector4f vec)
{
	Vector4f ret = (vec.x() * $i$) + (vec.y() * $j$) + (vec.z() * $k$) + (vec.w() * $h$);
	return ret;
}

Matrix4f Matrix4f::operator*(Matrix4f mat)
{
	Matrix4f ret;
	ret.$i$ = this->operator*(mat.$i$);
	ret.$j$ = this->operator*(mat.$j$);
	ret.$k$ = this->operator*(mat.$k$);
	ret.$h$ = this->operator*(mat.$h$);
	return ret;
}

void Matrix4f::identity()
{
	Vector4f i(1, 0, 0, 0), j(0, 1, 0, 0), k(0, 0, 1, 0), h(0, 0, 0, 1);
	$i$ = i;
	$j$ = j;
	$k$ = k;
	$h$ = h;
}
bool Matrix4f::operator==(Matrix4f mat)
{
	return ($i$ == mat.$i$) && ($j$ == mat.$j$) && ($k$ == mat.$k$) && ($h$ == mat.$h$);
}

bool Matrix4f::operator!=(Matrix4f mat)
{
	return ($i$ != mat.$i$) || ($j$ != mat.$j$) || ($k$ != mat.$k$) || ($h$ != mat.$h$);
}

Vector4f & Matrix4f::operator[](int b)
{
	switch (b)
	{
	case 0:
		return $i$;
		break;
	case 1:
		return $j$;
		break;
	case 2:
		return $k$;
		break;
	case 3:
		return $h$;
		break;
	default:
		std::cout << " Index for basis vectors must belong to [0,3] for this matrix " << std::endl;
		break;
	}
}

float * Matrix4f::operator!()
{
	float* ret = new float[16];
	int k = 0;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			ret[k] = this->operator[](i)[j];
			k++;
		}
	}
	return ret;
}

void Matrix4f::setRotation(Vector3f & rotation)
{
	if (rotation == Vector3f(0, 0, 0))
	{
		this->identity();
		return;
	}


	float conv = PI / 180.0f;
	float radx = rotation.x()*conv, rady = rotation.y()*conv, radz = rotation.z()*conv;
	float cx = std::cos(radx), sx = std::sin(radx);
	float cy = std::cos(rady), sy = std::sin(rady);
	float cz = std::cos(radz), sz = std::sin(radz);

	this->operator[](0)[0] = cy * cz;					this->operator[](1)[0] = -sz*cy;					this->operator[](2)[0] = sy;		this->operator[](3)[0] = 0;
	this->operator[](0)[1] = cx*sz + cz*sy*sx;			this->operator[](1)[1] = cx*cz-sx*sy*sz;			this->operator[](2)[1] = -cy*sx;	this->operator[](3)[1] = 0;
	this->operator[](0)[2] = sz * sx - cx * cz*sy;		this->operator[](1)[2] = cz * sx + sz * sy*cx;		this->operator[](2)[2] = cx * cy;	this->operator[](3)[2] = 0;
	this->operator[](0)[3] = 0;							this->operator[](1)[3] = 0;							this->operator[](2)[3] = 0;			this->operator[](3)[3] = 1;
}

std::ostream & operator<<(std::ostream & os, Matrix4f mat)
{
	os << " | " << mat[0][0] << "\t" << mat[1][0] << "\t" << mat[2][0] << "\t" << mat[3][0] << "  | \n"
	   << " | " << mat[0][1] << "\t" << mat[1][1] << "\t" << mat[2][1] << "\t" << mat[3][1] << "  | \n"
	   << " | " << mat[0][2] << "\t" << mat[1][2] << "\t" << mat[2][2] << "\t" << mat[3][2] << "  | \n"
	   << " | " << mat[0][3] << "\t" << mat[1][3] << "\t" << mat[2][3] << "\t" << mat[3][3] << "  | \n";
	return os;
}
