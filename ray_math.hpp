#pragma once
#include <cmath>
#include <cassert>

// we use a right-handed coordinate system (yes, this is going to confuse the hell out of me :)

//  +y
//  |  -z
//  | /
//  |/
//  +------ +x
//

struct Vec3
{
	Vec3() {}
	Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
	union {
		struct {
			float x, y, z;
		};
		float d[3];
	};

  float operator[](int idx) const { assert(idx >= 0 && idx < 3); return d[idx];}
  float& operator[](int idx) { assert(idx >= 0 && idx < 3); return d[idx];}

	inline float len() const;

	const float static kEps;
};

struct Matrix3x3
{
	Matrix3x3() {}
	Matrix3x3(float m11, float m12, float m13, float m21, float m22, float m23, float m31, float m32, float m33)
		: m11(m11), m12(m12), m13(m13), m21(m21), m22(m22), m23(m23), m31(m31), m32(m32), m33(m33) {}

	union {
		struct {
			float m11, m12, m13;
			float m21, m22, m23;
			float m31, m32, m33;
		};
		float d[3*3];
	};
};

inline Vec3 operator*(const Vec3& v, const Matrix3x3& m)
{
	return Vec3(
		v.x * m.m11 + v.y * m.m12 + v.z * m.m13,
		v.x * m.m21 + v.y * m.m22 + v.z * m.m23,
		v.x * m.m31 + v.y * m.m32 + v.z * m.m33
		);
}

inline Matrix3x3 rotate_x(float q)
{
	const float sin_q = sinf(q);
	const float cos_q = cosf(q);
	return Matrix3x3(
		1, 0, 0,
		0, cos_q, sin_q,
		0, -sin_q, cos_q
		);
}

inline Matrix3x3 rotate_y(float q)
{
	const float sin_q = sinf(q);
	const float cos_q = cosf(q);
	return Matrix3x3(
		cos_q, 0, -sin_q,
		0, 1, 0,
		sin_q, 0, cos_q
		);
}

inline Matrix3x3 rotate_z(float q)
{
	const float sin_q = sinf(q);
	const float cos_q = cosf(q);
	return Matrix3x3(
		cos_q, sin_q, 0,
		-sin_q, cos_q, 0,
		0, 0, 1
		);
}

inline float Vec3::len() const
{
	return sqrtf(x*x + y*y + z*z);
}

inline Vec3 operator+(const Vec3& a, const Vec3& b)
{
	return Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

inline Vec3 operator-(const Vec3& a, const Vec3& b)
{
	return Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline Vec3 operator-(const Vec3& a)
{
	return Vec3(-a.x, -a.y, -a.z);
}

inline Vec3 operator/(const Vec3& a, float s)
{
	return Vec3(a.x / s, a.y / s, a.z / s);
}

inline Vec3 operator*(float s, const Vec3& a)
{
	return Vec3(s * a.x, s * a.y, s * a.z);
}

inline Vec3 normalize(const Vec3& a)
{
	Vec3 res;
	const float len = a.len();
	if (len < Vec3::kEps)
		res.x = res.y = res.z = 0;
	else
		res = a / len;
	return res;
}

inline float dot(const Vec3& a, const Vec3& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 cross(const Vec3& a, const Vec3& b)
{
	return Vec3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}
