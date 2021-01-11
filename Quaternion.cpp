#include "Quaternion.hpp"
#include <math.h>
Quaternion::Quaternion(const float euler_angle[3])
{
    float cosPhi_2 = cosf(euler_angle[0] / 2.0f);
    float sinPhi_2 = sinf(euler_angle[0] / 2.0f);
    float cosTheta_2 = cosf(euler_angle[1] / 2.0f);
    float sinTheta_2 = sinf(euler_angle[1] / 2.0f);
    float cosPsi_2 = cosf(euler_angle[2] / 2.0f);
    float sinPsi_2 = sinf(euler_angle[2] / 2.0f);

    this->w =
        (cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2);
    this->x =
        (sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2);
    this->y =
        (cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2);
    this->z =
        (cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2);
}

Quaternion::Quaternion(const float w[3], const float dt)
{
    float norm_w = sqrtf(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    float theta = norm_w * dt;

    if (theta > 0.03f)
    {
        this->w = cosf(theta / 2);
        this->x = (w[0] / norm_w) * sinf(theta / 2);
        this->y = (w[1] / norm_w) * sinf(theta / 2);
        this->z = (w[2] / norm_w) * sinf(theta / 2);
    }
    else
    {
        this->w = cosf(theta / 2);
        this->x = w[0] * (dt / 2);
        this->y = w[1] * (dt / 2);
        this->z = w[2] * (dt / 2);
    }
}

Quaternion Quaternion::fromTwoVector(Quaternion q1, Quaternion q2)
{
    q1.normalize();
    q2.normalize();
    Quaternion v0 = q1;
    Quaternion v1 = q2;
    float c = v0.x * v1.x + v0.y * v1.y + v0.z * v1.z;

    if (c < -1.0f + 1e-4)
    {
        return Quaternion(1.0, 0, 0, 0);
    }
    Quaternion axis;
    axis.w = 0.0f;
    axis.x = v0.y * v1.z - v0.z * v1.y;
    axis.y = v0.z * v1.x - v0.x * v1.z;
    axis.z = v0.x * v1.y - v0.y * v1.x;
    float s = sqrtf((1.0f + c) * 2.0f); //2cos(theta/2)

    float invs = 1.0f / s;
    axis = axis * invs;
    axis.w = s * 0.5f;
    axis.normalize();
    return axis;
}

Quaternion Quaternion::dot(Quaternion v0, Quaternion v1)
{
    Quaternion axis;
    axis.w = 0.0f;
    axis.x = v0.y * v1.z - v0.z * v1.y;
    axis.y = v0.z * v1.x - v0.x * v1.z;
    axis.z = v0.x * v1.y - v0.y * v1.x;
    return axis;
}

float Quaternion::norm()
{
    return sqrtf(this->w * this->w + this->x * this->x + this->y * this->y +
                 this->z * this->z);
}

void Quaternion::normalize(void)
{
    float norm = this->norm();
    if (norm > 0.000001f)
    {
        this->w /= norm;
        this->x /= norm;
        this->y /= norm;
        this->z /= norm;
    }
    // TODO:
    // ANCHOR  need to change norm to be invNorm
}

float Quaternion::invNorm(void)
{
    float num = this->w * this->w + this->x * this->x + this->y * this->y +
                this->z * this->z;

    float halfnum = 0.5f * num;
    float temp = num;
    long i = *(long *)&temp;
    i = 0x5f3759df - (i >> 1);
    temp = *(float *)&i;
    temp = temp * (1.5f - (halfnum * temp * temp));
    return temp;
}

void Quaternion::set(float w, float x, float y, float z)
{
    this->w = w;
    this->x = x;
    this->y = y;
    this->z = z;
}

void Quaternion::toEulerAngle(float *roll, float *pitch,
                              float *yaw) const
{
    // roll (x-axis rotation)
    float sinr_cosp = +2.0f * (w * x + y * z);
    float cosr_cosp = +1.0f - 2.0f * (x * x + y * y);
    *roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = +2.0f * (w * y - z * x);
    if (fabs(sinp) >= 1)
        *pitch = copysign(M_PI / 2.0f, sinp); // use 90 degrees if out of range
    else
        *pitch = asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = +2.0f * (w * z + x * y);
    float cosy_cosp = +1.0f - 2.0f * (y * y + z * z);
    *yaw = atan2(siny_cosp, cosy_cosp);
}

Quaternion Quaternion::rotate(Quaternion v) const
{
    float result[3];

    float ww = w * w;
    float xx = x * x;
    float yy = y * y;
    float zz = z * z;
    float wx = w * x;
    float wy = w * y;
    float wz = w * z;
    float xy = x * y;
    float xz = x * z;
    float yz = y * z;

    // Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
    // p2.x = w*w*p1.x + 2*y*w*p1.z - 2*z*w*p1.y + x*x*p1.x + 2*y*x*p1.y + 2*z*x*p1.z - z*z*p1.x - y*y*p1.x;
    // p2.y = 2*x*y*p1.x + y*y*p1.y + 2*z*y*p1.z + 2*w*z*p1.x - z*z*p1.y + w*w*p1.y - 2*x*w*p1.z - x*x*p1.y;
    // p2.z = 2*x*z*p1.x + 2*y*z*p1.y + z*z*p1.z - 2*w*y*p1.x - y*y*p1.z + 2*w*x*p1.y - x*x*p1.z + w*w*p1.z;

    result[0] = ww * v.x + 2 * wy * v.z - 2 * wz * v.y +
                xx * v.x + 2 * xy * v.y + 2 * xz * v.z -
                zz * v.x - yy * v.x;
    result[1] = 2 * xy * v.x + yy * v.y + 2 * yz * v.z +
                2 * wz * v.x - zz * v.y + ww * v.y -
                2 * wx * v.z - xx * v.y;
    result[2] = 2 * xz * v.x + 2 * yz * v.y + zz * v.z -
                2 * wy * v.x - yy * v.z + 2 * wx * v.y -
                xx * v.z + ww * v.z;

    return Quaternion(0.0, result[0], result[1], result[2]);
}

Quaternion Quaternion::slerp(Quaternion q1, Quaternion q2, float t)
{
    const float one = 1.0f - 1e-4;
    float d = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
    float absD = fabs(d);

    float scale0;
    float scale1;

    if (absD >= one)
    {
        scale0 = 1.0f - t;
        scale1 = t;
    }
    else
    {
        // theta is the angle between the 2 quaternions
        float theta = acos(absD);
        float sinTheta = sin(theta);

        scale0= sin((1 - t) * theta) / sinTheta;
        scale1= sin((t * theta)) / sinTheta;
    }
    if (d < 0)
        scale1 = -scale1;
    Quaternion result = q1*scale0 + q2*scale1;
    return result;
}

Quaternion operator+(const Quaternion &lhs, const Quaternion &rhs)
{
    Quaternion sum = lhs;
    sum += rhs;
    return sum;
}

Quaternion operator*(const Quaternion &lhs, const Quaternion &rhs)
{
    Quaternion mult = lhs;
    mult *= rhs;
    return mult;
}

Quaternion operator*(const Quaternion &lhs, const float scalar)
{
    Quaternion result = lhs;
    result *= scalar;
    return result;
}

Quaternion operator/(const Quaternion &lhs, const float scalar)
{
    Quaternion result = lhs;
    result /= scalar;
    return result;
}

Quaternion operator*(const float scalar, const Quaternion &rhs)
{
    Quaternion result = rhs;
    return result *= scalar;
}
