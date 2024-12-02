#include "helper_3dmath.h"
#include <cmath>


Quaternion::Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}

Quaternion::Quaternion(float nw, float nx, float ny, float nz) : w(nw), x(nx), y(ny), z(nz) {}

Quaternion Quaternion::getProduct(const Quaternion q) const {
    // Quaternion multiplication is defined by:
    //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
    //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
    //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
    //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
    return Quaternion{
        w * q.w - x * q.x - y * q.y - z * q.z,  // new w
        w * q.x + x * q.w + y * q.z - z * q.y,  // new x
        w * q.y - x * q.z + y * q.w + z * q.x,  // new y
        w * q.z + x * q.y - y * q.x + z * q.w   // new z
    };
}

Quaternion Quaternion::getConjugate() const {
    return Quaternion{ w, -x, -y, -z };
}

float Quaternion::getMagnitude() const {
    return sqrt(w * w + x * x + y * y + z * z);
}

void Quaternion::normalize() {
    float m = getMagnitude();
    w /= m;
    x /= m;
    y /= m;
    z /= m;
}

Quaternion Quaternion::getNormalized() const {
    Quaternion r(w, x, y, z);
    r.normalize();
    return r;
}

Quaternion Quaternion::operator*(const Quaternion& q) const
{
    return Quaternion{
		w * q.w - x * q.x - y * q.y - z * q.z,
		w * q.x + x * q.w + y * q.z - z * q.y,
		w * q.y - x * q.z + y * q.w + z * q.x,
		w * q.z + x * q.y - y * q.x + z * q.w
    };
}


VectorFloat::VectorFloat() {
    x = 0;
    y = 0;
    z = 0;
}

VectorFloat::VectorFloat(float nx, float ny, float nz) {
    x = nx;
    y = ny;
    z = nz;
}

float VectorFloat::getMagnitude() const {
    return sqrt(x * x + y * y + z * z);
}

void VectorFloat::normalize() {
    float m = getMagnitude();
    x /= m;
    y /= m;
    z /= m;
}

VectorFloat VectorFloat::getNormalized() const{
    VectorFloat r(x, y, z);
    r.normalize();
    return r;
}

void VectorFloat::rotate(Quaternion* q) {
    Quaternion p(0, x, y, z);

    // quaternion multiplication: q * p, stored back in p
    p = q->getProduct(p);

    // quaternion multiplication: p * conj(q), stored back in p
    p = p.getProduct(q->getConjugate());

    // p quaternion is now [0, x', y', z']
    x = p.x;
    y = p.y;
    z = p.z;
}

VectorFloat VectorFloat::getRotated(Quaternion* q) {
    VectorFloat r(x, y, z);
    r.rotate(q);
    return r;
}
