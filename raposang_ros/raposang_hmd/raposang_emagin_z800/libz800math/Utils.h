#pragma once

#include "Vec3.h"
#include "Quat.h"

bool equivalent(double a, double b, double e);

float clampUnity(float x);

Vec3 convertQuatToTaitBryan(const Quat &q);

Quat convertTaitBryanToQuat(const Vec3 &xyz);
