
#include "../geometry/spline.h"

template<typename T> T Spline<T>::at(float time) const {

	// A4T1b: Evaluate a Catumull-Rom spline

	// Given a time, find the nearest positions & tangent values
	// defined by the control point map.

	// Transform them for use with cubic_unit_spline

	// Be wary of edge cases! What if time is before the first knot,
	// before the second knot, etc...
	//edge cases
	if (knots.empty()) return T();
	if (knots.size() == 1) return knots.begin()->second;
	if (time <= knots.begin()->first) return knots.begin()->second;
	if (time >= std::prev(knots.end())->first) return std::prev(knots.end())->second;

	auto k2 = knots.upper_bound(time);
	auto k1 = std::prev(k2);
 
	float t1 = k1->first;
	float t2 = k2->first;
	T p1 = k1->second;
	T p2 = k2->second;

	float t0;
	T p0;
	if (k1 != knots.begin()) {
		auto k0 = std::prev(k1);
		t0 = k0->first;
		p0 = k0->second;
	} else {
		t0 = t1 - (t2 - t1);
		p0 = p1 - (p2 - p1);
	}

	float t3;
	T p3;
	auto k3 = std::next(k2);
	if (k3 != knots.end()) {
		t3 = k3->first;
		p3 = k3->second;
	} else {
		t3 = t2 + (t2 - t1);
		p3 = p2 + (p2 - p1);
	}
 
	T m0 = (p2 - p0) * (1.0f / (t2 - t0));
	T m1 = (p3 - p1) * (1.0f / (t3 - t1));
	float u = (time - t1) / (t2 - t1);
	float dt = t2 - t1;

	return cubic_unit_spline(u, p1, p2, m0 * dt, m1 * dt);
}

template<typename T>
T Spline<T>::cubic_unit_spline(float time, const T& position0, const T& position1,
                               const T& tangent0, const T& tangent1) {

	// A4T1a: Hermite Curve over the unit interval

	// Given time in [0,1] compute the cubic spline coefficients and use them to compute
	// the interpolated value at time 'time' based on the positions & tangents

	// Note that Spline is parameterized on type T, which allows us to create splines over
	// any type that supports the * and + operators.
	float t  = time;
	float t2 = t * t;
	float t3 = t2 * t;

	float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
	float h10 = t3 - 2.0f * t2 + t;
	float h01 = -2.0f * t3 + 3.0f * t2;
	float h11 = t3 - t2;

	return h00 * position0 + h10 * tangent0 + h01 * position1 + h11 * tangent1;
}

template class Spline<float>;
template class Spline<double>;
template class Spline<Vec4>;
template class Spline<Vec3>;
template class Spline<Vec2>;
template class Spline<Mat4>;
template class Spline<Spectrum>;
