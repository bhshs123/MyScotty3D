
#include "samplers.h"
#include "../util/rand.h"
#include "../scene/shape.h"

constexpr bool IMPORTANCE_SAMPLING = true;

namespace Samplers {

Vec2 Rect::sample(RNG &rng) const {
	//A3T1 - step 2 - supersampling
    // Return a point selected uniformly at random from the rectangle [0,size.x)x[0,size.y)
    // Useful function: rng.unit()
    return Vec2(rng.unit() * size.x, rng.unit() * size.y); 
}

float Rect::pdf(Vec2 at) const {
	if (at.x < 0.0f || at.x > size.x || at.y < 0.0f || at.y > size.y) return 0.0f;
	return 1.0f / (size.x * size.y);
}

Vec2 Circle::sample(RNG &rng) const {
	//A3EC - bokeh - circle sampling

    // Return a point selected uniformly at random from a circle defined by its
	// center and radius.
    // Useful function: rng.unit()

    return Vec2{};
}

float Circle::pdf(Vec2 at) const {
	//A3EC - bokeh - circle pdf

	// Return the pdf of sampling the point 'at' for a circle defined by its
	// center and radius.

    return 1.f;
}

Vec3 Point::sample(RNG &rng) const {
	return point;
}

float Point::pdf(Vec3 at) const {
	return at == point ? 1.0f : 0.0f;
}

Vec3 Triangle::sample(RNG &rng) const {
	float u = std::sqrt(rng.unit());
	float v = rng.unit();
	float a = u * (1.0f - v);
	float b = u * v;
	return a * v0 + b * v1 + (1.0f - a - b) * v2;
}

float Triangle::pdf(Vec3 at) const {
	float a = 0.5f * cross(v1 - v0, v2 - v0).norm();
	float u = 0.5f * cross(at - v1, at - v2).norm() / a;
	float v = 0.5f * cross(at - v2, at - v0).norm() / a;
	float w = 1.0f - u - v;
	if (u < 0.0f || v < 0.0f || w < 0.0f) return 0.0f;
	if (u > 1.0f || v > 1.0f || w > 1.0f) return 0.0f;
	return 1.0f / a;
}

Vec3 Hemisphere::Uniform::sample(RNG &rng) const {

	float Xi1 = rng.unit();
	float Xi2 = rng.unit();

	float theta = std::acos(Xi1);
	float phi = 2.0f * PI_F * Xi2;

	float xs = std::sin(theta) * std::cos(phi);
	float ys = std::cos(theta);
	float zs = std::sin(theta) * std::sin(phi);

	return Vec3(xs, ys, zs);
}

float Hemisphere::Uniform::pdf(Vec3 dir) const {
	if (dir.y < 0.0f) return 0.0f;
	return 1.0f / (2.0f * PI_F);
}

Vec3 Hemisphere::Cosine::sample(RNG &rng) const {

	float phi = rng.unit() * 2.0f * PI_F;
	float cos_t = std::sqrt(rng.unit());

	float sin_t = std::sqrt(1 - cos_t * cos_t);
	float x = std::cos(phi) * sin_t;
	float z = std::sin(phi) * sin_t;
	float y = cos_t;

	return Vec3(x, y, z);
}

float Hemisphere::Cosine::pdf(Vec3 dir) const {
	if (dir.y < 0.0f) return 0.0f;
	return dir.y / PI_F;
}

Vec3 Sphere::Uniform::sample(RNG &rng) const {
	//A3T7 - sphere sampler

    // Generate a uniformly random point on the unit sphere.
    // Tip: start with Hemisphere::Uniform
	Vec3 dir = hemi.sample(rng);
	if (rng.coin_flip(0.5f)) {
		dir.y = -dir.y;
	}
	return dir; 
}

float Sphere::Uniform::pdf(Vec3 dir) const {
	return 1.0f / (4.0f * PI_F);
}

Sphere::Image::Image(const HDR_Image& image) {
    //A3T7 - image sampler init

    // Set up importance sampling data structures for a spherical environment map image.
    // You may make use of the _pdf, _cdf, and total members, or create your own.

    const auto [_w, _h] = image.dimension();
    w = _w;
    h = _h;

    jitter = Rect(Vec2(1.0f, 1.0f));

    _pdf.assign(w * h, 0.0f);
    _cdf.assign(w * h, 0.0f);

    float total = 0.0f;

    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) { 
            float v = (float(y) + 0.5f) / float(h);

            // image y goes up, theta goes down:
            float theta = PI_F * (1.0f - v);

            float weight = image.at(x, y).luma() * std::sin(theta);

            uint32_t idx = y * w + x;
            _pdf[idx] = weight;
            total += weight;
        }
    }

    if (total > 0.0f) {
        float accum = 0.0f;
        for (uint32_t i = 0; i < w * h; ++i) {
            _pdf[i] /= total;
            accum += _pdf[i];
            _cdf[i] = accum;
        }
        _cdf.back() = 1.0f;
    }
}

Vec3 Sphere::Image::sample(RNG &rng) const {
	if(!IMPORTANCE_SAMPLING) {
		// Step 1: Uniform sampling
		// Declare a uniform sampler and return its sample
		Sphere::Uniform sampler;
    	return sampler.sample(rng);
	} else {
		// Step 2: Importance sampling
		// Use your importance sampling data structure to generate a sample direction.
		// Tip: std::upper_bound
        if (_cdf.empty()) return Vec3(0.0f, 1.0f, 0.0f);

        float xi = rng.unit();
        auto it = std::upper_bound(_cdf.begin(), _cdf.end(), xi);
        uint32_t idx = uint32_t(std::distance(_cdf.begin(), it));
        if (idx >= _cdf.size()) idx = uint32_t(_cdf.size() - 1);

        uint32_t px = idx % w;
        uint32_t py = idx / w;

        Vec2 j = jitter.sample(rng);

        float u = (float(px) + j.x) / float(w);
        float v = (float(py) + j.y) / float(h);

        float phi = 2.0f * PI_F * u;
        float theta = PI_F * (1.0f - v);

        float sin_theta = std::sin(theta);
        float x = sin_theta * std::cos(phi);
        float y = std::cos(theta);
        float z = sin_theta * std::sin(phi);

    	return Vec3(x, y, z);
	}
}

float Sphere::Image::pdf(Vec3 dir) const {
    if(!IMPORTANCE_SAMPLING) {
		// Step 1: Uniform sampling
		// Declare a uniform sampler and return its pdf
		Sphere::Uniform sampler;
    	return sampler.pdf(dir);
	} else {
		// A3T7 - image sampler importance sampling pdf
		// What is the PDF of this distribution at a particular direction?
        if (_pdf.empty()) return 0.0f;

        Vec2 uv = Shapes::Sphere::uv(dir);

        uint32_t px = std::min(uint32_t(uv.x * float(w)), w - 1);
        uint32_t py = std::min(uint32_t(uv.y * float(h)), h - 1);

        uint32_t idx = py * w + px;

        float theta = PI_F * (1.0f - uv.y);
        float sin_theta = std::sin(theta);

        if (sin_theta <= 0.0f) return 0.0f;

        float jacobian = (float(w) * float(h)) / (2.0f * PI_F * PI_F * sin_theta);
    	return _pdf[idx] * jacobian;
	}
}

} // namespace Samplers
