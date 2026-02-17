
#include "texture.h"

#include <iostream>

namespace Textures {


Spectrum sample_nearest(HDR_Image const &image, Vec2 uv) {
	//clamp texture coordinates, convert to [0,w]x[0,h] pixel space:
	float x = image.w * std::clamp(uv.x, 0.0f, 1.0f);
	float y = image.h * std::clamp(uv.y, 0.0f, 1.0f);

	//the pixel with the nearest center is the pixel that contains (x,y):
	int32_t ix = int32_t(std::floor(x));
	int32_t iy = int32_t(std::floor(y));

	//texture coordinates of (1,1) map to (w,h), and need to be reduced:
	ix = std::min(ix, int32_t(image.w) - 1);
	iy = std::min(iy, int32_t(image.h) - 1);

	return image.at(ix, iy);
}

Spectrum sample_bilinear(HDR_Image const &image, Vec2 uv) {
	// A1T6: sample_bilinear
	//TODO: implement bilinear sampling strategy on texture 'image'
	float x = uv.x * float(image.w);
	float y = uv.y * float(image.h);
	int32_t xp = int32_t(std::floor(x-0.5f));
	int32_t yp = int32_t(std::floor(y-0.5f));
	int32_t x1 = xp+1;
	int32_t y1 = yp+1;
	float cx = (x-0.5f)-float(xp);
	float cy = (y-0.5f)-float(yp);

	xp = std::clamp(xp, 0, int32_t(image.w)-1);
	x1 = std::clamp(x1, 0, int32_t(image.w)-1);
	yp = std::clamp(yp, 0, int32_t(image.h)-1);
	y1 = std::clamp(y1, 0, int32_t(image.h)-1);

	Spectrum t00 = image.at(xp, yp); //t(x',y')
	Spectrum t10 = image.at(x1, yp); //t(x'+1,y')
	Spectrum t01 = image.at(xp, y1); //t(x',y'+1)
	Spectrum t11 = image.at(x1, y1); //t(x'+1,y'+1)
	Spectrum tx0 = (1.0f-cx)*t00+cx*t10; 
	Spectrum tx1 = (1.0f-cx)*t01+cx*t11; 
	return (1.0f-cy)*tx0+cy*tx1; 
}


Spectrum sample_trilinear(HDR_Image const &base, std::vector< HDR_Image > const &levels, Vec2 uv, float lod) {
	// A1T6: sample_trilinear
	//TODO: implement trilinear sampling strategy on using mip-map 'levels' 

	float lodp = std::clamp(lod, 0.0f, float(levels.size()));
	int dp = int(std::floor(lodp));
	int dp1 = std::min(dp + 1, int(levels.size()));
	float cd = lodp - float(dp);

	auto lv_img = [&](int32_t L) -> HDR_Image const& {
		if (L <= 0) return base;
		return levels[uint32_t(L - 1)];
	};
	HDR_Image const &img0 = lv_img(dp);
	HDR_Image const &img1 = lv_img(dp1);
	Spectrum td = sample_bilinear(img0, uv);
	Spectrum td1 = sample_bilinear(img1, uv);
	return (1.0f-cd)*td+cd*td1;
}

/*
 * generate_mipmap- generate mipmap levels from a base image.
 *  base: the base image
 *  levels: pointer to vector of levels to fill (must not be null)
 *
 * generates a stack of levels [1,n] of sizes w_i, h_i, where:
 *   w_i = max(1, floor(w_{i-1})/2)
 *   h_i = max(1, floor(h_{i-1})/2)
 *  with:
 *   w_0 = base.w
 *   h_0 = base.h
 *  and n is the smalles n such that w_n = h_n = 1
 *
 * each level should be calculated by downsampling a blurred version
 * of the previous level to remove high-frequency detail.
 *
 */
void generate_mipmap(HDR_Image const &base, std::vector< HDR_Image > *levels_) {
	assert(levels_);
	auto &levels = *levels_;


	{ // allocate sublevels sufficient to scale base image all the way to 1x1:
		int32_t num_levels = static_cast<int32_t>(std::log2(std::max(base.w, base.h)));
		assert(num_levels >= 0);

		levels.clear();
		levels.reserve(num_levels);

		uint32_t width = base.w;
		uint32_t height = base.h;
		for (int32_t i = 0; i < num_levels; ++i) {
			assert(!(width == 1 && height == 1)); //would have stopped before this if num_levels was computed correctly

			width = std::max(1u, width / 2u);
			height = std::max(1u, height / 2u);

			levels.emplace_back(width, height);
		}
		assert(width == 1 && height == 1);
		assert(levels.size() == uint32_t(num_levels));
	}

	//now fill in the levels using a helper:
	//downsample:
	// fill in dst to represent the low-frequency component of src
	auto downsample = [](HDR_Image const &src, HDR_Image &dst) {
		//dst is half the size of src in each dimension:
		assert(std::max(1u, src.w / 2u) == dst.w);
		assert(std::max(1u, src.h / 2u) == dst.h);

		// A1T6: generate
		//TODO: Write code to fill the levels of the mipmap hierarchy by downsampling
		for (uint32_t y = 0; y < dst.h; ++y) {
        	for (uint32_t x = 0; x < dst.w; ++x) {
				Vec2 v = Vec2((float(x)+0.5f)/float(dst.w), (float(y)+0.5f)/float(dst.h));
				 dst.at(x, y) = sample_bilinear(src, v);
			}
		}
		//Be aware that the alignment of the samples in dst and src will be different depending on whether the image is even or odd.

	};

	std::cout << "Regenerating mipmap (" << levels.size() << " levels): [" << base.w << "x" << base.h << "]";
	std::cout.flush();
	for (uint32_t i = 0; i < levels.size(); ++i) {
		HDR_Image const &src = (i == 0 ? base : levels[i-1]);
		HDR_Image &dst = levels[i];
		std::cout << " -> [" << dst.w << "x" << dst.h << "]"; std::cout.flush();

		downsample(src, dst);
	}
	std::cout << std::endl;
	
}

Image::Image(Sampler sampler_, HDR_Image const &image_) {
	sampler = sampler_;
	image = image_.copy();
	update_mipmap();
}

Spectrum Image::evaluate(Vec2 uv, float lod) const {
	if (image.w == 0 && image.h == 0) return Spectrum();
	if (sampler == Sampler::nearest) {
		return sample_nearest(image, uv);
	} else if (sampler == Sampler::bilinear) {
		return sample_bilinear(image, uv);
	} else {
		return sample_trilinear(image, levels, uv, lod);
	}
}

void Image::update_mipmap() {
	if (sampler == Sampler::trilinear) {
		generate_mipmap(image, &levels);
	} else {
		levels.clear();
	}
}

GL::Tex2D Image::to_gl() const {
	return image.to_gl(1.0f);
}

void Image::make_valid() {
	update_mipmap();
}

Spectrum Constant::evaluate(Vec2 uv, float lod) const {
	return color * scale;
}

} // namespace Textures
bool operator!=(const Textures::Constant& a, const Textures::Constant& b) {
	return a.color != b.color || a.scale != b.scale;
}

bool operator!=(const Textures::Image& a, const Textures::Image& b) {
	return a.image != b.image;
}

bool operator!=(const Texture& a, const Texture& b) {
	if (a.texture.index() != b.texture.index()) return false;
	return std::visit(
		[&](const auto& data) { return data != std::get<std::decay_t<decltype(data)>>(b.texture); },
		a.texture);
}
