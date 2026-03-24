
#include "bvh.h"
#include "aggregate.h"
#include "instance.h"
#include "tri_mesh.h"

#include <stack>

namespace PT {

struct BVHBuildData {
	BVHBuildData(size_t start, size_t range, size_t dst) : start(start), range(range), node(dst) {
	}
	size_t start; ///< start index into the primitive array
	size_t range; ///< range of index into the primitive array
	size_t node;  ///< address to update
};

struct SAHBucketData {
	BBox bb;          ///< bbox of all primitives
	size_t num_prims; ///< number of primitives in the bucket
};

template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {
	//A3T3 - build a bvh

	// Keep these
    nodes.clear();
    primitives = std::move(prims);

    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration.

	//TODO
	if (primitives.empty()) {
		root_idx = 0;
		return;
	}
	struct Bucket {
		BBox bbox;
		size_t count = 0;
	};
	std::function<size_t(size_t, size_t)> build_res = [&](size_t start, size_t size) -> size_t {
		//compute bbox
		BBox node_box;
		BBox centroid_box;
		for (size_t i = start; i < start + size; ++i) {
			BBox pb = primitives[i].bbox();
			node_box.enclose(pb);
			centroid_box.enclose(pb.center());
		}
		//create node
		size_t node_idx = new_node(node_box, start, size, 0, 0); 

		//if leaf
		if (size <= max_leaf_size) {
			nodes[node_idx].l = nodes[node_idx].r = 0;
			return node_idx;
		}
		//SAH
		static constexpr size_t n_buckets = 8;

		float best_cost = std::numeric_limits<float>::infinity();
		int best_axis = -1;
		size_t best_split = 0; 
		for (int axis = 0; axis < 3; ++axis) {

			float cmin = centroid_box.min[axis];
			float cmax = centroid_box.max[axis];
			if (cmin == cmax) continue;

			std::array<Bucket, n_buckets> buckets;

			//assign primitives to buckets:
			for (size_t i = start; i < start + size; ++i) {
				BBox pb = primitives[i].bbox();
				float c = pb.center()[axis];
				float t = (c - cmin) / (cmax - cmin);
				size_t b = std::min(n_buckets - 1, size_t(t * float(n_buckets)));
				buckets[b].bbox.enclose(pb);
				buckets[b].count++;
			}

			std::array<BBox, n_buckets> left_box, right_box;
			std::array<size_t, n_buckets> left_count{}, right_count{};

			BBox running_left;
			size_t running_left_count = 0;
			for (size_t i = 0; i < n_buckets; ++i) {
				running_left.enclose(buckets[i].bbox);
				running_left_count += buckets[i].count;
				left_box[i] = running_left;
				left_count[i] = running_left_count;
			}

			BBox running_right;
			size_t running_right_count = 0;
			for (int i = int(n_buckets) - 1; i >= 0; --i) {
				running_right.enclose(buckets[i].bbox);
				running_right_count += buckets[i].count;
				right_box[i] = running_right;
				right_count[i] = running_right_count;
			}

			//cehck all |B|-1 partitions
			for (size_t split = 1; split < n_buckets; ++split) {
				size_t n_left = left_count[split - 1];
				size_t n_right = right_count[split];
 
				if (n_left == 0 || n_right == 0) continue;

				float parent_area = node_box.surface_area();
				constexpr float c_trav = 1.0f;
				constexpr float c_isect = 1.0f;
				float cost = c_trav +
						(left_box[split - 1].surface_area() / parent_area) * float(n_left) * c_isect +
						(right_box[split].surface_area() / parent_area) * float(n_right) * c_isect;

				if (cost < best_cost) {
					best_cost = cost;
					best_axis = axis;
					best_split = split;
				}
			}
		}
		//If invalid SAH
		if (best_axis == -1) {
			nodes[node_idx].l = nodes[node_idx].r = 0;
			return node_idx;
		}
		//If valid
		float cmin = centroid_box.min[best_axis];
		float cmax = centroid_box.max[best_axis];

		auto begin = primitives.begin() + start;
		auto end = begin + size;

		auto mid = std::partition(begin, end, [&](const Primitive& p) {
			float c = p.bbox().center()[best_axis];
			float t = (c - cmin) / (cmax - cmin);
			size_t b = std::min(n_buckets - 1, size_t(t * float(n_buckets)));
			return b < best_split;
		});

		size_t left_size = size_t(mid - begin);
		size_t right_size = size - left_size;

		//validation cehck
		if (left_size == 0 || right_size == 0) {
			nodes[node_idx].l = nodes[node_idx].r = 0;
			return node_idx;
		}

		//recursive case
		size_t l = build_res(start, left_size);
		size_t r = build_res(start + left_size, right_size);

		nodes[node_idx].l = l;
		nodes[node_idx].r = r;
		return node_idx;
	};

	root_idx = build_res(0, primitives.size());
}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {
	//A3T3 - traverse your BVH

    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

	//TODO: replace this code with a more efficient traversal:
	if (nodes.empty()) return {};

	Trace ret;

	std::function<void(size_t)> find_closest_hit = [&](size_t node_idx) {
		const Node& node = nodes[node_idx];

		if (node.is_leaf()) {
			for (size_t i = node.start; i < node.start + node.size; ++i) {
				ret = Trace::min(ret, primitives[i].hit(ray));
			}
		} else {
			const Node& child1 = nodes[node.l];
			const Node& child2 = nodes[node.r];

			Vec2 hit1 = ray.dist_bounds;
			Vec2 hit2 = ray.dist_bounds;

			bool child1_hit = child1.bbox.hit(ray, hit1);
			bool child2_hit = child2.bbox.hit(ray, hit2);

			if (!child1_hit && !child2_hit) return;

			if (child1_hit && !child2_hit) {
				find_closest_hit(node.l);
				return;
			}
			if (!child1_hit && child2_hit) {
				find_closest_hit(node.r);
				return;
			}

			size_t first = (hit1.x <= hit2.x) ? node.l : node.r;
			size_t second = (hit1.x <= hit2.x) ? node.r : node.l;

			Vec2 hitsecond = (hit1.x <= hit2.x) ? hit2 : hit1;

			find_closest_hit(first);

			if (!ret.hit || hitsecond.x < ret.distance) {
				find_closest_hit(second);
			}
		}
	};

	find_closest_hit(root_idx);
	return ret;
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
	build(std::move(prims), max_leaf_size);
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
	nodes.clear();
	return std::move(primitives);
}

template<typename Primitive>
template<typename P>
typename std::enable_if<std::is_copy_assignable_v<P>, BVH<P>>::type BVH<Primitive>::copy() const {
	BVH<Primitive> ret;
	ret.nodes = nodes;
	ret.primitives = primitives;
	ret.root_idx = root_idx;
	return ret;
}

template<typename Primitive> Vec3 BVH<Primitive>::sample(RNG &rng, Vec3 from) const {
	if (primitives.empty()) return {};
	int32_t n = rng.integer(0, static_cast<int32_t>(primitives.size()));
	return primitives[n].sample(rng, from);
}

template<typename Primitive>
float BVH<Primitive>::pdf(Ray ray, const Mat4& T, const Mat4& iT) const {
	if (primitives.empty()) return 0.0f;
	float ret = 0.0f;
	for (auto& prim : primitives) ret += prim.pdf(ray, T, iT);
	return ret / primitives.size();
}

template<typename Primitive> void BVH<Primitive>::clear() {
	nodes.clear();
	primitives.clear();
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {
	// A node is a leaf if l == r, since all interior nodes must have distinct children
	return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
	Node n;
	n.bbox = box;
	n.start = start;
	n.size = size;
	n.l = l;
	n.r = r;
	nodes.push_back(n);
	return nodes.size() - 1;
}
 
template<typename Primitive> BBox BVH<Primitive>::bbox() const {
	if(nodes.empty()) return BBox{Vec3{0.0f}, Vec3{0.0f}};
	return nodes[root_idx].bbox;
}

template<typename Primitive> size_t BVH<Primitive>::n_primitives() const {
	return primitives.size();
}

template<typename Primitive>
uint32_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, uint32_t level,
                                   const Mat4& trans) const {

	std::stack<std::pair<size_t, uint32_t>> tstack;
	tstack.push({root_idx, 0u});
	uint32_t max_level = 0u;

	if (nodes.empty()) return max_level;

	while (!tstack.empty()) {

		auto [idx, lvl] = tstack.top();
		max_level = std::max(max_level, lvl);
		const Node& node = nodes[idx];
		tstack.pop();

		Spectrum color = lvl == level ? Spectrum(1.0f, 0.0f, 0.0f) : Spectrum(1.0f);
		GL::Lines& add = lvl == level ? active : lines;

		BBox box = node.bbox;
		box.transform(trans);
		Vec3 min = box.min, max = box.max;

		auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

		edge(min, Vec3{max.x, min.y, min.z});
		edge(min, Vec3{min.x, max.y, min.z});
		edge(min, Vec3{min.x, min.y, max.z});
		edge(max, Vec3{min.x, max.y, max.z});
		edge(max, Vec3{max.x, min.y, max.z});
		edge(max, Vec3{max.x, max.y, min.z});
		edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
		edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
		edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
		edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
		edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
		edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

		if (!node.is_leaf()) {
			tstack.push({node.l, lvl + 1});
			tstack.push({node.r, lvl + 1});
		} else {
			for (size_t i = node.start; i < node.start + node.size; i++) {
				uint32_t c = primitives[i].visualize(lines, active, level - lvl, trans);
				max_level = std::max(c + lvl, max_level);
			}
		}
	}
	return max_level;
}

template class BVH<Triangle>;
template class BVH<Instance>;
template class BVH<Aggregate>;
template BVH<Triangle> BVH<Triangle>::copy<Triangle>() const;

} // namespace PT
