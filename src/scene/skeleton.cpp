#include <unordered_set>
#include "skeleton.h"
#include "test.h"
#include <iostream>

void Skeleton::Bone::compute_rotation_axes(Vec3 *x_, Vec3 *y_, Vec3 *z_) const {
	assert(x_ && y_ && z_);
	auto &x = *x_;
	auto &y = *y_;
	auto &z = *z_;

	//y axis points in the direction of extent:
	y = extent.unit();
	//if extent is too short to normalize nicely, point along the skeleton's 'y' axis:
	if (!y.valid()) {
		y = Vec3{0.0f, 1.0f, 0.0f};
	}

	//x gets skeleton's 'x' axis projected to be orthogonal to 'y':
	x = Vec3{1.0f, 0.0f, 0.0f};
	x = (x - dot(x,y) * y).unit();
	if (!x.valid()) {
		//if y perfectly aligns with skeleton's 'x' axis, x, gets skeleton's z axis:
		x = Vec3{0.0f, 0.0f, 1.0f};
		x = (x - dot(x,y) * y).unit(); //(this should do nothing)
	}

	//z computed from x,y:
	z = cross(x,y);

	//x,z rotated by roll:
	float cr = std::cos(roll / 180.0f * PI_F);
	float sr = std::sin(roll / 180.0f * PI_F);
	// x = cr * x + sr * -z;
	// z = cross(x,y);
	std::tie(x, z) = std::make_pair(cr * x + sr * -z, cr * z + sr * x);
}

std::vector< Mat4 > Skeleton::bind_pose() const {
	//A4T2a: bone-to-skeleton transformations in the bind pose
	//(the bind pose does not rotate by Bone::pose)

	std::vector< Mat4 > bind;
	bind.reserve(bones.size());

	//NOTE: bones is guaranteed to be ordered such that parents appear before child bones.

	for (auto const &bone : bones) {
		
		Mat4 local_to_parent;
		if (bone.parent == -1U) {
			local_to_parent = Mat4::translate(base);
		} else {
			local_to_parent = Mat4::translate(bones[bone.parent].extent);
		}

		if (bone.parent == -1U) {
			bind.emplace_back(local_to_parent);
		} else {
			bind.emplace_back(bind[bone.parent] * local_to_parent);
		}
	}

	assert(bind.size() == bones.size()); //should have a transform for every bone.
	return bind;
}

std::vector< Mat4 > Skeleton::current_pose() const {
    //A4T2a: bone-to-skeleton transformations in the current pose

	//Similar to bind_pose(), but takes rotation from Bone::pose into account.
	// (and translation from Skeleton::base_offset!)

	//You'll probably want to write a loop similar to bind_pose().

	//Useful functions:
	//Bone::compute_rotation_axes() will tell you what axes (in local bone space) Bone::pose should rotate around.
	//Mat4::angle_axis(angle, axis) will produce a matrix that rotates angle (in degrees) around a given axis.
	std::vector< Mat4 > current;
	current.reserve(bones.size());
	for (auto const &bone : bones) {
		Vec3 x, y, z;
		bone.compute_rotation_axes(&x, &y, &z);

		Mat4 rx = Mat4::angle_axis(bone.pose.x, x);
		Mat4 ry = Mat4::angle_axis(bone.pose.y, y);
		Mat4 rz = Mat4::angle_axis(bone.pose.z, z);

		Mat4 rotation = rz * ry * rx;
		Mat4 local_to_parent;

		if (bone.parent == -1U) {
			local_to_parent = Mat4::translate(base + base_offset) * rotation;
		} else {
			local_to_parent = Mat4::translate(bones[bone.parent].extent) * rotation;
		}

		if (bone.parent == -1U) {
			current.emplace_back(local_to_parent);
		} else {
			current.emplace_back(current[bone.parent] * local_to_parent);
		}
	}

	assert(current.size() == bones.size());
	return current;
}

std::vector< Vec3 > Skeleton::gradient_in_current_pose() const {
    //A4T2b: IK gradient

    // Computes the gradient (partial derivative) of IK energy relative to each bone's Bone::pose, in the current pose.

	//The IK energy is the sum over all *enabled* handles of the squared distance from the tip of Handle::bone to Handle::target
	std::vector< Vec3 > gradient(bones.size(), Vec3{0.0f, 0.0f, 0.0f});
	std::vector< Mat4 > pose = current_pose();
	//TODO: loop over handles and over bones in the chain leading to the handle, accumulating gradient contributions.
	//remember bone.compute_rotation_axes() -- should be useful here, too!

	for (const auto& handle : handles) {
		if (!handle.enabled) continue;
		if (handle.bone >= bones.size()) continue;

		BoneIndex end = handle.bone;
		//current tip position 
		Vec3 tip = pose[end] * bones[end].extent;
		Vec3 error = tip - handle.target;
 
		for (BoneIndex b = end; b != -1U; b = bones[b].parent) {
			const Bone& bone = bones[b];

			//local rotation axes 
			Vec3 x, y, z;
			bone.compute_rotation_axes(&x, &y, &z);

			//In skeleton-local space
			Vec3 center = pose[b] * Vec3{0.0f, 0.0f, 0.0f};
 
			Mat4 rx = Mat4::angle_axis(bone.pose.x, x);
			Mat4 ry = Mat4::angle_axis(bone.pose.y, y);

 			Vec3 axis_x = pose[b].rotate(x); 
			Vec3 axis_y = (pose[b] * Mat4::angle_axis(-bone.pose.x, x)).rotate(y);
			Vec3 axis_z = (pose[b] * Mat4::angle_axis(-bone.pose.x, x)
            				* Mat4::angle_axis(-bone.pose.y, y)).rotate(z);

			Vec3 to_tip = tip - center;

			Vec3 dpx = cross(axis_x, to_tip);
			Vec3 dpy = cross(axis_y, to_tip);
			Vec3 dpz = cross(axis_z, to_tip);
			gradient[b].x += dot(error, dpx);
			gradient[b].y += dot(error, dpy);
			gradient[b].z += dot(error, dpz);
		}
	}

	assert(gradient.size() == bones.size());
	return gradient;
}

bool Skeleton::solve_ik(uint32_t steps) {
	//A4T2b - gradient descent
	//check which handles are enabled
	//run `steps` iterations
	bool any_enabled = false;
	for (const auto& handle : handles) {
		if (handle.enabled) {
			any_enabled = true;
			break;
		}
	}
	if (!any_enabled) return true;
	//call gradient_in_current_pose() to compute d loss / d pose
	//add ...
	//if at a local minimum (e.g., gradient is near-zero), return 'true'.
	//if run through all steps, return `false`.
	//step size and onvergence threshold
	const float tau = 1.0f;
	const float eps = 1e-4f;
	for (uint32_t ii = 0; ii < steps; ++ii) {
		std::vector< Vec3 > gradient = gradient_in_current_pose();

		float max_len = 0.0f;
		for (const auto& g : gradient) {
			max_len = std::max(max_len, g.norm());
		}

		//Found a local minimum
		if (max_len < eps) {
			return true;
		}

		for (uint32_t b = 0; b < bones.size(); ++b) {
			bones[b].pose -= tau * gradient[b];
		}
	}

	return false;
}

Vec3 Skeleton::closest_point_on_line_segment(Vec3 const &a, Vec3 const &b, Vec3 const &p) {
	//A4T3: bone weight computation (closest point helper)

    // Return the closest point to 'p' on the line segment from a to b

	//Efficiency note: you can do this without any sqrt's! (no .unit() or .norm() is needed!)
	Vec3 ab = b - a;
	float ab2 = dot(ab, ab);

	if (ab2 == 0.0f) return a;

	float t = dot(p - a, ab) / ab2;
	t = std::clamp(t, 0.0f, 1.0f);

	return a + t * ab;
}

void Skeleton::assign_bone_weights(Halfedge_Mesh *mesh_) const {
	assert(mesh_);
	auto &mesh = *mesh_;
	(void)mesh; //avoid complaints about unused mesh

	//A4T3: bone weight computation

	//visit every vertex and **set new values** in Vertex::bone_weights (don't append to old values)

	//be sure to use bone positions in the bind pose (not the current pose!)

	//you should fill in the helper closest_point_on_line_segment() before working on this function
	std::vector<Mat4> bind = bind_pose();

	for (auto v = mesh.vertices.begin(); v != mesh.vertices.end(); ++v) {
		v->bone_weights.clear();
		Vec3 const p = v->position;

		std::vector<Halfedge_Mesh::Vertex::Bone_Weight> unnormalized;
		unnormalized.reserve(bones.size());

		float sum = 0.0f;
		for (BoneIndex bi = 0; bi < bones.size(); ++bi) {
			Bone const &bone = bones[bi];

			if (bone.radius <= 0.0f) continue;

			Vec3 a = bind[bi] * Vec3{0.0f, 0.0f, 0.0f};
			Vec3 b = bind[bi] * bone.extent;

			Vec3 q = closest_point_on_line_segment(a, b, p);
			float d = (p - q).norm();

			float w_hat = std::max(0.0f, (bone.radius - d) / bone.radius);
			if (w_hat > 0.0f) {
				unnormalized.emplace_back(Halfedge_Mesh::Vertex::Bone_Weight{bi, w_hat});
				sum += w_hat;
			}
		}

		if (sum == 0.0f) continue;
		//normalize the weights
		for (auto &bw : unnormalized) {
			bw.weight /= sum;
			v->bone_weights.emplace_back(bw);
		}
	}

}

Indexed_Mesh Skeleton::skin(Halfedge_Mesh const &mesh, std::vector< Mat4 > const &bind, std::vector< Mat4 > const &current) {
	assert(bind.size() == current.size());


	//A4T3: linear blend skinning

	//one approach you might take is to first compute the skinned positions (at every vertex) and normals (at every corner)
	// then generate faces in the style of Indexed_Mesh::from_halfedge_mesh

	//---- step 1: figure out skinned positions ---

	std::unordered_map< Halfedge_Mesh::VertexCRef, Vec3 > skinned_positions;
	std::unordered_map< Halfedge_Mesh::HalfedgeCRef, Vec3 > skinned_normals;
	//reserve hash table space to (one hopes) avoid re-hashing:
	skinned_positions.reserve(mesh.vertices.size());
	skinned_normals.reserve(mesh.halfedges.size());

	//(you will probably want to precompute some bind-to-current transformation matrices here)
	std::vector< Mat4 > bone_trans;
	bone_trans.reserve(bind.size());
	for (uint32_t i = 0; i < bind.size(); ++i) {
		bone_trans.emplace_back(current[i] * bind[i].inverse());
	}

	//zero matrix helper:
	auto zero_mat4 = []() -> Mat4 {
		Mat4 m;
		for (uint32_t c = 0; c < 4; ++c) {
			for (uint32_t r = 0; r < 4; ++r) {
				m[c][r] = 0.0f;
			}
		}
		return m;
	};
	
	for (auto vi = mesh.vertices.begin(); vi != mesh.vertices.end(); ++vi) {
 		if (vi->bone_weights.empty()) {
			skinned_positions.emplace(vi, vi->position);

			auto h = vi->halfedge;
			do {
				skinned_normals.emplace(h, h->corner_normal);
				h = h->twin->next;
			} while (h != vi->halfedge);

			continue;
		}
		//skinned_positions.emplace(vi, vi->position); //PLACEHOLDER! Replace with code that computes the position of the vertex according to vi->position and vi->bone_weights.
		//NOTE: vertices with empty bone_weights should remain in place.
		//blend all bone transforms influencing current vertex
		Mat4 blended = zero_mat4();

		for (auto const &bw : vi->bone_weights) {
			assert(bw.bone < bone_trans.size());
			blended = blended + bw.weight * bone_trans[bw.bone];
		}

		//transform position
		Vec3 p = blended * vi->position;
		skinned_positions.emplace(vi, p);

		//normals transform by inverse-transpose
		Mat4 normalT = blended.inverse().T();

		//circulate corners at this vertex:
		auto h = vi->halfedge;
		do {
			//NOTE: could skip if h->face->boundary, since such corners don't get emitted
			Vec3 n = normalT.rotate(h->corner_normal).unit();

			if (!n.valid()) n = h->corner_normal.unit();
			if (!n.valid()) n = h->corner_normal;

			skinned_normals.emplace(h, n);

			h = h->twin->next; 
		} while (h != vi->halfedge);
	}

	//---- step 2: transform into an indexed mesh ---

	//Hint: you should be able to use the code from Indexed_Mesh::from_halfedge_mesh (SplitEdges version) pretty much verbatim, you'll just need to fill in the positions and normals.

	Indexed_Mesh result;
	auto &verts = result.vertices();
	auto &idxs = result.indices();

	verts.clear();
	idxs.clear();

	for (auto f = mesh.faces.begin(); f != mesh.faces.end(); ++f) {
		if (f->boundary) continue;

		uint32_t base = uint32_t(verts.size());

		auto h = f->halfedge;
		do {
			Indexed_Mesh::Vert v;
			v.pos = skinned_positions.at(h->vertex);
			v.norm = skinned_normals.at(h);
			v.uv = h->corner_uv;
			v.id = h->vertex->id; 

			verts.emplace_back(v);

			h = h->next;
		} while (h != f->halfedge);
		
		uint32_t deg = f->degree();
		for (uint32_t i = 1; i + 1 < deg; ++i) {
			idxs.emplace_back(base + 0);
			idxs.emplace_back(base + i);
			idxs.emplace_back(base + i + 1);
		}
	}

	return result; 
}

void Skeleton::for_bones(const std::function<void(Bone&)>& f) {
	for (auto& bone : bones) {
		f(bone);
	}
}


void Skeleton::erase_bone(BoneIndex bone) {
	assert(bone < bones.size());
	//update indices in bones:
	for (uint32_t b = 0; b < bones.size(); ++b) {
		if (bones[b].parent == -1U) continue;
		if (bones[b].parent == bone) {
			assert(b > bone); //topological sort!
			//keep bone tips in the same place when deleting parent bone:
			bones[b].extent += bones[bone].extent;
			bones[b].parent = bones[bone].parent;
		} else if (bones[b].parent > bone) {
			assert(b > bones[b].parent); //topological sort!
			bones[b].parent -= 1;
		}
	}
	// erase the bone
	bones.erase(bones.begin() + bone);
	//update indices in handles (and erase any handles on this bone):
	for (uint32_t h = 0; h < handles.size(); /* later */) {
		if (handles[h].bone == bone) {
			erase_handle(h);
		} else if (handles[h].bone > bone) {
			handles[h].bone -= 1;
			++h;
		} else {
			++h;
		}
	}
}

void Skeleton::erase_handle(HandleIndex handle) {
	assert(handle < handles.size());

	//nothing internally refers to handles by index so can just delete:
	handles.erase(handles.begin() + handle);
}


Skeleton::BoneIndex Skeleton::add_bone(BoneIndex parent, Vec3 extent) {
	assert(parent == -1U || parent < bones.size());
	Bone bone;
	bone.extent = extent;
	bone.parent = parent;
	//all other parameters left as default.

	//slightly unfortunate hack:
	//(to ensure increasing IDs within an editing session, but reset on load)
	std::unordered_set< uint32_t > used;
	for (auto const &b : bones) {
		used.emplace(b.channel_id);
	}
	while (used.count(next_bone_channel_id)) ++next_bone_channel_id;
	bone.channel_id = next_bone_channel_id++;

	//all other parameters left as default.

	BoneIndex index = BoneIndex(bones.size());
	bones.emplace_back(bone);

	return index;
}

Skeleton::HandleIndex Skeleton::add_handle(BoneIndex bone, Vec3 target) {
	assert(bone < bones.size());
	Handle handle;
	handle.bone = bone;
	handle.target = target;
	//all other parameters left as default.

	//slightly unfortunate hack:
	//(to ensure increasing IDs within an editing session, but reset on load)
	std::unordered_set< uint32_t > used;
	for (auto const &h : handles) {
		used.emplace(h.channel_id);
	}
	while (used.count(next_handle_channel_id)) ++next_handle_channel_id;
	handle.channel_id = next_handle_channel_id++;

	HandleIndex index = HandleIndex(handles.size());
	handles.emplace_back(handle);

	return index;
}


Skeleton Skeleton::copy() {
	//turns out that there aren't any fancy pointer data structures to fix up here.
	return *this;
}

void Skeleton::make_valid() {
	for (uint32_t b = 0; b < bones.size(); ++b) {
		if (!(bones[b].parent == -1U || bones[b].parent < b)) {
			warn("bones[%u].parent is %u, which is not < %u; setting to -1.", b, bones[b].parent, b);
			bones[b].parent = -1U;
		}
	}
	if (bones.empty() && !handles.empty()) {
		warn("Have %u handles but no bones. Deleting handles.", uint32_t(handles.size()));
		handles.clear();
	}
	for (uint32_t h = 0; h < handles.size(); ++h) {
		if (handles[h].bone >= HandleIndex(bones.size())) {
			warn("handles[%u].bone is %u, which is not < bones.size(); setting to 0.", h, handles[h].bone);
			handles[h].bone = 0;
		}
	}
}

//-------------------------------------------------

Indexed_Mesh Skinned_Mesh::bind_mesh() const {
	return Indexed_Mesh::from_halfedge_mesh(mesh, Indexed_Mesh::SplitEdges);
}

Indexed_Mesh Skinned_Mesh::posed_mesh() const {
	return Skeleton::skin(mesh, skeleton.bind_pose(), skeleton.current_pose());
}

Skinned_Mesh Skinned_Mesh::copy() {
	return Skinned_Mesh{mesh.copy(), skeleton.copy()};
}
