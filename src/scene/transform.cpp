
#include "transform.h"

Mat4 Transform::local_to_parent() const {
	return Mat4::translate(translation) * rotation.to_mat() * Mat4::scale(scale);
}

Mat4 Transform::parent_to_local() const {
	return Mat4::scale(1.0f / scale) * rotation.inverse().to_mat() * Mat4::translate(-translation);
}

Mat4 Transform::local_to_world() const {
	// A1T1: local_to_world
	//don't use Mat4::inverse() in your code.
	Mat4 localToParen = local_to_parent();
	if (std::shared_ptr< Transform > parent_ = parent.lock()) {
		//case where transform has a parent
		//...
		return parent_->local_to_world()*localToParen;
	} else {
		//case where transform doesn't have a parent
		return localToParen;
	}
	return Mat4::I; //<-- wrong, but here so code will compile
}

Mat4 Transform::world_to_local() const {
	// A1T1: world_to_local
	//don't use Mat4::inverse() in your code.
	Mat4 ParenToLocal = parent_to_local();
	if (std::shared_ptr< Transform > parent_ = parent.lock()) {
		//case where transform has a parent
		//...
		return ParenToLocal*parent_->world_to_local();
	} else {
		//case where transform doesn't have a parent
		return ParenToLocal;
	}
	return Mat4::I; //<-- wrong, but here so code will compile
}

bool operator!=(const Transform& a, const Transform& b) {
	return a.parent.lock() != b.parent.lock() || a.translation != b.translation ||
	       a.rotation != b.rotation || a.scale != b.scale;
}
