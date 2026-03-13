
#include "halfedge.h"

#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <iostream>

/******************************************************************
*********************** Local Operations **************************
******************************************************************/

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it cannot perform an operation (i.e., because
    the resulting mesh does not have a valid representation).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementation, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/


/*
 * add_face: add a standalone face to the mesh
 *  sides: number of sides
 *  radius: distance from vertices to origin
 *
 * We provide this method as an example of how to make new halfedge mesh geometry.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::add_face(uint32_t sides, float radius) {
	//faces with fewer than three sides are invalid, so abort the operation:
	if (sides < 3) return std::nullopt;


	std::vector< VertexRef > face_vertices;
	//In order to make the first edge point in the +x direction, first vertex should
	// be at -90.0f - 0.5f * 360.0f / float(sides) degrees, so:
	float const start_angle = (-0.25f - 0.5f / float(sides)) * 2.0f * PI_F;
	for (uint32_t s = 0; s < sides; ++s) {
		float angle = float(s) / float(sides) * 2.0f * PI_F + start_angle;
		VertexRef v = emplace_vertex();
		v->position = radius * Vec3(std::cos(angle), std::sin(angle), 0.0f);
		face_vertices.emplace_back(v);
	}

	assert(face_vertices.size() == sides);

	//assemble the rest of the mesh parts:
	FaceRef face = emplace_face(false); //the face to return
	FaceRef boundary = emplace_face(true); //the boundary loop around the face

	std::vector< HalfedgeRef > face_halfedges; //will use later to set ->next pointers

	for (uint32_t s = 0; s < sides; ++s) {
		//will create elements for edge from a->b:
		VertexRef a = face_vertices[s];
		VertexRef b = face_vertices[(s+1)%sides];

		//h is the edge on face:
		HalfedgeRef h = emplace_halfedge();
		//t is the twin, lies on boundary:
		HalfedgeRef t = emplace_halfedge();
		//e is the edge corresponding to h,t:
		EdgeRef e = emplace_edge(false); //false: non-sharp

		//set element data to something reasonable:
		//(most ops will do this with interpolate_data(), but no data to interpolate here)
		h->corner_uv = a->position.xy() / (2.0f * radius) + 0.5f;
		h->corner_normal = Vec3(0.0f, 0.0f, 1.0f);
		t->corner_uv = b->position.xy() / (2.0f * radius) + 0.5f;
		t->corner_normal = Vec3(0.0f, 0.0f,-1.0f);

		//thing -> halfedge pointers:
		e->halfedge = h;
		a->halfedge = h;
		if (s == 0) face->halfedge = h;
		if (s + 1 == sides) boundary->halfedge = t;

		//halfedge -> thing pointers (except 'next' -- will set that later)
		h->twin = t;
		h->vertex = a;
		h->edge = e;
		h->face = face;

		t->twin = h;
		t->vertex = b;
		t->edge = e;
		t->face = boundary;

		face_halfedges.emplace_back(h);
	}

	assert(face_halfedges.size() == sides);

	for (uint32_t s = 0; s < sides; ++s) {
		face_halfedges[s]->next = face_halfedges[(s+1)%sides];
		face_halfedges[(s+1)%sides]->twin->next = face_halfedges[s]->twin;
	}

	return face;
}


/*
 * bisect_edge: split an edge without splitting the adjacent faces
 *  e: edge to split
 *
 * returns: added vertex
 *
 * We provide this as an example for how to implement local operations.
 * (and as a useful subroutine!)
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::bisect_edge(EdgeRef e) {
	// Phase 0: draw a picture
	//
	// before:
	//    ----h--->
	// v1 ----e--- v2
	//   <----t---
	//
	// after:
	//    --h->    --h2->
	// v1 --e-- vm --e2-- v2
	//    <-t2-    <--t--
	//

	// Phase 1: collect existing elements
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	VertexRef v1 = h->vertex;
	VertexRef v2 = t->vertex;

	// Phase 2: Allocate new elements, set data
	VertexRef vm = emplace_vertex();
	vm->position = (v1->position + v2->position) / 2.0f;
	interpolate_data({v1, v2}, vm); //set bone_weights

	EdgeRef e2 = emplace_edge();
	e2->sharp = e->sharp; //copy sharpness flag

	HalfedgeRef h2 = emplace_halfedge();
	interpolate_data({h, h->next}, h2); //set corner_uv, corner_normal

	HalfedgeRef t2 = emplace_halfedge();
	interpolate_data({t, t->next}, t2); //set corner_uv, corner_normal

	// The following elements aren't necessary for the bisect_edge, but they are here to demonstrate phase 4
    FaceRef f_not_used = emplace_face();
    HalfedgeRef h_not_used = emplace_halfedge();

	// Phase 3: Reassign connectivity (careful about ordering so you don't overwrite values you may need later!)

	vm->halfedge = h2;

	e2->halfedge = h2;

	assert(e->halfedge == h); //unchanged

	//n.b. h remains on the same face so even if h->face->halfedge == h, no fixup needed (t, similarly)

	h2->twin = t;
	h2->next = h->next;
	h2->vertex = vm;
	h2->edge = e2;
	h2->face = h->face;

	t2->twin = h;
	t2->next = t->next;
	t2->vertex = vm;
	t2->edge = e;
	t2->face = t->face;
	
	h->twin = t2;
	h->next = h2;
	assert(h->vertex == v1); // unchanged
	assert(h->edge == e); // unchanged
	//h->face unchanged

	t->twin = h2;
	t->next = t2;
	assert(t->vertex == v2); // unchanged
	t->edge = e2;
	//t->face unchanged


	// Phase 4: Delete unused elements
    erase_face(f_not_used);
    erase_halfedge(h_not_used);

	// Phase 5: Return the correct iterator
	return vm;
}


/*
 * split_edge: split an edge and adjacent (non-boundary) faces
 *  e: edge to split
 *
 * returns: added vertex. vertex->halfedge should lie along e
 *
 * Note that when splitting the adjacent faces, the new edge
 * should connect to the vertex ccw from the ccw-most end of e
 * within the face.
 *
 * Do not split adjacent boundary faces.
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(EdgeRef e) {
	// A2L2 (REQUIRED): split_edge
	// Phase 1: collect existing elements
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;

	VertexRef v1 = h->vertex;
	VertexRef v2 = t->vertex;

	FaceRef f_h = h->face;
	FaceRef f_t = t->face;

	// Save the old connectivity
	HalfedgeRef h_next_old = h->next;
	HalfedgeRef t_next_old = t->next;

	//Helpers 
 	auto lastEdge = [](HalfedgeRef x) -> HalfedgeRef {
		HalfedgeRef cur = x; 
		while (cur->next != x) cur = cur->next;
		return cur;
	};
	//Find Edge Of Vertex in face f
	auto EdgeOfVertex = [this](FaceRef f, VertexRef v) -> HalfedgeRef {
		HalfedgeRef start = f->halfedge;
		HalfedgeRef cur = start;
		do {
			if (cur->vertex == v) return cur;
			cur = cur->next;
		} while (cur != start);
		return this->halfedges.end();
	};

	// Split a non-boundary face by a diagonal from vm t0 vc
	auto SplitFaceWithDiagonal = [&](FaceRef f, VertexRef vm, VertexRef vc) -> bool {
		HalfedgeRef h_vm = EdgeOfVertex(f, vm);
		HalfedgeRef h_vc  = EdgeOfVertex(f, vc);

		if (h_vm == halfedges.end() || h_vc == halfedges.end() ) return false;

		HalfedgeRef h_vm_prev = lastEdge(h_vm);
		HalfedgeRef h_vc_prev  = lastEdge(h_vc);

		//Allocate new data:
		FaceRef f_new = emplace_face();
		EdgeRef e_diag = emplace_edge();

		HalfedgeRef h_vc_vm = emplace_halfedge();  
		HalfedgeRef h_vm_vc = emplace_halfedge(); 
 
		interpolate_data({h_vc, h_vc->next}, h_vc_vm);
		interpolate_data({h_vm, h_vm->next}, h_vm_vc);
 
		h_vc_vm->twin = h_vm_vc;
		h_vm_vc->twin = h_vc_vm;

		e_diag->halfedge = h_vc_vm;
		h_vc_vm->edge = e_diag;
		h_vm_vc->edge = e_diag;

		h_vc_vm->vertex = vc;
		h_vm_vc->vertex = vm;

		h_vc_vm->face = f;
		h_vm_vc->face = f_new;

		//set pointers
		h_vc_prev->next = h_vc_vm;
		h_vc_vm->next   = h_vm;
 
		h_vm_prev->next = h_vm_vc;
		h_vm_vc->next    = h_vc;

		//assign faces
		{
			HalfedgeRef cur = h_vc;
			do {
				cur->face = f_new;
				cur = cur->next;
			} while (cur != h_vc);
		}

		f->halfedge = h_vm;
		f_new->halfedge = h_vc;

		return true;
	};

	//Phase 2:allocate new elements, set data

	//new midpoint vertex:
	VertexRef vm = emplace_vertex();
	vm->position = (v1->position + v2->position) / 2.0f;
	interpolate_data({v1, v2}, vm);
 
	EdgeRef e2 = emplace_edge();
	e2->sharp = e->sharp;
 
	HalfedgeRef h2 = emplace_halfedge(); // vm -> v2   (will be twin of t)
	HalfedgeRef t2 = emplace_halfedge(); // v2 -> vm   (will be twin of h)

	interpolate_data({h, h_next_old}, h2);
	interpolate_data({t, t_next_old}, t2);

	//Phase 3: Reassign connectivity 

	// WTS
	//h :v1->vm   
	//t2:vm->v1 
	//h2:vm->v2  
	//t :v2->vm 
 
 	vm->halfedge = t2;
	e2->halfedge = h2;
 
	assert(e->halfedge == h);

	h2->twin = t;
	h2->next = h_next_old;
	h2->vertex = vm;
	h2->edge = e2;
	h2->face = f_h;

	t2->twin = h;
	t2->next = t_next_old;
	t2->vertex = vm;
	t2->edge = e;
	t2->face = f_t;


	h->twin = t2;
	h->next = h2;
	assert(h->vertex == v1);
	assert(h->edge == e);

	t->twin = h2;
	t->next = t2;
	assert(t->vertex == v2);
	t->edge = e2;

	//split according to boundary
	VertexRef v_ccw_h = h_next_old->twin->vertex;
	VertexRef v_ccw_t = t_next_old->twin->vertex;

	if (!f_h->boundary) {
		(void)SplitFaceWithDiagonal(f_h, vm, v_ccw_h);
	}

	if (!f_t->boundary) { 
		(void)SplitFaceWithDiagonal(f_t, vm, v_ccw_t); 
	}
	//Phase 4:Delete unused elements
	//Phase 5:Return the correct iterator
	return vm;
}



/*
 * inset_vertex: divide a face into triangles by placing a vertex at f->center()
 *  f: the face to add the vertex to
 *
 * returns:
 *  std::nullopt if insetting a vertex would make mesh invalid
 *  the inset vertex otherwise
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::inset_vertex(FaceRef f) {
	// A2Lx4 (OPTIONAL): inset vertex
	
	(void)f;
    return std::nullopt;
}


/* [BEVEL NOTE] Note on the beveling process:

	Each of the bevel_vertex, bevel_edge, and extrude_face functions do not represent
	a full bevel/extrude operation. Instead, they should update the _connectivity_ of
	the mesh, _not_ the positions of newly created vertices. In fact, you should set
	the positions of new vertices to be exactly the same as wherever they "started from."

	When you click on a mesh element while in bevel mode, one of those three functions
	is called. But, because you may then adjust the distance/offset of the newly
	beveled face, we need another method of updating the positions of the new vertices.

	This is where bevel_positions and extrude_positions come in: these functions are
	called repeatedly as you move your mouse, the position of which determines the
	amount / shrink parameters. These functions are also passed an array of the original
	vertex positions, stored just after the bevel/extrude call, in order starting at
	face->halfedge->vertex, and the original element normal, computed just *before* the
	bevel/extrude call.

	Finally, note that the amount, extrude, and/or shrink parameters are not relative
	values -- you should compute a particular new position from them, not a delta to
	apply.
*/

/*
 * bevel_vertex: creates a face in place of a vertex
 *  v: the vertex to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(VertexRef v) {
	//A2Lx5 (OPTIONAL): Bevel Vertex
	// Reminder: This function does not update the vertex positions.
	// Remember to also fill in bevel_vertex_helper (A2Lx5h)

	(void)v;
    return std::nullopt;
}

/*
 * bevel_edge: creates a face in place of an edge
 *  e: the edge to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(EdgeRef e) {
	//A2Lx6 (OPTIONAL): Bevel Edge
	// Reminder: This function does not update the vertex positions.
	// remember to also fill in bevel_edge_helper (A2Lx6h)

	(void)e;
    return std::nullopt;
}

/*
 * extrude_face: creates a face inset into a face
 *  f: the face to inset
 *
 * returns: reference to the inner face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::extrude_face(FaceRef f) {
	//A2L4: Extrude Face
	// Reminder: This function does not update the vertex positions.
	// Remember to also fill in extrude_helper (A2L4h)

	//return early
    if (f == faces.end()) return std::nullopt;
    if (f->boundary) return std::nullopt;

    //Collect old halfedges of f
    std::vector<HalfedgeRef> h_old;
    {
        HalfedgeRef h0 = f->halfedge;
        HalfedgeRef h = h0;
        do {
            h_old.push_back(h);
            h = h->next;
        } while (h != h0);
    }

    uint32_t n = uint32_t(h_old.size());
    if (n < 3) return std::nullopt;

    std::vector<VertexRef> v_old(n);
    for (uint32_t i = 0; i < n; ++i) {
        v_old[i] = h_old[i]->vertex;
    }
	//create inner Vertex 
    std::vector<VertexRef> v_new(n);
    for (uint32_t i = 0; i < n; ++i) {
        v_new[i] = emplace_vertex();
        v_new[i]->position = v_old[i]->position;
        interpolate_data({VertexCRef(v_old[i])}, v_new[i]);
    }

    //Create diagonal edges
    std::vector<EdgeRef> e_diag(n);
    std::vector<HalfedgeRef> h_up(n);
    std::vector<HalfedgeRef> h_down(n);

    for (uint32_t i = 0; i < n; ++i) {
        e_diag[i] = emplace_edge();
        h_up[i] = emplace_halfedge();
        h_down[i] = emplace_halfedge();

        h_up[i]->twin = h_down[i];
        h_down[i]->twin = h_up[i];

        h_up[i]->vertex = v_old[i];
        h_down[i]->vertex = v_new[i];

        h_up[i]->edge = e_diag[i];
        h_down[i]->edge = e_diag[i];

        e_diag[i]->halfedge = h_up[i];
    }

    //create inner edges
    std::vector<EdgeRef> e_inner(n);
    std::vector<HalfedgeRef> h_in(n);
    std::vector<HalfedgeRef> h_in_twin(n);

    for (uint32_t i = 0; i < n; ++i) {
        uint32_t j = (i + 1) % n;

        e_inner[i] = emplace_edge();
        h_in[i] = emplace_halfedge();
        h_in_twin[i] = emplace_halfedge();

        h_in[i]->twin = h_in_twin[i];
        h_in_twin[i]->twin = h_in[i];

        h_in[i]->vertex = v_new[i];
        h_in_twin[i]->vertex = v_new[j];

        h_in[i]->edge = e_inner[i];
        h_in_twin[i]->edge = e_inner[i];

        e_inner[i]->halfedge = h_in[i];
    }

    //f set to inner face
    for (uint32_t i = 0; i < n; ++i) {
        uint32_t j = (i + 1) % n;
        h_in[i]->next = h_in[j];
        h_in[i]->face = f;
    }
    f->halfedge = h_in[0];

    //side faces
    for (uint32_t i = 0; i < n; ++i) {

        uint32_t i1 = (i + 1) % n;
        FaceRef f_side = emplace_face(false);

        HalfedgeRef h_o = h_old[i];
        HalfedgeRef h_v_up = h_up[i1];
        HalfedgeRef h_t = h_in_twin[i];
        HalfedgeRef h_v_down = h_down[i];

        h_o->next = h_v_up;
        h_v_up->next = h_t;
        h_t->next = h_v_down;
        h_v_down->next = h_o;

        h_o->face = f_side;
        h_v_up->face = f_side;
        h_t->face = f_side;
        h_v_down->face = f_side;

        f_side->halfedge = h_o;
    }

    //make sure new_vertex are defined
    for (uint32_t i = 0; i < n; ++i) {
        v_new[i]->halfedge = h_in[i];
    }

    return f;
}

/*
 * flip_edge: rotate non-boundary edge ccw inside its containing faces
 *  e: edge to flip
 *
 * if e is a boundary edge, does nothing and returns std::nullopt
 * if flipping e would create an invalid mesh, does nothing and returns std::nullopt
 *
 * otherwise returns the edge, post-rotation
 *
 * does not create or destroy mesh elements.
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(EdgeRef e) {
	//A2L1: Flip Edge
	//create
    HalfedgeRef h = e->halfedge;
    HalfedgeRef t = h->twin; 
    FaceRef hf = h->face;
    FaceRef tf = t->face;

	
	if (hf->boundary || tf->boundary) return std::nullopt;

 	auto lastEdge = [](HalfedgeRef x) -> HalfedgeRef {
		HalfedgeRef cur = x; 
		while (cur->next != x) cur = cur->next;
		return cur;
	};
	HalfedgeRef ph = lastEdge(h);
	HalfedgeRef pt = lastEdge(t);       
	if (ph == t || pt == h) return std::nullopt;
	
    HalfedgeRef h_next = h->next;
    HalfedgeRef t_next = t->next;

	VertexRef va = h->vertex;
	VertexRef vb = t->vertex;
    VertexRef vc = h_next->twin->vertex;
    VertexRef vd = t_next->twin->vertex;

	if (vc == vd) return std::nullopt;


	
	auto neighbors = [&](VertexRef x, VertexRef y, EdgeRef e) -> bool {
		HalfedgeRef start = x->halfedge;
		HalfedgeRef cur = start;
		do { 
			VertexRef nbr = cur->twin->vertex;
			if (nbr == y && cur->edge != e) return true;
			cur = cur->twin->next;
		} while (cur != start);
		return false;
	};

	if (neighbors(vc, vd, e)) return std::nullopt;
 
	h->vertex = vc;
	t->vertex = vd;

    h->face = tf;
    t->face = hf;

    h_next->face = tf;
    t_next->face = hf;
	
    HalfedgeRef hf_chain_start = h_next->next;
    HalfedgeRef tf_chain_start = t_next->next;

    t->next = hf_chain_start;
    ph->next = t_next;
    t_next->next = t;
  
    h->next = tf_chain_start;
    pt->next = h_next;
    h_next->next = h;
	 
    hf->halfedge = t;
    tf->halfedge = h;

    va->halfedge = t_next;
    vb->halfedge = h_next;
    vc->halfedge = h;
    vd->halfedge = t;
	
    return e;
}


/*
 * make_boundary: add non-boundary face to boundary
 *  face: the face to make part of the boundary
 *
 * if face ends up adjacent to other boundary faces, merge them into face
 *
 * if resulting mesh would be invalid, does nothing and returns std::nullopt
 * otherwise returns face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::make_boundary(FaceRef face) {
	//A2Lx7: (OPTIONAL) make_boundary

	return std::nullopt; //TODO: actually write this code!
}

/*
 * dissolve_vertex: merge non-boundary faces adjacent to vertex, removing vertex
 *  v: vertex to merge around
 *
 * if merging would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_vertex(VertexRef v) {
	// A2Lx1 (OPTIONAL): Dissolve Vertex

    return std::nullopt;
}

/*
 * dissolve_edge: merge the two faces on either side of an edge
 *  e: the edge to dissolve
 *
 * merging a boundary and non-boundary face produces a boundary face.
 *
 * if the result of the merge would be an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_edge(EdgeRef e) {
	// A2Lx2 (OPTIONAL): dissolve_edge

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data
	
    return std::nullopt;
}

/* collapse_edge: collapse edge to a vertex at its middle
 *  e: the edge to collapse
 *
 * if collapsing the edge would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(EdgeRef e) {
	//A2L3: Collapse Edge
 
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;


	VertexRef v_keep = h->vertex;
	VertexRef v_kill = t->vertex;

	FaceRef f_h = h->face;
	FaceRef f_t = t->face;

	bool h_is_tri = (!f_h->boundary && f_h->degree() == 3);
	bool t_is_tri = (!f_t->boundary && f_t->degree() == 3);
 

	auto LastEdge  = [](HalfedgeRef x) -> HalfedgeRef {
		HalfedgeRef p = x;
		while (p->next != x) p = p->next;
		return p;
	};

	auto InvalidHalfedge = [this]() { return this->halfedges.end(); };

	auto get_nbrs = [this](VertexRef v, VertexRef exclude) -> std::set<VertexRef> {
		std::set<VertexRef> nbrs;
		HalfedgeRef start = v->halfedge;
		HalfedgeRef cur = start;
 
		if (start == this->halfedges.end()) return nbrs;

		do {
			VertexRef nbr = cur->twin->vertex;
			if (nbr != exclude) nbrs.insert(nbr);
			cur = cur->twin->next;
		} while (cur != start);

		return nbrs;
	};

 

	//detecting invalid collapses
	std::set<VertexRef> keep_nbrs = get_nbrs(v_keep, v_kill);
	std::set<VertexRef> kill_nbrs = get_nbrs(v_kill, v_keep);

	std::set<VertexRef> intersect;
	for (VertexRef x : keep_nbrs) {
		if (kill_nbrs.find(x) != kill_nbrs.end()) intersect.insert(x);
	}

	auto TriangleThirdVertex = [](HalfedgeRef he) -> VertexRef {
		return he->next->twin->vertex;
	};
	std::set<VertexRef> allowed;
	if (!f_h->boundary && f_h->degree() == 3) allowed.insert(TriangleThirdVertex(h));
	if (!f_t->boundary && f_t->degree() == 3) allowed.insert(TriangleThirdVertex(t));

	for (VertexRef x : intersect) {
		if (allowed.find(x) == allowed.end()) {
			return std::nullopt;
		}
	}
   

	//updating data
	Vec3 midpoint = (v_keep->position + v_kill->position) * 0.5f;
	v_keep->position = midpoint;
	interpolate_data({VertexCRef(v_keep), VertexCRef(v_kill)}, v_keep);

	{
		HalfedgeRef start = v_kill->halfedge;
		if (start != this->halfedges.end()) {
			HalfedgeRef cur = start;
			do {
				cur->vertex = v_keep;
				cur = cur->twin->next;
			} while (cur != start);
		}
	}
 
	//update faces
	auto FixFaceEdgeTri = [this, &LastEdge](HalfedgeRef old_h, HalfedgeRef new_h) {
		FaceRef f = old_h->face;
		HalfedgeRef prev = LastEdge(old_h);

		prev->next = new_h;
		new_h->next = old_h->next;
		new_h->face = f;

		if (f->halfedge == old_h) f->halfedge = new_h;
	};

	auto RemoveIncidentTriangle = [this, &LastEdge, &FixFaceEdgeTri, &InvalidHalfedge]
	(HalfedgeRef he, VertexRef v_keep, VertexRef v_kill) {

		FaceRef f_tri = he->face;

		// triangle halfedges:
		HalfedgeRef he_next = he->next;      
		HalfedgeRef he_prev = LastEdge(he);  
		VertexRef x = he_next->twin->vertex;  

		EdgeRef e_dup = he_next->edge;
		HalfedgeRef hx = he_next;     
		HalfedgeRef hx_twin = hx->twin;  

		EdgeRef e_keep = he_prev->edge;
		HalfedgeRef hk = he_prev;      
		HalfedgeRef hk_twin = hk->twin; 

  
 
		FixFaceEdgeTri(hx_twin, hk);
 
		hk->edge = e_keep;
		hk_twin->edge = e_keep;
		e_keep->halfedge = hk;
  
		this->erase_face(f_tri);

 
		hx->next = InvalidHalfedge();
		hx->face = this->faces.end();
		hx->vertex = this->vertices.end();

		hx_twin->next = InvalidHalfedge();
		hx_twin->face = this->faces.end();
		hx_twin->vertex = this->vertices.end();

		if (v_keep->halfedge == hx || v_keep->halfedge == hx_twin) v_keep->halfedge = hk_twin;
		if (v_kill->halfedge == hx || v_kill->halfedge == hx_twin) v_kill->halfedge = he; // temporary
		if (x->halfedge == hx || x->halfedge == hx_twin) x->halfedge = hk;

 

		this->erase_halfedge(hx);
		this->erase_halfedge(hx_twin);
		this->erase_edge(e_dup);

 

 
		x->halfedge = hk;
	};

	if (!f_h->boundary && f_h->degree() == 3) {
		RemoveIncidentTriangle(h, v_keep, v_kill);
	}
	if (!f_t->boundary && f_t->degree() == 3) {
		RemoveIncidentTriangle(t, v_keep, v_kill);
	}
 

	auto unlinkEdge = [this, &LastEdge](HalfedgeRef he) {
		FaceRef f = he->face;
		HalfedgeRef prev = LastEdge(he);
		prev->next = he->next;
		if (f->halfedge == he) f->halfedge = he->next;
	};

	if (!h_is_tri) unlinkEdge(h);
	if (!t_is_tri) unlinkEdge(t);

	auto pick_valid = [&](VertexRef v) -> HalfedgeRef {
		for (auto he = this->halfedges.begin(); he != this->halfedges.end(); ++he) {
			if (he->vertex == v) {
				if (he != h && he != t && he->face != this->faces.end()) {
					return he;
				}
			}
		}
		return this->halfedges.end();
	};
 
	if (v_keep->halfedge == h || v_keep->halfedge == t || v_keep->halfedge == this->halfedges.end()) {
		HalfedgeRef repl = pick_valid(v_keep);
		if (repl != this->halfedges.end()) v_keep->halfedge = repl;
	}
 
	erase_halfedge(h);
	erase_halfedge(t);
	erase_edge(e);
	erase_vertex(v_kill);

	return v_keep;
}

/*
 * collapse_face: collapse a face to a single vertex at its center
 *  f: the face to collapse
 *
 * if collapsing the face would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(FaceRef f) {
	//A2Lx3 (OPTIONAL): Collapse Face

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
	// (also works for bone_weights data on vertices!)

    return std::nullopt;
}

/*
 * weld_edges: glue two boundary edges together to make one non-boundary edge
 *  e, e2: the edges to weld
 *
 * if welding the edges would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns e, updated to represent the newly-welded edge
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::weld_edges(EdgeRef e, EdgeRef e2) {
	//A2Lx8: Weld Edges

	//Reminder: use interpolate_data() to merge bone_weights data on vertices!

    return std::nullopt;
}



/*
 * bevel_positions: compute new positions for the vertices of a beveled vertex/edge
 *  face: the face that was created by the bevel operation
 *  start_positions: the starting positions of the vertices
 *     start_positions[i] is the starting position of face->halfedge(->next)^i
 *  direction: direction to bevel in (unit vector)
 *  distance: how far to bevel
 *
 * push each vertex from its starting position along its outgoing edge until it has
 *  moved distance `distance` in direction `direction`. If it runs out of edge to
 *  move along, you may choose to extrapolate, clamp the distance, or do something
 *  else reasonable.
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after bevel_vertex or bevel_edge.
 * (So you can assume the local topology is set up however your bevel_* functions do it.)
 *
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::bevel_positions(FaceRef face, std::vector<Vec3> const &start_positions, Vec3 direction, float distance) {
	//A2Lx5h / A2Lx6h (OPTIONAL): Bevel Positions Helper
	
	// The basic strategy here is to loop over the list of outgoing halfedges,
	// and use the preceding and next vertex position from the original mesh
	// (in the start_positions array) to compute an new vertex position.
	
}

/*
 * extrude_positions: compute new positions for the vertices of an extruded face
 *  face: the face that was created by the extrude operation
 *  move: how much to translate the face
 *  shrink: amount to linearly interpolate vertices in the face toward the face's centroid
 *    shrink of zero leaves the face where it is
 *    positive shrink makes the face smaller (at shrink of 1, face is a point)
 *    negative shrink makes the face larger
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after extrude_face.
 * (So you can assume the local topology is set up however your extrude_face function does it.)
 *
 * Using extrude face in the GUI will assume a shrink of 0 to only extrude the selected face
 * Using bevel face in the GUI will allow you to shrink and increase the size of the selected face
 * 
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::extrude_positions(FaceRef face, Vec3 move, float shrink) {
	//A2L4h: Extrude Positions Helper

	//General strategy:
	// use mesh navigation to get starting positions from the surrounding faces,
	// compute the centroid from these positions + use to shrink,
	// offset by move
    if (face == faces.end()) return;
    if (face->boundary) return;

    std::vector<VertexRef> vs;
    std::vector<Vec3> old_pos ;

    HalfedgeRef h0 = face->halfedge;
    HalfedgeRef h = h0;

    do {
        VertexRef v_new = h->vertex;
        HalfedgeRef h_down = h->twin->next;
        VertexRef v_old = h_down->twin->vertex;

        vs.push_back(v_new);
        old_pos.push_back(v_old->position);

        h = h->next;
    } while (h != h0);

    uint32_t n = uint32_t(vs.size());
    if (n < 3) return;

    //Compute centroid
    Vec3 centroid(0.0f, 0.0f, 0.0f);
    for (const Vec3 &p : old_pos) {
        centroid += p;
    }
    centroid /= float(n);

    //update positions
    for (uint32_t i = 0; i < n; ++i) {
        Vec3 dir = old_pos[i]-centroid;
        Vec3 new_pos = centroid+dir*(1.0f-shrink)+move;
        vs[i]->position = new_pos;
    }
}

