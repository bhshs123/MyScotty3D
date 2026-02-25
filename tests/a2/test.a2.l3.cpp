#include "test.h"
#include "geometry/halfedge.h"

static void expect_collapse(Halfedge_Mesh &mesh, Halfedge_Mesh::EdgeRef edge, Halfedge_Mesh const &after) {
	if (auto ret = mesh.collapse_edge(edge)) {
		if (auto msg = mesh.validate()) {
			throw Test::error("Invalid mesh: " + msg.value().second);
		}
		// check mesh shape:
		if (auto difference = Test::differs(mesh, after, Test::CheckAllBits)) {
			throw Test::error("Resulting mesh did not match expected: " + *difference);
		}
	} else {
		throw Test::error("collapse_edge rejected operation!");
	}
}

/*
BASIC CASE

Initial mesh:
0--1\
|  | \
2--3--4
|  | /
5--6/

Collapse Edge on Edge: 2-3

After mesh:
0-----1\
 \   /  \
  \ /    \
   2------3
  / \    /
 /   \  /
4-----5/
*/
Test test_a2_l3_collapse_edge_basic_simple("a2.l3.collapse_edge.basic.simple", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		  Vec3(-1.0f, 1.0f, 0.0f), 	Vec3(1.1f, 1.0f, 0.0f),
		 Vec3(-1.2f, 0.0f, 0.0f),   	 Vec3(1.2f, 0.0f, 0.0f),  Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-1.0f, 0.0f), 		Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0, 2, 3, 1}, 
		{2, 5, 6, 3}, 
		{1, 3, 4}, 
		{3, 6, 4}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		  Vec3(-1.0f, 1.0f, 0.0f), 	Vec3(1.1f, 1.0f, 0.0f),
		 			Vec3(0.0f, 0.0f, 0.0f),  			Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-1.0f, 0.0f), 		Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0, 2, 1}, 
		{2, 4, 5}, 
		{1, 2, 3}, 
		{2, 5, 3}
	});

	expect_collapse(mesh, edge, after);
});

/*
EDGE CASE

Initial mesh:
0--1\
|\ | \
| \2--3
|  | /
4--5/

Collapse Edge on Edge: 0-1

After mesh:
    0--\
   / \  \
  /   \  \
 /     1--2
/      | /
3------4/
*/
Test test_a2_l3_collapse_edge_edge_boundary("a2.l3.collapse_edge.edge.boundary", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                         Vec3(1.2f, 0.0f, 0.0f),  Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-0.7f, 0.0f), Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0, 2, 1}, 
		{0, 4, 5, 2}, 
		{1, 2, 3}, 
		{2, 5, 3}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		       Vec3(0.05f, 1.05f, 0.0f), 
		                         Vec3(1.2f, 0.0f, 0.0f),  Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-0.7f, 0.0f), Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0, 1, 2}, 
		{0, 3, 4, 1}, 
		{1, 4, 2}
	});

	expect_collapse(mesh, edge, after);
});
/*

CASE 1: 3-FAN CENTER COLLAPSE (Valid)

Initial mesh (A pyramid flattened):

0

/|\

/ | \

/ 3 \

/ / \ \

1---------2

Collapse Edge on Edge: 3-0

After mesh (Leaves a single triangle):

0

/ \

/ \

/ \

/ \

1---------2

*/

Test test_a2_l3_collapse_edge_3_fan("a2.l3.collapse_edge.3_fan", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(0.0f, 1.0f, 0.0f), // 0

Vec3(-1.0f, -1.0f, 0.0f), // 1

Vec3(1.0f, -1.0f, 0.0f), // 2

Vec3(0.0f, 0.0f, 0.0f) // 3 (Center)

}, {

{0, 1, 3},

{1, 2, 3},

{2, 0, 3}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({

Vec3(0.0f, 0.5f, 0.0f),

Vec3(-1.0f, -1.0f, 0.0f), // 1

Vec3(1.0f, -1.0f, 0.0f) // 2

}, {

{0, 1, 2}

});

expect_collapse(mesh, edge, after);

});

/*

Just triangle

0

/ \

/ \

/ \

/ \

1---------2

Collapse Edge on Edge: 0-1

*/

Test test_a2_l3_collapse_edge_3("a2.l3.collapse_edge.31", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(0.0f, 1.0f, 0.0f), // 0

Vec3(-1.0f, -1.0f, 0.0f), // 1

Vec3(1.0f, -1.0f, 0.0f) // 2

}, {

{0, 1, 2}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->edge;

if (mesh.collapse_edge(edge)) {

throw Test::error("collapse_edge should not work on a triangle case.");

}

});

/*

CASE: BOUNDARY EDGE COLLAPSE (Valid)

Initial mesh:

0---------3

/ \ |

/ \ |

/ \ |

1-------2----

Collapse Edge: 0-1

After mesh:

0'--------3

\ |

\ |

\ |

2-----

*/

Test test_a2_l3_collapse_edge_3_more("a2.l3.collapse_edge.3.more", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(0.0f, 1.0f, 0.0f), // 0

Vec3(-1.0f, -1.0f, 0.0f), // 1

Vec3(1.0f, -1.0f, 0.0f), // 2

Vec3(1.0f, 1.0f, 0.0f) // 3

}, {

{0, 1, 2},

{0, 2, 3}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->edge;

Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({

Vec3(-0.5f, 0.0f, 0.0f),

Vec3(1.0f, -1.0f, 0.0f),

Vec3(1.0f, 1.0f, 0.0f)

}, {

{0, 1, 2}

});

expect_collapse(mesh, edge, after);

});

/*

CASE 2: INTERIOR GRID (Valid)

Initial mesh:

0---1---2

| /| /|

| / | / |

3---4---5

| /| /|

| / | / |

6---7---8

Collapse Edge on Edge: 4-1

After mesh:

0 2

| \ /|

| \ / |

3---4---5

| /| /|

| / | / |

6---7---8

*/

Test test_a2_l3_collapse_edge_interior_grid("a2.l3.collapse_edge.interior_grid", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(-1.0f, 1.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f), Vec3(1.0f, 1.0f, 0.0f), // 0, 1, 2

Vec3(-1.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f), // 3, 4, 5

Vec3(-1.0f,-1.0f, 0.0f), Vec3(0.0f,-1.0f, 0.0f), Vec3(1.0f,-1.0f, 0.0f) // 6, 7, 8

}, {

{0, 3, 1}, {3, 4, 1},

{1, 4, 2}, {4, 5, 2},

{3, 6, 4}, {6, 7, 4},

{4, 7, 5}, {7, 8, 5}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->twin->next->next->edge;

Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({

Vec3(-1.0f, 1.0f, 0.0f), Vec3(1.0f, 1.0f, 0.0f),

Vec3(-1.0f, 0.0f, 0.0f), Vec3(0.0f, 0.5f, 0.0f), Vec3(1.0f, 0.0f, 0.0f),

Vec3(-1.0f,-1.0f, 0.0f), Vec3(0.0f,-1.0f, 0.0f), Vec3(1.0f,-1.0f, 0.0f)

}, {

{0, 2, 3},

{3, 4, 1},

{2, 5, 3},

{5, 6, 3},

{3, 6, 4},

{6, 7, 4}

});

expect_collapse(mesh, edge, after);

});

/*

CASE 2: INTERIOR GRID (INVALID)

Initial mesh:

0---1---2

| | |

| | |

3. | 4

| | |

| | |

5---6---7

Collapse Edge on Edge: 1-7

*/

Test test_a2_l3_collapse_edge_bad_grid("a2.l3.collapse_edge.bad_grid", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(-1.0f, 1.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f), Vec3(1.0f, 1.0f, 0.0f), // 0, 1, 2

Vec3(-1.0f, 0.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f), // 3, 4

Vec3(-1.0f,-1.0f, 0.0f), Vec3(0.0f,-1.0f, 0.0f), Vec3(1.0f,-1.0f, 0.0f) // 5, 6, 7

}, {

{0, 3, 5, 6, 1},

{1, 6, 7, 4, 2}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->next->edge;

if (mesh.collapse_edge(edge)) {

throw Test::error("collapse_edge should not work on a non-manifold case.");

}

});

/*

CASE 2: INTERIOR GRID (VALID)

Initial mesh:

0---1---2

| | |

| | |

3. | 4

| | |

| | |

5---6---7

Collapse Edge on Edge: 0-3

----1---2

| | |

| | |

3. | 4

| | |

| | |

5---6---7

*/

Test test_a2_l3_collapse_edge_good_grid("a2.l3.collapse_edge.good_grid", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(-1.0f, 1.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f), Vec3(1.0f, 1.0f, 0.0f), // 0, 1, 2

Vec3(-1.0f, 0.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f), // 3, 4

Vec3(-1.0f,-1.0f, 0.0f), Vec3(0.0f,-1.0f, 0.0f), Vec3(1.0f,-1.0f, 0.0f) // 5, 6, 7

}, {

{0, 3, 5, 6, 1},

{1, 6, 7, 4, 2}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->edge;

Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({

Vec3(-1.0f, 0.5f, 0.0f),

Vec3(0.0f, 1.0f, 0.0f), // 1: old 1

Vec3(1.0f, 1.0f, 0.0f), // 2: old 2

Vec3(1.0f, 0.0f, 0.0f), // 3: old 4

Vec3(-1.0f,-1.0f, 0.0f), // 4: old 5

Vec3(0.0f,-1.0f, 0.0f), // 5: old 6

Vec3(1.0f,-1.0f, 0.0f) // 6: old 7

}, {

{0, 4, 5, 1},

{1, 5, 6, 3, 2}

});

expect_collapse(mesh, edge, after);

});

/*

CASE 3: INVALID BOUNDARY PINCH (bad pinch)

:

Top ring: 0---1

\ /

2

Bottom ring: 3---4

\ /

5

edge 0-1.

*/

Test test_a2_l3_collapse_edge_invalid_pinch("a2.l3.collapse_edge.invalid_pinch", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3( 0.0f, 1.0f, 1.0f), Vec3( 1.0f,-1.0f, 1.0f), Vec3(-1.0f,-1.0f, 1.0f),

Vec3( 0.0f, 1.0f, 0.0f), Vec3( 1.0f,-1.0f, 0.0f), Vec3(-1.0f,-1.0f, 0.0f)

}, {

{0, 1, 4}, {0, 4, 3}, // Side 1

{1, 2, 5}, {1, 5, 4}, // Side 2

{2, 0, 3}, {2, 3, 5} // Side 3

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->edge;

if (mesh.collapse_edge(edge)) {

throw Test::error("collapse_edge should not work on a degenerate case.");

}

});
 