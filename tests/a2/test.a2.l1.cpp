#include "test.h"
#include "geometry/halfedge.h"

static void expect_flip(Halfedge_Mesh &mesh, Halfedge_Mesh::EdgeRef edge, Halfedge_Mesh const &after) {
	if (auto ret = mesh.flip_edge(edge)) {
		if (auto msg = mesh.validate()) {
			throw Test::error("Invalid mesh: " + msg.value().second);
		}
		// check if returned edge is the same edge
		if (ret != edge) {
			throw Test::error("Did not return the same edge!");
		}
		// check mesh shape:
		if (auto difference = Test::differs(mesh, after, Test::CheckAllBits)) {
			throw Test::error("Resulting mesh did not match expected: " + *difference);
		}
	} else {
		throw Test::error("flip_edge rejected operation!");
	}
}

/*
BASIC CASE

Initial mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/

Flip Edge on Edge: 1-4

After mesh:
0--1\
|\   \
| \---2
|    /
3--4/
*/
Test test_a2_l1_flip_edge_basic_simple("a2.l1.flip_edge.basic.simple", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 3, 4, 1}, 
		{1, 4, 2}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 3, 4, 2}, 
		{0, 2, 1}
	});

	expect_flip(mesh, edge, after);
});

/*
EDGE CASE

Initial mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/

Flip Edge on Edge: 3-4

After mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/
*/
Test test_a2_l1_flip_edge_edge_boundary("a2.l1.flip_edge.edge.boundary", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 3, 4, 1}, 
		{1, 4, 2}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	if (mesh.flip_edge(edge)) {
		throw Test::error("flip_edge should not work at the boundary.");
	}
});

/*

QUAD CASE

Initial mesh:

0---1---2

| | |

| | |

| | |

5---4---3

Flip Edge on Edge: 1-4

After mesh:

0---1---2

| \ |

| \ |

| \ |

5---4---3

*/

Test test_a2_l1_flip_edge_quad("a2.l1.flip_edge.quad", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(0, 1, 0), Vec3(1, 1, 0), Vec3(2, 1, 0),

Vec3(2, 0, 0), Vec3(1, 0, 0), Vec3(0, 0, 0)

}, {

{0, 5, 4, 1},

{1, 4, 3, 2}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({

Vec3(0, 1, 0), Vec3(1, 1, 0), Vec3(2, 1, 0),

Vec3(2, 0, 0), Vec3(1, 0, 0), Vec3(0, 0, 0)

}, {

{0, 5, 4, 3}, {0, 3, 2, 1}

});

expect_flip(mesh, edge, after);

});

/*

DEGENERATE CASE

Initial mesh:

0---1

| /

| /

2

Flip Edge on Edge: 0-1

After mesh:

0---1

| /

| /

2

(flip should be rejected)

*/

Test test_a2_l1_flip_edge_degenerate("a2.l1.flip_edge.degenerate", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(0, 1, 0), Vec3(1, 1, 0), Vec3(0, 0, 0)

}, {

{0, 1, 2}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->edge;

if (mesh.flip_edge(edge)) {

throw Test::error("flip_edge should not work on a degenerate case.");

}

});

/*

HEXAGON + TRI

Initial mesh:

0---1---2---\

| \- |

| \- |

| \3

| |

| |

7---6---5---4

Flip Edge on Edge: 1-3

After mesh:

0---1---\

\ -\

|\ -\

| \ -\

| --------2

| |

| 3

| |

| |

7---6---5---4

*/

Test test_a2_l1_flip_edge_hex_TRI_ccw("a2.l1.flip_edge.hex_TRI.ccw", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(0, 2, 0), Vec3(1, 2, 0), Vec3(2, 2, 0), Vec3(3, 2, 0),

Vec3(3, 0, 0), Vec3(2, 0, 0), Vec3(1, 0, 0), Vec3(0, 0, 0)

}, {

{0, 7, 6, 5, 4, 3, 1},

{1, 3, 2}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->next->next->next->edge;

Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({

Vec3(0, 2, 0), Vec3(1, 2, 0), Vec3(2, 2, 0), Vec3(3, 2, 0),

Vec3(3, 0, 0), Vec3(2, 0, 0), Vec3(1, 0, 0), Vec3(0, 0, 0)

}, {

{0, 7, 6, 5, 4, 3, 2},

{0, 2, 1}

});

expect_flip(mesh, edge, after);

});

/*

HEXAGON + TRI

Initial mesh:

0---1---2---\

| \- |

| \- |

| \3

| |

| |

7---6---5---4

Flip Edge on Edge: 4-3

*/

Test test_a2_l1_flip_edge_hex_TRI_ccw_1("a2.l1.flip_edge.hex_TRI.degen1.ccw", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(0, 2, 0), Vec3(1, 2, 0), Vec3(2, 2, 0), Vec3(3, 2, 0),

Vec3(3, 0, 0), Vec3(2, 0, 0), Vec3(1, 0, 0), Vec3(0, 0, 0)

}, {

{0, 7, 6, 5, 4, 3, 1},

{1, 3, 2}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->next->next->edge;

if (mesh.flip_edge(edge)) {

throw Test::error("flip_edge should not work at the boundary.");

}

});

/*

HEXAGON + TRI

Initial mesh:

0---1---2---\

| \- |

| \- |

| \3

| |

| |

7---6---5---4

Flip Edge on Edge: 5-4

*/

Test test_a2_l1_flip_edge_hex_TRI_ccw_2("a2.l1.flip_edge.hex_TRI.degen2.ccw", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(0, 2, 0), Vec3(1, 2, 0), Vec3(2, 2, 0), Vec3(3, 2, 0),

Vec3(3, 0, 0), Vec3(2, 0, 0), Vec3(1, 0, 0), Vec3(0, 0, 0)

}, {

{0, 7, 6, 5, 4, 3, 1},

{1, 3, 2}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->next->edge;

if (mesh.flip_edge(edge)) {

throw Test::error("flip_edge should not work at the boundary.");

}

});

/*

HEXAGON + TRI

Initial mesh:

0---1---2---\

| \- |

| \- |

| \3

| |

| |

7---6---5---4

Flip Edge on Edge: 1-2

*/

Test test_a2_l1_flip_edge_hex_TRI_ccw_3("a2.l1.flip_edge.hex_TRI.degen3.ccw", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(0, 2, 0), Vec3(1, 2, 0), Vec3(2, 2, 0), Vec3(3, 2, 0),

Vec3(3, 0, 0), Vec3(2, 0, 0), Vec3(1, 0, 0), Vec3(0, 0, 0)

}, {

{0, 7, 6, 5, 4, 3, 1},

{1, 3, 2}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->next->next->next->next->edge;

if (mesh.flip_edge(edge)) {

throw Test::error("flip_edge should not work at the boundary.");

}

});

/*

HEXAGON + QUAD

Initial mesh:

0---1---2---\

| \ |

| \ |

| \ 3

| \ |

| \|

7---6---5---4

Flip Edge on Edge: 0-4

Final mesh:

0---1---2---3

| / |

| / |

| / |

| / |

| / |

7---6---5---4

*/

Test test_a2_l1_flip_edge_hex_quad_ccw("a2.l1.flip_edge.hex_quad.ccw", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(0, 2, 0), Vec3(1, 2, 0), Vec3(2, 2, 0), Vec3(3, 2, 0),

Vec3(3, 0, 0), Vec3(2, 0, 0), Vec3(1, 0, 0), Vec3(0, 0, 0)

}, {

{0, 7, 6, 5, 4},

{0, 4, 3, 2, 1}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->next->next->edge;

Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({

Vec3(0, 2, 0), Vec3(1, 2, 0), Vec3(2, 2, 0), Vec3(3, 2, 0),

Vec3(3, 0, 0), Vec3(2, 0, 0), Vec3(1, 0, 0), Vec3(0, 0, 0)

}, {

{0, 7, 3, 2, 1},

{3, 7, 6, 5, 4}

});

expect_flip(mesh, edge, after);

});

/*

TETRAHEDRON CASE

Initial mesh:

0

/|\

/ | \

/ | \

1---2---3

Flip Edge on Edge: 0-1

*/

Test test_a2_l1_flip_edge_tetrahedron01("a2.l1.flip_edge.tetrahedron01", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(0, 1, 1), // 0

Vec3(-1, -1, 0), // 1

Vec3(1, -1, 0), // 2

Vec3(0, 0, -1) // 3

}, {

{0, 1, 2},

{0, 2, 3},

{0, 3, 1},

{1, 3, 2}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->edge;

if (mesh.flip_edge(edge)) {

throw Test::error("flip_edge should not work at the boundary.");

}

});

/*

TETRAHEDRON CASE

Initial mesh:

0

/|\

/ | \

/ | \

1---2---3

Flip Edge on Edge: 1-2

*/

Test test_a2_l1_flip_edge_tetrahedron12("a2.l1.flip_edge.tetrahedron12", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(0, 1, 1), // 0

Vec3(-1, -1, 0), // 1

Vec3(1, -1, 0), // 2

Vec3(0, 0, -1) // 3

}, {

{0, 1, 2},

{0, 2, 3},

{0, 3, 1},

{1, 3, 2}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

if (mesh.flip_edge(edge)) {

throw Test::error("flip_edge should not work at the boundary.");

}

});

/*

TETRAHEDRON CASE

Initial mesh:

0

/|\

/ | \

/ | \

1---2---3

Flip Edge on Edge: 2-0

*/

Test test_a2_l1_flip_edge_tetrahedron20("a2.l1.flip_edge.tetrahedron20", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(0, 1, 1), // 0

Vec3(-1, -1, 0), // 1

Vec3(1, -1, 0), // 2

Vec3(0, 0, -1) // 3

}, {

{0, 1, 2},

{0, 2, 3},

{0, 3, 1},

{1, 3, 2}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

if (mesh.flip_edge(edge)) {

throw Test::error("flip_edge should not work at the boundary.");

}

});

/*

TETRAHEDRON CASE

Flip Edge on Edge: 0-2

*/

Test test_a2_l1_flip_edge_tetrahedron_02("a2.l1.flip_edge.tetrahedron.02", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(0, 1, 1), // 0

Vec3(-1, -1, 0), // 1

Vec3(1, -1, 0), // 2

Vec3(0, 0, -1) // 3

}, {

{0, 1, 2},

{0, 2, 3},

{0, 3, 1},

{1, 3, 2}

});

Halfedge_Mesh::EdgeRef edge = std::next(mesh.halfedges.begin(), 3)->edge;

if (mesh.flip_edge(edge)) {

throw Test::error("flip_edge should not work at the boundary.");

}

});

/*

TETRAHEDRON CASE

Flip Edge on Edge: 2-3

*/

Test test_a2_l1_flip_edge_tetrahedron_23("a2.l1.flip_edge.tetrahedron.23", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(0, 1, 1), // 0

Vec3(-1, -1, 0), // 1

Vec3(1, -1, 0), // 2

Vec3(0, 0, -1) // 3

}, {

{0, 1, 2},

{0, 2, 3},

{0, 3, 1},

{1, 3, 2}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->next->edge;

if (mesh.flip_edge(edge)) {

throw Test::error("flip_edge should not work at the boundary.");

}

});

/*

TETRAHEDRON CASE

Flip Edge on Edge: 3-0

*/

Test test_a2_l1_flip_edge_tetrahedron_30("a2.l1.flip_edge.tetrahedron.30", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(0, 1, 1), // 0

Vec3(-1, -1, 0), // 1

Vec3(1, -1, 0), // 2

Vec3(0, 0, -1) // 3

}, {

{0, 1, 2},

{0, 2, 3},

{0, 3, 1},

{1, 3, 2}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->next->next->edge;

if (mesh.flip_edge(edge)) {

throw Test::error("flip_edge should not work at the boundary.");

}

});

/*

TETRAHEDRON CASE

Flip Edge on Edge: 1-3

*/

Test test_a2_l1_flip_edge_tetrahedron_13("a2.l1.flip_edge.tetrahedron.13", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(0, 1, 1), // 0

Vec3(-1, -1, 0), // 1

Vec3(1, -1, 0), // 2

Vec3(0, 0, -1) // 3

}, {

{0, 1, 2},

{0, 2, 3},

{0, 3, 1},

{1, 3, 2}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->next->next->next->edge;

if (mesh.flip_edge(edge)) {

throw Test::error("flip_edge should not work at the boundary.");

}

});
 