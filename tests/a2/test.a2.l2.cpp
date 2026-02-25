#include "test.h"
#include "geometry/halfedge.h"

static void expect_split(Halfedge_Mesh &mesh, Halfedge_Mesh::EdgeRef edge, Halfedge_Mesh const &after) {
	if (auto ret = mesh.split_edge(edge)) {
		if (auto msg = mesh.validate()) {
			throw Test::error("Invalid mesh: " + msg.value().second);
		}
		// check mesh shape:
		if (auto difference = Test::differs(mesh, after, Test::CheckAllBits)) {
			throw Test::error("Resulting mesh did not match expected: " + *difference);
		}
	} else {
		throw Test::error("split_edge rejected operation!");
	}
}

/*
BASIC CASE:

Initial mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/

Split Edge on Edge: 1-4

After mesh:
0--1\
|\ | \
| \2--3
|  | /
4--5/
*/
Test test_a2_l2_split_edge_basic_simple("a2.l2.split_edge.basic.simple", []() {
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
		                         Vec3(1.25f, 0.0f, 0.0f),  Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 4, 5, 2}, 
		{0, 2, 1}, 
		{1, 2, 3}, 
		{2, 5, 3}
	});

	expect_split(mesh, edge, after);
});

/*
EDGE CASE: 

Initial mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/

Split Edge on Edge: 0-1

After mesh:
0--1--2\
|  /  | \
| /   |  3
|/    | /
4-----5/
*/
Test test_a2_l2_split_edge_edge_boundary("a2.l2.split_edge.edge.boundary", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 3, 4, 1}, 
		{1, 4, 2}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f),  Vec3(0.05f, 1.05f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            						Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), 							Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 4, 1}, 
		{1, 4, 5, 2}, 
		{2, 5, 3}
	});

	expect_split(mesh, edge, after);
});

/*

CASE 1: TWO TRIANGLES (Interior Edge)

Initial mesh (Square made of 2 triangles):

0-------3

| \ |

| \ |

| \ |

1-------2

Split Edge on Edge: 0-2 (Diagonal)

After mesh (Square made of 4 triangles):

0-------4

| \ / |

| \2/ |

| / \ |

1-------3

*/

Test test_a2_l2_split_edge_two_triangles("a2.l2.split_edge.two_triangles", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(-1.0f, 1.0f, 0.0f), Vec3(-1.0f, -1.0f, 0.0f),

Vec3(1.0f, -1.0f, 0.0f), Vec3(1.0f, 1.0f, 0.0f)

}, {

{0, 1, 2},

{0, 2, 3}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({

Vec3(-1.0f, 1.0f, 0.0f), Vec3(-1.0f, -1.0f, 0.0f),

Vec3(0.0f, 0.0f, 0.0f), // added vertex 2

Vec3(1.0f, -1.0f, 0.0f), Vec3(1.0f, 1.0f, 0.0f)

}, {

{0, 1, 2},

{2, 1, 3},

{0, 2, 4},

{2, 3, 4}

});

expect_split(mesh, edge, after);

});

/*

CASE 2: SINGLE TRIANGLE (Boundary Edge)

Initial mesh:

0

|\

| \

| \

1---2

Split Edge on Edge: 1-2 (Bottom boundary)

After mesh:

0

|\ \

| \ \

| \ \

1---2---3

*/

Test test_a2_l2_split_edge_triangle_boundary("a2.l2.split_edge.triangle_boundary", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(0.0f, 1.0f, 0.0f), Vec3(-1.0f, -1.0f, 0.0f), Vec3(1.0f, -1.0f, 0.0f)

}, {

{0, 1, 2}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({

Vec3(0.0f, 1.0f, 0.0f), Vec3(-1.0f, -1.0f, 0.0f),

Vec3(0.0f, -1.0f, 0.0f), // added vertex 2

Vec3(1.0f, -1.0f, 0.0f)

}, {

{0, 1, 2},

{0, 2, 3}

});

expect_split(mesh, edge, after);

});

/*

CASE 3: TWO QUADS (Interior Edge)

Initial mesh:

0---1---2

| | |

| | |

| | |

3---4---5

Split Edge on Edge: 1-4

After mesh:

0---1---3

| \ | |

| 2 |

| | \ |

4---5---6

*/

Test test_a2_l2_split_edge_two_quads("a2.l2.split_edge.two_quads", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(-1.0f, 1.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f), Vec3(1.0f, 1.0f, 0.0f),

Vec3(-1.0f, -1.0f, 0.0f), Vec3(0.0f, -1.0f, 0.0f), Vec3(1.0f, -1.0f, 0.0f)

}, {

{0, 3, 4, 1},

{1, 4, 5, 2}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({

Vec3(-1.0f, 1.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f),

Vec3(0.0f, 0.0f, 0.0f), // added vertex 2

Vec3(1.0f, 1.0f, 0.0f),

Vec3(-1.0f, -1.0f, 0.0f), Vec3(0.0f, -1.0f, 0.0f), Vec3(1.0f, -1.0f, 0.0f)

}, {

{0, 4, 5, 2},

{0, 2, 1},

{2, 5, 6},

{1, 2, 6, 3}

});

expect_split(mesh, edge, after);

});

/*

CASE 4: HIGH DEGREE VERTEX (Star Center)

Initial mesh (Triangle Fan):

1-------4

| \ / |

| \ /. |

| 0. |

| / \. |

| / \ |

2-------3

Split Edge on Edge: 0-2

After mesh:

1---------5

| \ /

| | \ /

| | 0

| |/\

| 2 \

| / \ \

3--------4

*/

Test test_a2_l2_split_edge_star_center("a2.l2.split_edge.star_center", []() {

Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({

Vec3(0.0f, 0.0f, 0.0f),

Vec3(-1.0f, 1.0f, 0.0f), Vec3(-1.0f, -1.0f, 0.0f),

Vec3(1.0f, -1.0f, 0.0f), Vec3(1.0f, 1.0f, 0.0f)

}, {

{0, 1, 2}, {0, 2, 3}, {0, 3, 4}, {0, 4, 1}

});

Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({

Vec3(0.0f, 0.0f, 0.0f),

Vec3(-1.0f, 1.0f, 0.0f),

Vec3(-0.5f, -0.5f, 0.0f), // added vertex 2

Vec3(-1.0f, -1.0f, 0.0f),

Vec3(1.0f, -1.0f, 0.0f), Vec3(1.0f, 1.0f, 0.0f)

}, {

{0, 1, 2},

{2, 1, 3},

{0, 2, 4},

{2, 3, 4},

{0, 4, 5},
    
{0, 5, 1}

});

expect_split(mesh, edge, after);

});