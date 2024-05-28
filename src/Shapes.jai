
KodaShape :: struct (V: int = MAX_VERTICES, I: int = MAX_VERTICES) {
    type: ShapeType;

    union {
        aabb: AABB3;
        hull: HullShape(V, I);
        mesh: MeshShape;
        vertices: VerticesShape(V);
    }
}

ShapeType :: enum {
    ConvexHull;
    TriangleMesh;
    Vertices;
    // Box;
    // Sphere;
}

HullShape :: struct (V: int, I: int) {
    aabb: AABB3;
    vertices: KodaVertices(V);
    indices: KodaIndices(I);
}

MeshShape :: struct {
    aabb: AABB3;
    triangles: BoundingTree(Triangle3);
}

VerticesShape :: struct (V: int) {
    aabb: AABB3;
    vertices: KodaVertices(V);
}

create_detected_shape :: (vertices: [] Vector3, indices: [] int) -> KodaShape(MAX_VERTICES, MAX_VERTICES) {
    convex_hull := vertices.count <= MAX_VERTICES && indices.count <= MAX_VERTICES;
    convex_hull = convex_hull && detect_convex_hull(vertices, indices);

    print("Convex hull %\n", convex_hull);

    // print("convex: %\nhull: %\nvertex count: %\n\n", temp_convex, temp_hull, temp_vertex_count);

    if convex_hull {
        return create_convex_hull(vertices, indices);
    } else {
        return create_triangle_mesh(vertices, indices);
    }
}

create_convex_hull :: (vertices: [] Vector3, indices: [] int) -> KodaShape(MAX_VERTICES, MAX_VERTICES) {
    assert(vertices.count <= MAX_VERTICES, "You cannot create a polygon with more than MAX_VERTICES.");

    shape: KodaShape(MAX_VERTICES, MAX_VERTICES);
    shape.type = .ConvexHull;
    shape.hull.vertices.count = vertices.count;
    shape.hull.indices.count = indices.count;

    memcpy(shape.hull.vertices.fixed.data, vertices.data, vertices.count * size_of(Vector3));
    memcpy(shape.hull.indices.fixed.data, indices.data, indices.count * size_of(int));

    if vertices.count > 0 {
        shape.aabb.lower = vertices[0];
        shape.aabb.upper = vertices[0];

        for i: 1..vertices.count - 1 {
            shape.aabb.lower = min(shape.aabb.lower, vertices[i]);
            shape.aabb.upper = max(shape.aabb.upper, vertices[i]);
        }
    }

    return shape;
}

create_convex_hull :: (vertices: [$N] Vector3) -> KodaShape(N, 0) {
    shape: KodaShape(N, 0);
    shape.type = .ConvexHull;
    shape.hull.vertices.count = vertices.count;

    memcpy(shape.hull.vertices.fixed.data, vertices.data, vertices.count * size_of(Vector3));

    if vertices.count > 0 {
        shape.aabb.lower = vertices[0];
        shape.aabb.upper = vertices[0];

        for i: 1..vertices.count - 1 {
            shape.aabb.lower = min(shape.aabb.lower, vertices[i]);
            shape.aabb.upper = max(shape.aabb.upper, vertices[i]);
        }
    }

    return shape;
}

create_triangle_mesh :: (vertices: [] Vector3, indices: [] int) -> KodaShape(MAX_VERTICES, MAX_VERTICES) {
    shape: KodaShape;
    shape.type = .TriangleMesh;
    shape.mesh.triangles = create_bounding_tree(Triangle3);

    i := 0;
    while i < indices.count {
        defer i += 3;

        a := vertices[indices[i]];
        b := vertices[indices[i + 1]];
        c := vertices[indices[i + 2]];

        aabb: AABB3 = ---;
        aabb.lower = a;
        aabb.lower = min(aabb.lower, b);
        aabb.lower = min(aabb.lower, c);
        aabb.upper = a;
        aabb.upper = max(aabb.upper, b);
        aabb.upper = max(aabb.upper, c);

        triangle: Triangle3 = ---;
        triangle.a = a;
        triangle.b = b;
        triangle.c = c;

        create_proxy(*shape.mesh.triangles, aabb, triangle);
    }

    if vertices.count > 0 {
        shape.aabb.lower = vertices[0];
        shape.aabb.upper = vertices[0];

        for i: 1..vertices.count - 1 {
            shape.aabb.lower = min(shape.aabb.lower, vertices[i]);
            shape.aabb.upper = max(shape.aabb.upper, vertices[i]);
        }
    }

    return shape;
}

create_triangle_mesh :: (vertices: [] Vector3) -> KodaShape(MAX_VERTICES, MAX_VERTICES) {
    shape: KodaShape;
    shape.type = .TriangleMesh;
    shape.mesh.triangles = create_bounding_tree(Triangle3);

    i := 0;
    while i < vertices.count {
        defer i += 3;

        a := vertices[i];
        b := vertices[i + 1];
        c := vertices[i + 2];

        aabb: AABB3 = ---;
        aabb.lower = a;
        aabb.lower = min(aabb.lower, b);
        aabb.lower = min(aabb.lower, c);
        aabb.upper = a;
        aabb.upper = max(aabb.upper, b);
        aabb.upper = max(aabb.upper, c);

        triangle: Triangle3 = ---;
        triangle.a = a;
        triangle.b = b;
        triangle.c = c;

        create_proxy(*shape.mesh.triangles, aabb, triangle);
    }

    if vertices.count > 0 {
        shape.aabb.lower = vertices[0];
        shape.aabb.upper = vertices[0];

        for i: 1..vertices.count - 1 {
            shape.aabb.lower = min(shape.aabb.lower, vertices[i]);
            shape.aabb.upper = max(shape.aabb.upper, vertices[i]);
        }
    }

    return shape;
}

// create_vertices :: (vertices: [] Vector3) -> KodaShape {
//     assert(vertices.count <= MAX_VERTICES, "You cannot create a polygon with more than MAX_VERTICES.");

//     shape: KodaShape;
//     shape.type = .Vertices;
//     shape.vertices.count = vertices.count;

//     memcpy(shape.vertices.fixed.data, vertices.data, vertices.count * size_of(Vector3));

//     if vertices.count > 0 {
//         shape.aabb.lower = vertices[0];
//         shape.aabb.upper = vertices[0];

//         for i: 1..vertices.count - 1 {
//             shape.aabb.lower = min(shape.aabb.lower, vertices[i]);
//             shape.aabb.upper = max(shape.aabb.upper, vertices[i]);
//         }
//     }

//     return shape;
// }

detect_convex_hull :: (vertices: [] Vector3, indices: [] $I) -> bool {
    // 1. each edge must have 2 triangles
    // 2. the dot product of the normal of each 2 adjacent triangles must be negative (pointing away from each other)

    EPSILON :: 0.0001;

    Edge :: struct {
        v1: Vector3;
        v2: Vector3;
    }

    EdgeTriangles :: struct {
        edge_triangles: [2] Triangle3;
        // edge_normals: [2] Vector3;
        edge_triangle_bits: u8;
    }

    hash_function :: inline (edge: Edge) -> u32 {
        values: [] u32;
        values.data = xx *edge.v1;
        values.count = 6;

        hash: u32;
        for value: values {
            hash ^= value;
        }

        return hash;
    }

    compare_function :: inline (e1: Edge, e2: Edge) -> bool {
        return e1.v1 == e2.v1 && e1.v2 == e2.v2;
    }

    equalish :: inline (t1: Triangle3, t2: Triangle3) -> bool {
        // if we can find all 3 points somewhere on the other triangle we can assume the triangles are equal regardless of order
        // its either this, or one of the triangles is a line
        matches := true;
        matches = matches && (t1.a == t2.a || t1.a == t2.b || t1.a == t2.c);
        matches = matches && (t1.b == t2.a || t1.b == t2.b || t1.b == t2.c);
        matches = matches && (t1.c == t2.a || t1.c == t2.b || t1.c == t2.c);
        return matches;
    }

    edge_triangles: Table(Edge, EdgeTriangles, hash_function, compare_function);
    edge_triangles.allocator = temp;

    index := 0;
    while index < indices.count {
        defer index += 3;
        
        a := vertices[indices[index]];
        b := vertices[indices[index + 1]];
        c := vertices[indices[index + 2]];

        // we want to insert the triangle so that the triangle edge ab is always shared
        edges: [6] Edge = ---;
        edges[0] = .{a, b};
        edges[1] = .{b, a};
        edges[2] = .{b, c};
        edges[3] = .{c, b};
        edges[4] = .{c, a};
        edges[5] = .{a, c};

        insertion_bits: [6] u8 = ---;
        insertion_bits[0] = 0x1;
        insertion_bits[1] = 0x2;
        insertion_bits[2] = 0x1;
        insertion_bits[3] = 0x2;
        insertion_bits[4] = 0x1;
        insertion_bits[5] = 0x2;

        corrected_triangles: [6] Triangle3 = ---;
        corrected_triangles[0] = .{a, b, c};
        corrected_triangles[1] = .{a, b, c};
        corrected_triangles[2] = .{b, c, a};
        corrected_triangles[3] = .{b, c, a};
        corrected_triangles[4] = .{c, a, b};
        corrected_triangles[5] = .{c, a, b};

        triangle := Triangle3.{a, b, c};

        for edge, i: edges {
            edge_info := find_or_add(*edge_triangles, edge);

            // if the triangle is already accounted for check the next edge
            // could check the equal bits but we might as well just compare against 0,0,0 triangles
            found := equalish(triangle, edge_info.edge_triangles[0]);
            found = found || equalish(triangle, edge_info.edge_triangles[1]);
            if found {
                continue;
            }

            insertion_bit := insertion_bits[i];
            if edge_info.edge_triangle_bits & insertion_bit {
                print("Potential convex hull has an edge that's already taken in this winding. %\n", edge);
                return false;
            }

            edge_info.edge_triangle_bits |= insertion_bit;
            edge_info.edge_triangles[insertion_bit - 1] = corrected_triangles[i];
        }
    }

    for edge_info: edge_triangles {
        if edge_info.edge_triangle_bits != 0x3 {
            print("Not convex due to edge triangle count.\n");
            return false;
        }

        triangle1 := edge_info.edge_triangles[0];
        normal1 := cross(triangle1.b - triangle1.a, triangle1.c - triangle1.a);

        triangle2 := edge_info.edge_triangles[1];

        // a and b are both part of the testing line, so we'll check that both are satisfied
        // to hopefully prevent floating point issues
        // or not

        if dot(triangle2.a - triangle2.c, normal1) < -EPSILON {
            return false;
        }
    }

    return true;
}

#scope_file

#run {
    shape: KodaShape(0, 0);
    shape.aabb.lower = .{1, 2, 3};
    shape.aabb.upper = .{4, 5, 6};
    assert(shape.aabb == shape.hull.aabb);
    assert(shape.aabb == shape.mesh.aabb);
    assert(shape.aabb == shape.vertices.aabb);
}

// detect_hull :: (vertices: [] Vector3, indices: [] int) -> bool {
//     Edge :: struct {
//         v1: Vector3;
//         v2: Vector3;
//     }

//     EdgeTriangles :: struct {
//         edge_triangles: [2] Triangle3;
//         edge_triangle_count: int;
//     }

//     hash_function :: (edge: Edge) -> u32 {
//         values: [] u32;
//         values.data = xx *edge.v1;
//         values.count = 6;

//         hash: u32;
//         for value: values {
//             hash ^= value;
//         }

//         return hash;
//     }

//     compare_function :: (e1: Edge, e2: Edge) -> bool {
//         return e1.v1 == e2.v1 && e1.v2 == e2.v2;
//     }

//     equalish :: (t1: Triangle3, t2: Triangle3) -> bool {
//         // if we can find all 3 points somewhere on the other triangle we can assume the triangles are equal regardless of order
//         // its either this, or one of the triangles is a line
//         matches := true;
//         matches = matches && (t1.a == t2.a || t1.a == t2.b || t1.a == t2.c);
//         matches = matches && (t1.b == t2.a || t1.b == t2.b || t1.b == t2.c);
//         matches = matches && (t1.c == t2.a || t1.c == t2.b || t1.c == t2.c);
//         return matches;
//     }

//     edge_triangles: Table(Edge, EdgeTriangles, hash_function, compare_function);
//     edge_triangles.allocator = temp;

//     index := 0;
//     while index < indices.count {
//         defer index += 3;
        
//         v1 := vertices[indices[index]];
//         v2 := vertices[indices[index + 1]];
//         v3 := vertices[indices[index + 2]];

//         edges: [6] Edge;
//         edges[0] = .{v1, v2};
//         edges[1] = .{v2, v1};
//         edges[2] = .{v2, v3};
//         edges[3] = .{v3, v2};
//         edges[4] = .{v3, v1};
//         edges[5] = .{v1, v3};

//         triangle := create_triangle3(v1, v2, v3);

//         for edge: edges {
//             edge_info := find_or_add(*edge_triangles, edge);
//             for i: 0..edge_info.edge_triangle_count - 1 {
//                 // if the triangle is already accounted for check the next edge
//                 if equalish(triangle, edge_info.edge_triangles[i]) {
//                     continue edge;
//                 }
//             }

//             // if it gets to this point it hasn't found the triangle, so we'll add it
//             if edge_info.edge_triangle_count < 2 {
//                 edge_info.edge_triangles[edge_info.edge_triangle_count] = triangle;
//                 edge_info.edge_triangle_count += 1;
//                 continue edge;
//             } else {
//                 // if there are already 2 triangles we have a weird mesh
//                 print("Potential convex hull has an edge with 3+ triangles. %\n", edge);
//                 return false;
//             }
//         }
//     }

//     for edge_info: edge_triangles {
//         if edge_info.edge_triangle_count != 2 {
//             return false;
//         }
//     }

//     return true;
// }

#scope_export
