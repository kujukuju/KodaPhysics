KodaShape :: struct {
    type: ShapeType;
    aabb: AABB3;

    union {
        hull: HullShape;
        mesh: MeshShape;
    }
}

ShapeType :: enum {
    ConvexHull;
    TriangleMesh;
    // Box;
    // Sphere;
}

HullShape :: struct {
    vertices: KodaVertices;
    indices: KodaIndices;
}

MeshShape :: struct {
    triangles: BoundingTree(Triangle3);
}

create_convex_hull :: (vertices: [] Vector3, indices: [] int) -> KodaShape {
    assert(vertices.count <= MAX_VERTICES, "You cannot create a polygon with more than MAX_VERTICES.");
    shape: KodaShape;
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

create_triangle_mesh :: (vertices: [] Vector3, indices: [] int) -> KodaShape {
    assert(vertices.count <= MAX_VERTICES, "You cannot create a polygon with more than MAX_VERTICES.");
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

create_triangle_mesh :: (vertices: [] Vector3) -> KodaShape {
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

#scope_file
