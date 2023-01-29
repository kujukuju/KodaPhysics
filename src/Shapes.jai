KodaShape :: struct {
    type: KodaShapeType;
    vertices: KodaVertices;
    aabb: AABB3;
}

KodaShapeType :: enum {
    ConvexHull;
    // TriangleMesh;
    // Box;
    // Sphere;
}

create_convex_hull :: (vertices: [] Vector3) -> KodaShape {
    assert(vertices.count <= MAX_VERTICES, "You cannot create a polygon with more than MAX_VERTICES.");
    shape: KodaShape;
    shape.type = .ConvexHull;
    shape.vertices.count = vertices.count;

    memcpy(shape.vertices.fixed.data, vertices.data, vertices.count * size_of(Vector3));

    shape.aabb = .{};
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
