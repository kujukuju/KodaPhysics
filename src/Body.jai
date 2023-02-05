KodaBody :: struct {
    type: KodaBodyType;
    shapes: [..] KodaShape;
    aabb: AABB3;

    position: Vector3;
    velocity: Vector3;
    rotation: Quaternion;
    scale: Vector3 = .{1, 1, 1};

    dirty_position: Vector3;
    dirty_velocity: Vector3;
    dirty_rotation: Quaternion;
    dirty_scale: Vector3 = .{1, 1, 1};

    collision_normals: [..] Vector3;

    proxy: int = -1;
}

KodaBodyType :: enum {
    Static;
    Dynamic;
}

create_body :: (shapes: ..KodaShape, type: KodaBodyType) -> *KodaBody {
    body := claim(*allocated_bodies);
    body.type = type;
    body.aabb = shapes[0].aabb;
    for shape: shapes {
        array_add(*body.shapes, shape);
        body.aabb = merge(body.aabb, shape.aabb);
    }

    body.proxy = create_proxy(tree_bodies, get_world_aabb(body), body);

    return body;
}

is_dirty :: inline (body: KodaBody) -> bool {
    return body.position != body.dirty_position || body.rotation != body.dirty_rotation;
}

clear_dirty :: inline (body: *KodaBody) {
    body.dirty_position = body.position;
    body.dirty_velocity = body.velocity;
    body.dirty_rotation = body.rotation;
}

set_position :: (body: *KodaBody, position: Vector3) {
    body.position = position;
}

set_velocity :: (body: *KodaBody, velocity: Vector3) {
    body.velocity = velocity;
}

set_rotation :: (body: *KodaBody, rotation: Quaternion) {
    body.rotation = rotation;
}

get_transform :: (body: KodaBody) -> Matrix4 {
    matrix := Matrix4Identity;
    matrix = translate(matrix, body.position);
    matrix = rotate(matrix, body.rotation);
    matrix = scale(matrix, body.scale);
    return matrix;
}

get_world_aabb :: (body: KodaBody) -> AABB3 {
    matrix := get_transform(body);

    aabb_points: [8] Vector3;
    aabb_points[0] = Vector3.{body.aabb.lower.x, body.aabb.lower.y, body.aabb.lower.z};
    aabb_points[1] = Vector3.{body.aabb.lower.x, body.aabb.lower.y, body.aabb.upper.z};
    aabb_points[2] = Vector3.{body.aabb.lower.x, body.aabb.upper.y, body.aabb.lower.z};
    aabb_points[3] = Vector3.{body.aabb.lower.x, body.aabb.upper.y, body.aabb.upper.z};
    aabb_points[4] = Vector3.{body.aabb.upper.x, body.aabb.lower.y, body.aabb.lower.z};
    aabb_points[5] = Vector3.{body.aabb.upper.x, body.aabb.lower.y, body.aabb.upper.z};
    aabb_points[6] = Vector3.{body.aabb.upper.x, body.aabb.upper.y, body.aabb.lower.z};
    aabb_points[7] = Vector3.{body.aabb.upper.x, body.aabb.upper.y, body.aabb.upper.z};

    first: Vector4 = ---;
    first.xyz = aabb_points[0];
    first.w = 1.0;
    first = matrix * first;

    aabb: AABB3 = ---;
    aabb.lower = first.xyz;
    aabb.upper = first.xyz;

    for i: 1..aabb_points.count - 1 {
        point: Vector4 = ---;
        point.xyz = aabb_points[i];
        point.w = 1.0;
        point = matrix * point;

        aabb.lower = min(aabb.lower, point.xyz);
        aabb.upper = max(aabb.upper, point.xyz);
    }

    return aabb;
}

// TODO I should be able to do all these modifications in the support function, this isnt necessary
get_world_vertices :: (body: KodaBody, shape_index: int) -> KodaVertices(MAX_VERTICES) {
    return get_world_vertices(body, body.shapes[shape_index]);
}

get_world_vertices :: (body: KodaBody, shape: KodaShape) -> KodaVertices(MAX_VERTICES) {
    // TODO should not do this, just offset in the support function
    assert(shape.type == .ConvexHull, "Cannot get world vertices of a triangle mesh. It's too expensive.");

    matrix := get_transform(body);
    vertices := shape.vertices;
    for i: 0..vertices.count - 1 {
        vertex: Vector4 = ---;
        vertex.xyz = vertices[i];
        vertex.w = 1.0;

        vertex = matrix * vertex;
        vertices[i] = vertex.xyz;
    }

    return vertices;
}
