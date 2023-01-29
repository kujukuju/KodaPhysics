KodaBody :: struct {
    type: KodaBodyType;
    shape: KodaShape;
    id: int;

    position: Vector3;
    velocity: Vector3;
    rotation: Quaternion;
    scale: Vector3 = .{1, 1, 1};
    // aabb: AABB3;

    dirty_position: Vector3;
    dirty_velocity: Vector3;
    dirty_rotation: Quaternion;
    dirty_scale: Vector3;

    collision_normals: [..] Vector3;

    tree_proxy: int = -1;
}

KodaBodyType :: enum {
    Static;
    Kinematic;
}

create_body :: (shape: KodaShape, type: KodaBodyType = .Static) -> KodaBody {
    body: KodaBody;
    body.type = type;
    body.shape = shape;
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
    matrix := Matrix4_Identity;
    matrix = translate(matrix, body.position);
    matrix = rotate(matrix, body.rotation);
    matrix = scale(matrix, body.scale);
    return matrix;
}

get_world_aabb :: (body: KodaBody) -> AABB3 {
    matrix := get_transform(body);

    shape_aabb_points: [8] Vector3;
    shape_aabb_points[0] = Vector3.{body.shape.aabb.lower.x, body.shape.aabb.lower.y, body.shape.aabb.lower.z};
    shape_aabb_points[1] = Vector3.{body.shape.aabb.lower.x, body.shape.aabb.lower.y, body.shape.aabb.upper.z};
    shape_aabb_points[2] = Vector3.{body.shape.aabb.lower.x, body.shape.aabb.upper.y, body.shape.aabb.lower.z};
    shape_aabb_points[3] = Vector3.{body.shape.aabb.lower.x, body.shape.aabb.upper.y, body.shape.aabb.upper.z};
    shape_aabb_points[4] = Vector3.{body.shape.aabb.upper.x, body.shape.aabb.lower.y, body.shape.aabb.lower.z};
    shape_aabb_points[5] = Vector3.{body.shape.aabb.upper.x, body.shape.aabb.lower.y, body.shape.aabb.upper.z};
    shape_aabb_points[6] = Vector3.{body.shape.aabb.upper.x, body.shape.aabb.upper.y, body.shape.aabb.lower.z};
    shape_aabb_points[7] = Vector3.{body.shape.aabb.upper.x, body.shape.aabb.upper.y, body.shape.aabb.upper.z};

    first: Vector4 = ---;
    first.xyz = shape_aabb_points[0];
    first.w = 1.0;
    first = matrix * first;

    aabb: AABB3 = ---;
    aabb.lower = first.xyz;
    aabb.upper = first.xyz;

    for i: 1..shape_aabb_points.count - 1 {
        point: Vector4 = ---;
        point.xyz = shape_aabb_points[i];
        point.w = 1.0;
        point = matrix * point;

        aabb.lower = min(aabb.lower, point.xyz);
        aabb.upper = max(aabb.upper, point.xyz);
    }

    return aabb;
}

get_world_vertices :: (body: KodaBody) -> KodaVertices(MAX_VERTICES) {
    matrix := get_transform(body);
    vertices := body.shape.vertices;

    for i: 0..vertices.count - 1 {
        vertex: Vector4 = ---;
        vertex.xyz = vertices[i];
        vertex.w = 1.0;

        vertex = matrix * vertex;
        vertices[i] = vertex.xyz;
    }

    return vertices;
}
