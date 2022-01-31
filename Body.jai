KodaBody :: struct (ShapeType: Type) {
    shape: ShapeType;
    type: KodaBodyType;
    id: int;

    position: Vector2;
    velocity: Vector2;
    rotation: float;

    dirty_position: Vector2;
    dirty_rotation: float;
    aabb: AABB;

    collision_normals: [..] Vector2;

    tree_proxy: int = -1;
}

KodaBodyType :: enum {
    Static;
    Kinematic;
}

is_dirty :: inline (body: KodaBody($T)) -> bool {
    return body.position != body.dirty_position || body.rotation != body.dirty_rotation;
}

clear_dirty :: inline (body: *KodaBody($T)) {
    body.dirty_position = body.position;
    body.dirty_rotation = body.rotation;   
}

set_position :: (body: *KodaBody($T), position: Vector2) {
    body.position = position;
}

set_rotation :: (body: *KodaBody($T), rotation: float) {
    body.rotation = rotation;
}

calculate_world_aabb :: (body: KodaBody(KodaPolygon)) -> AABB {
    aabb: AABB = ---;
    aabb.lower = body.position;
    aabb.upper = body.position;

    for i: 0..body.shape.count - 1 {
        point := rotate(body.shape.vertices[i], body.rotation);

        aabb.lower.x = min(aabb.lower.x, point.x);
        aabb.lower.y = min(aabb.lower.y, point.y);
        aabb.upper.x = max(aabb.upper.x, point.x);
        aabb.upper.y = max(aabb.upper.y, point.y);
    }

    return aabb;
}

calculate_world_aabb :: (body: KodaBody(KodaRectangle)) -> AABB {
    aabb: AABB = ---;
    aabb.lower = body.position;
    aabb.upper = body.position;

    vertices: [4] Vector2;
    vertices[0] = rotate(body.position + body.shape.dimensions / 2.0 * .{1, 1}, body.rotation);
    vertices[1] = rotate(body.position + body.shape.dimensions / 2.0 * .{-1, 1}, body.rotation);
    vertices[2] = rotate(body.position + body.shape.dimensions / 2.0 * .{-1, -1}, body.rotation);
    vertices[3] = rotate(body.position + body.shape.dimensions / 2.0 * .{1, -1}, body.rotation);

    for point: vertices {
        aabb.lower.x = min(aabb.lower.x, point.x);
        aabb.lower.y = min(aabb.lower.y, point.y);
        aabb.upper.x = max(aabb.upper.x, point.x);
        aabb.upper.y = max(aabb.upper.y, point.y);
    }

    return aabb;
}

get_world_vertices :: (body: KodaBody(KodaRectangle)) -> BodyVertices {
    dimensions := body.shape.dimensions;

    vertices: BodyVertices;
    vertices.count = 4;
    vertices.vertices[0] = rotate(xy(-dimensions.x / 2, -dimensions.y / 2), body.rotation) + body.position;
    vertices.vertices[1] = rotate(xy(-dimensions.x / 2, dimensions.y / 2), body.rotation) + body.position;
    vertices.vertices[2] = rotate(xy(dimensions.x / 2, dimensions.y / 2), body.rotation) + body.position;
    vertices.vertices[3] = rotate(xy(dimensions.x / 2, -dimensions.y / 2), body.rotation) + body.position;

    return vertices;
}

get_world_vertices :: (body: KodaBody(KodaPolygon)) -> BodyVertices {
    vertices: BodyVertices;

    vertices.count = body.shape.count;
    for i: 0..body.shape.count - 1 {
        vertices.vertices[i] = rotate(body.shape.vertices[i], body.rotation) + body.position;
    }

    return vertices;
}