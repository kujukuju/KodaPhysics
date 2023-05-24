KodaBody :: struct {
    type: KodaBodyType;
    shapes: [..] KodaShape;
    aabb: AABB3;

    position: Vector3;
    velocity: Vector3;
    rotation: Quaternion;
    scale: Vector3 = .{1, 1, 1};
    gravity_strength: float = 1.0;

    dirty_position: Vector3;
    // dirty_velocity: Vector3;
    dirty_rotation: Quaternion;
    dirty_scale: Vector3 = .{1, 1, 1};

    collision_normals: [..] Vector3;
    ground_enabled: bool = true;
    step_height: float = 0.4;

    proxy: int = -1;
}

KodaBodyType :: enum {
    Static;
    Dynamic;
}

SerializedBodyHash :: struct {
    hash: u32;
    size: s64;
}

SerializedKodaBody :: struct {
    validity_hash: SerializedBodyHash;
    body: KodaBody;
}

create_body :: (shape: KodaShape, type: KodaBodyType) -> *KodaBody {
    shapes: [1] KodaShape;
    shapes[0] = shape;

    return create_body(shapes, type);
}

create_body :: (shapes: [] KodaShape, type: KodaBodyType) -> *KodaBody {
    assert(physics_initialized, "Your physics world must be initialized before you create bodies.");

    body := claim(*allocated_bodies);
    body.type = type;
    body.aabb = shapes[0].aabb;
    for shape: shapes {
        array_add(*body.shapes, shape);
        body.aabb = merge(body.aabb, shape.aabb);
    }

    body.proxy = create_proxy(*tree_bodies, get_world_aabb(body), body);

    return body;
}

load_body :: (path: string) -> *KodaBody {
    file, loaded := read_entire_file(path, log_errors = false);
    if !loaded {
        return null;
    }

    view: [] u8;
    view.data = file.data;
    view.count = file.count;

    body_bytes: [] u8;
    body_bytes.data = file.data + size_of(SerializedBodyHash);
    body_bytes.count = file.count - size_of(SerializedBodyHash);

    cache_hash := get_hash(body_bytes);
    cache_size := body_bytes.count;

    hash := deserialize_body_hash(view);
    if hash.hash == cache_hash && hash.size == cache_size {
        print("Skipping body file read. Reading serialized.\n");

        body := deserialize_body(body_bytes);

        return body;
    }

    return null;
}

save_body :: (body: KodaBody, path: string) {
    serialized: SerializedKodaBody;
    serialized.body = body;
    
    bytes: [..] u8;
    defer array_free(bytes);

    body_bytes := serialize_body(body);
    defer array_free(body_bytes);

    array_resize(*bytes, body_bytes.count + size_of(SerializedBodyHash));
    memcpy(bytes.data + size_of(SerializedBodyHash), body_bytes.data, body_bytes.count);

    cache_hash := get_hash(body_bytes);
    cache_size := body_bytes.count;

    body_hash: SerializedBodyHash;
    body_hash.hash = cache_hash;
    body_hash.size = cache_size;

    memcpy(bytes.data, *body_hash, size_of(SerializedBodyHash));

    write_entire_file(path, bytes.data, bytes.count);
}

serialize_body :: (body: KodaBody) -> [..] u8 {
    bytes: [..] u8;

    // get all body member variables and if switch throw between them to serialize individually
    body_type := cast(*Type_Info_Struct) type_info(KodaBody);
    members := body_type.members;
    member_names: [..] string;
    defer array_free(member_names);
    for member: members {
        array_add(*member_names, member.name);
    }

    // write all members and remove from the list

    array_ordered_remove_by_value(*member_names, "type");
    serializer.write(*bytes, body.type);

    array_ordered_remove_by_value(*member_names, "shapes");
    serializer.write(*bytes, body.shapes.count);
    for shape: body.shapes {
        serializer.write(*bytes, shape.type);
        serializer.write(*bytes, shape.aabb);
        if #complete shape.type == {
            case .ConvexHull;
                serializer.write(*bytes, shape.hull);
            case .TriangleMesh;
                serializer.write(*bytes, shape.mesh);
        }
    }

    array_ordered_remove_by_value(*member_names, "aabb");
    serializer.write(*bytes, body.aabb);

    array_ordered_remove_by_value(*member_names, "position");
    serializer.write(*bytes, body.position);

    array_ordered_remove_by_value(*member_names, "velocity");
    serializer.write(*bytes, body.velocity);

    array_ordered_remove_by_value(*member_names, "rotation");
    serializer.write(*bytes, body.rotation);

    array_ordered_remove_by_value(*member_names, "scale");
    serializer.write(*bytes, body.scale);

    array_ordered_remove_by_value(*member_names, "gravity_strength");
    serializer.write(*bytes, body.gravity_strength);

    array_ordered_remove_by_value(*member_names, "dirty_position");
    serializer.write(*bytes, body.dirty_position);

    array_ordered_remove_by_value(*member_names, "dirty_rotation");
    serializer.write(*bytes, body.dirty_rotation);

    array_ordered_remove_by_value(*member_names, "dirty_scale");
    serializer.write(*bytes, body.dirty_scale);

    array_ordered_remove_by_value(*member_names, "collision_normals");
    serializer.write(*bytes, body.collision_normals);

    array_ordered_remove_by_value(*member_names, "ground_enabled");
    serializer.write(*bytes, body.ground_enabled);

    array_ordered_remove_by_value(*member_names, "step_height");
    serializer.write(*bytes, body.step_height);

    array_ordered_remove_by_value(*member_names, "proxy");
    // nothing

    if member_names.count > 0 {
        print("Remaining members: %\n", member_names);
    }
    assert(member_names.count == 0, "Finished serializing the body but didn't get through all members.");

    return bytes;
}

deserialize_body :: (bytes: [] u8) -> *KodaBody {
    body := claim(*allocated_bodies);

    body_type := cast(*Type_Info_Struct) type_info(KodaBody);
    members := body_type.members;
    member_names: [..] string;
    defer array_free(member_names);
    for member: members {
        array_add(*member_names, member.name);
    }

    index: int;

    array_ordered_remove_by_value(*member_names, "type");
    serializer.read(bytes, *index, *body.type);

    array_ordered_remove_by_value(*member_names, "shapes");
    count: int;
    serializer.read(bytes, *index, *count);
    array_reserve(*body.shapes, count);
    for i: 0..count - 1 {
        shape: KodaShape;
        serializer.read(bytes, *index, *shape.type);
        serializer.read(bytes, *index, *shape.aabb);
        if #complete shape.type == {
            case .ConvexHull;
                serializer.read(bytes, *index, *shape.hull);
            case .TriangleMesh;
                serializer.read(bytes, *index, *shape.mesh);
        }
        array_add(*body.shapes, shape);
    }

    array_ordered_remove_by_value(*member_names, "aabb");
    serializer.read(bytes, *index, *body.aabb);

    array_ordered_remove_by_value(*member_names, "position");
    serializer.read(bytes, *index, *body.position);

    array_ordered_remove_by_value(*member_names, "velocity");
    serializer.read(bytes, *index, *body.velocity);

    array_ordered_remove_by_value(*member_names, "rotation");
    serializer.read(bytes, *index, *body.rotation);

    array_ordered_remove_by_value(*member_names, "scale");
    serializer.read(bytes, *index, *body.scale);

    array_ordered_remove_by_value(*member_names, "gravity_strength");
    serializer.read(bytes, *index, *body.gravity_strength);

    array_ordered_remove_by_value(*member_names, "dirty_position");
    serializer.read(bytes, *index, *body.dirty_position);

    array_ordered_remove_by_value(*member_names, "dirty_rotation");
    serializer.read(bytes, *index, *body.dirty_rotation);

    array_ordered_remove_by_value(*member_names, "dirty_scale");
    serializer.read(bytes, *index, *body.dirty_scale);

    array_ordered_remove_by_value(*member_names, "collision_normals");
    serializer.read(bytes, *index, *body.collision_normals);

    array_ordered_remove_by_value(*member_names, "ground_enabled");
    serializer.read(bytes, *index, *body.ground_enabled);

    array_ordered_remove_by_value(*member_names, "step_height");
    serializer.read(bytes, *index, *body.step_height);

    array_ordered_remove_by_value(*member_names, "proxy");
    body.proxy = create_proxy(*tree_bodies, get_world_aabb(body), body);

    if member_names.count > 0 {
        print("Remaining members: %\n", member_names);
    }
    assert(member_names.count == 0, "Finished deserializing the body but didn't get through all members.");

    return body;
}

deserialize_body_hash :: (bytes: [] u8) -> SerializedBodyHash {
    hash: SerializedBodyHash;
    memcpy(*hash, bytes.data, size_of(SerializedBodyHash));

    return hash;
}

is_dirty :: inline (body: KodaBody) -> bool {
    dirty := false;
    dirty = dirty || body.position != body.dirty_position;
    dirty = dirty || body.rotation != body.dirty_rotation;
    dirty = dirty || body.scale != body.dirty_scale;
    return dirty;
}

clear_dirty :: inline (body: *KodaBody) {
    body.dirty_position = body.position;
    body.dirty_rotation = body.rotation;
    body.dirty_scale = body.scale;
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

    return transform_aabb(matrix, body.aabb);

    // aabb_points: [8] Vector3;
    // aabb_points[0] = Vector3.{body.aabb.lower.x, body.aabb.lower.y, body.aabb.lower.z};
    // aabb_points[1] = Vector3.{body.aabb.lower.x, body.aabb.lower.y, body.aabb.upper.z};
    // aabb_points[2] = Vector3.{body.aabb.lower.x, body.aabb.upper.y, body.aabb.lower.z};
    // aabb_points[3] = Vector3.{body.aabb.lower.x, body.aabb.upper.y, body.aabb.upper.z};
    // aabb_points[4] = Vector3.{body.aabb.upper.x, body.aabb.lower.y, body.aabb.lower.z};
    // aabb_points[5] = Vector3.{body.aabb.upper.x, body.aabb.lower.y, body.aabb.upper.z};
    // aabb_points[6] = Vector3.{body.aabb.upper.x, body.aabb.upper.y, body.aabb.lower.z};
    // aabb_points[7] = Vector3.{body.aabb.upper.x, body.aabb.upper.y, body.aabb.upper.z};

    // first: Vector4 = ---;
    // first.xyz = aabb_points[0];
    // first.w = 1.0;
    // first = matrix * first;

    // aabb: AABB3 = ---;
    // aabb.lower = first.xyz;
    // aabb.upper = first.xyz;

    // for i: 1..aabb_points.count - 1 {
    //     point: Vector4 = ---;
    //     point.xyz = aabb_points[i];
    //     point.w = 1.0;
    //     point = matrix * point;

    //     aabb.lower = min(aabb.lower, point.xyz);
    //     aabb.upper = max(aabb.upper, point.xyz);
    // }

    // return aabb;
}

// TODO I should be able to do all these modifications in the support function, this isnt necessary
get_world_vertices :: (body: KodaBody, shape_index: int) -> KodaVertices(MAX_VERTICES) {
    return get_world_vertices(body, body.shapes[shape_index]);
}

get_world_vertices :: (body: KodaBody, shape: KodaShape) -> KodaVertices(MAX_VERTICES) {
    // TODO should not do this, just offset in the support function
    assert(shape.type == .ConvexHull, "Cannot get world vertices of a triangle mesh. It's too expensive.");

    matrix := get_transform(body);
    vertices := shape.hull.vertices;
    for i: 0..vertices.count - 1 {
        vertex: Vector4 = ---;
        vertex.xyz = vertices[i];
        vertex.w = 1.0;

        vertex = matrix * vertex;
        vertices[i] = vertex.xyz;
    }

    return vertices;
}

#scope_export
