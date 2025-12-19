
KODA_COLLISION_ALL: s32 : 0xffffffff;

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

    sensor: bool = false;
    collision_group: s32 = KODA_COLLISION_ALL;

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

load_bodies :: (path: string, $fallback: () -> [] *KodaBody) -> [] *KodaBody {
    file, loaded := read_entire_file(path, log_errors = false);
    if !loaded {
        bodies := fallback();
        save_bodies(bodies, path);

        return bodies;
    }

    view: [] u8;
    view.data = file.data;
    view.count = file.count;

    bodies_bytes: [] u8;
    bodies_bytes.data = file.data + size_of(SerializedBodyHash);
    bodies_bytes.count = file.count - size_of(SerializedBodyHash);

    cache_hash := get_hash(bodies_bytes);
    cache_size := bodies_bytes.count;

    hash := deserialize_body_hash(view);
    if hash.hash == cache_hash && hash.size == cache_size {
        print("Skipping body file read. Reading serialized.\n");

        bodies: [..] *KodaBody;
        offset := 0;
        while offset < bodies_bytes.count {
            body_bytes_count: int;
            memcpy(*body_bytes_count, bodies_bytes.data + offset, size_of(int));

            offset += size_of(int);

            body_view: [] u8;
            body_view.data = bodies_bytes.data + offset;
            body_view.count = body_bytes_count;

            offset += body_bytes_count;

            body := deserialize_body(body_view);
            array_add(*bodies, body);
        }

        return bodies;
    }

    return .[];
}

save_bodies :: (bodies: [] *KodaBody, path: string) {
    bytes: [..] u8;
    defer array_free(bytes);

    array_reserve(*bytes, size_of(SerializedBodyHash));
    
    body_hash: SerializedBodyHash;
    memcpy(bytes.data, *body_hash, size_of(SerializedBodyHash));
    bytes.count += size_of(SerializedBodyHash);

    for body: bodies {
        body_bytes := serialize_body(body);
        defer array_free(body_bytes);

        body_bytes_count: int = body_bytes.count;
        array_reserve(*bytes, bytes.count + size_of(int) + body_bytes.count);
        memcpy(bytes.data + bytes.count, *body_bytes_count, size_of(int));
        bytes.count += size_of(int);
        memcpy(bytes.data + bytes.count, body_bytes.data, body_bytes_count);
        bytes.count += body_bytes_count;
    }

    bodies_bytes: [] u8;
    bodies_bytes.data = bytes.data + size_of(SerializedBodyHash);
    bodies_bytes.count = bytes.count - size_of(SerializedBodyHash);

    cache_hash := get_hash(bodies_bytes);
    cache_size := bodies_bytes.count;

    body_hash.hash = cache_hash;
    body_hash.size = cache_size;

    memcpy(bytes.data, *body_hash, size_of(SerializedBodyHash));

    write_entire_file(path, bytes.data, bytes.count);
}

load_body :: (path: string, $fallback: () -> *KodaBody) -> *KodaBody {
    file, loaded := read_entire_file(path, log_errors = false);
    if !loaded {
        body := fallback();
        save_body(body, path);

        return body;
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
    serializer_write(*bytes, body.type);

    array_ordered_remove_by_value(*member_names, "shapes");
    serializer_write(*bytes, body.shapes.count);
    for shape: body.shapes {
        serializer_write(*bytes, shape.type);
        serializer_write(*bytes, shape.aabb);
        if #complete shape.type == {
            case .ConvexHull;
                serializer_write(*bytes, shape.hull);
            case .TriangleMesh;
                serializer_write(*bytes, shape.mesh);
            case .Vertices;
                serializer_write(*bytes, shape.vertices);
        }
    }

    array_ordered_remove_by_value(*member_names, "aabb");
    serializer_write(*bytes, body.aabb);

    array_ordered_remove_by_value(*member_names, "position");
    serializer_write(*bytes, body.position);

    array_ordered_remove_by_value(*member_names, "velocity");
    serializer_write(*bytes, body.velocity);

    array_ordered_remove_by_value(*member_names, "rotation");
    serializer_write(*bytes, body.rotation);

    array_ordered_remove_by_value(*member_names, "scale");
    serializer_write(*bytes, body.scale);

    array_ordered_remove_by_value(*member_names, "gravity_strength");
    serializer_write(*bytes, body.gravity_strength);

    array_ordered_remove_by_value(*member_names, "dirty_position");
    serializer_write(*bytes, body.dirty_position);

    array_ordered_remove_by_value(*member_names, "dirty_rotation");
    serializer_write(*bytes, body.dirty_rotation);

    array_ordered_remove_by_value(*member_names, "dirty_scale");
    serializer_write(*bytes, body.dirty_scale);

    array_ordered_remove_by_value(*member_names, "collision_normals");
    serializer_write(*bytes, body.collision_normals);

    array_ordered_remove_by_value(*member_names, "ground_enabled");
    serializer_write(*bytes, body.ground_enabled);

    array_ordered_remove_by_value(*member_names, "step_height");
    serializer_write(*bytes, body.step_height);

    array_ordered_remove_by_value(*member_names, "sensor");
    serializer_write(*bytes, body.sensor);

    array_ordered_remove_by_value(*member_names, "collision_group");
    serializer_write(*bytes, body.collision_group);

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

    offset: int;

    array_ordered_remove_by_value(*member_names, "type");
    offset = serializer_read(bytes, *body.type, offset);

    array_ordered_remove_by_value(*member_names, "shapes");
    count: int;
    offset = serializer_read(bytes, *count, offset);
    array_reserve(*body.shapes, count);
    for i: 0..count - 1 {
        shape: KodaShape;
        offset = serializer_read(bytes, *shape.type, offset);
        offset = serializer_read(bytes, *shape.aabb, offset);
        if #complete shape.type == {
            case .ConvexHull;
                offset = serializer_read(bytes, *shape.hull, offset);
            case .TriangleMesh;
                offset = serializer_read(bytes, *shape.mesh, offset);
            case .Vertices;
                offset = serializer_read(bytes, *shape.vertices, offset);
        }
        array_add(*body.shapes, shape);
    }

    array_ordered_remove_by_value(*member_names, "aabb");
    offset = serializer_read(bytes, *body.aabb, offset);

    array_ordered_remove_by_value(*member_names, "position");
    offset = serializer_read(bytes, *body.position, offset);

    array_ordered_remove_by_value(*member_names, "velocity");
    offset = serializer_read(bytes, *body.velocity, offset);

    array_ordered_remove_by_value(*member_names, "rotation");
    offset = serializer_read(bytes, *body.rotation, offset);

    array_ordered_remove_by_value(*member_names, "scale");
    offset = serializer_read(bytes, *body.scale, offset);

    array_ordered_remove_by_value(*member_names, "gravity_strength");
    offset = serializer_read(bytes, *body.gravity_strength, offset);

    array_ordered_remove_by_value(*member_names, "dirty_position");
    offset = serializer_read(bytes, *body.dirty_position, offset);

    array_ordered_remove_by_value(*member_names, "dirty_rotation");
    offset = serializer_read(bytes, *body.dirty_rotation, offset);

    array_ordered_remove_by_value(*member_names, "dirty_scale");
    offset = serializer_read(bytes, *body.dirty_scale, offset);

    array_ordered_remove_by_value(*member_names, "collision_normals");
    offset = serializer_read(bytes, *body.collision_normals, offset);

    array_ordered_remove_by_value(*member_names, "ground_enabled");
    offset = serializer_read(bytes, *body.ground_enabled, offset);

    array_ordered_remove_by_value(*member_names, "step_height");
    offset = serializer_read(bytes, *body.step_height, offset);

    array_ordered_remove_by_value(*member_names, "sensor");
    offset = serializer_read(bytes, *body.sensor, offset);

    array_ordered_remove_by_value(*member_names, "collision_group");
    offset = serializer_read(bytes, *body.collision_group, offset);

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

get_transform :: (body: *KodaBody) -> Matrix4 {
    matrix := Matrix4Identity;
    matrix = translate(matrix, body.position);
    matrix = rotate(matrix, body.rotation);
    matrix = scale(matrix, body.scale);
    return matrix;
}

get_world_aabb :: (body: KodaBody) -> AABB3 {
    matrix := get_transform(body);

    return transform_aabb(matrix, body.aabb);
}

get_world_vertices :: (body: KodaBody, shape_index: int) -> KodaVertices(MAX_VERTICES) {
    return get_world_vertices(body, body.shapes[shape_index]);
}

get_world_vertices :: (body: KodaBody, shape: KodaShape) -> KodaVertices(MAX_VERTICES) {
    if #complete shape.type == {
        case .ConvexHull;
            matrix := get_transform(body);
            vertices: KodaVertices(MAX_VERTICES);
            vertices.count = shape.hull.vertices.count;
            for vertex, index: shape.hull.vertices {
                vertices[i] = transform_point(matrix, vertex);
            }
            return vertices;

        case .TriangleMesh;
            assert(false, "Cannot get world vertices of a triangle mesh. It's too expensive.");

        case .Vertices;
            matrix := get_transform(body);
            vertices: KodaVertices(MAX_VERTICES);
            vertices.count = shape.vertices.count;
            for vertex, index: shape.vertices {
                vertices[i] = transform_point(matrix, vertex);
            }
            return vertices;
    }
}

#scope_export
