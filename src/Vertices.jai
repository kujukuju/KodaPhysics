
KodaVertices :: struct (N: int = MAX_VERTICES) {
    count: int;
    fixed: [N] Vector3;
}

operator [] :: (vertices: KodaVertices, i: int) -> Vector3 {
    assert(i < vertices.count, "Out of bounds vertex access.");

    return vertices.fixed[i];
}

operator *[] :: (vertices: *KodaVertices, i: int) -> *Vector3 {
    assert(i < vertices.count, "Out of bounds vertex access.");

    return *vertices.fixed[i];
}

add :: (vertices: *KodaVertices($N), v: Vector3) {
    if vertices.count < N {
        vertices.fixed[vertices.count] = v;
        vertices.count += 1;
    }
}

add :: (vertices: *KodaVertices($N), v1: Vector3, v2: Vector3, v3: Vector3) {
    if vertices.count < N - 2 {
        vertices.fixed[vertices.count] = v1;
        vertices.fixed[vertices.count + 1] = v2;
        vertices.fixed[vertices.count + 2] = v3;
        vertices.count += 3;
    }
}

ordered_remove :: (vertices: *KodaVertices($N), index: int) {
    assert(index < vertices.count, "Out of bounds vertex access.");

    for i: index..vertices.count - 2 {
        vertices.fixed[i] = vertices.fixed[i + 1];
    }
    vertices.count -= 1;
}

get_view :: (vertices: *KodaVertices) -> [] Vector3 {
    view: [] Vector3;
    view.data = vertices.fixed.data;
    view.count = vertices.count;
    return view;
}

MAX_VERTICES :: 1024;

#scope_export
