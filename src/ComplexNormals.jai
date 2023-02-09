
KodaComplexNormals :: struct (N: int = MAX_VERTICES) {
    count: int;
    fixed: [N] Vector4;
}

operator [] :: (complex_normals: KodaComplexNormals, i: int) -> Vector4 {
    assert(i < complex_normals.count, "Out of bounds normal access.");

    return complex_normals.fixed[i];
}

operator *[] :: (complex_normals: *KodaComplexNormals, i: int) -> *Vector4 {
    assert(i < complex_normals.count, "Out of bounds normal access.");

    return *complex_normals.fixed[i];
}

add :: (complex_normals: *KodaComplexNormals($N), v: Vector4) {
    if complex_normals.count < N {
        complex_normals.fixed[complex_normals.count] = v;
        complex_normals.count += 1;
    }
}

add :: (complex_normals: *KodaComplexNormals($N), v1: Vector4, v2: Vector4, v3: Vector4) {
    if complex_normals.count < N - 2 {
        complex_normals.fixed[complex_normals.count] = v1;
        complex_normals.fixed[complex_normals.count + 1] = v2;
        complex_normals.fixed[complex_normals.count + 2] = v3;
        complex_normals.count += 3;
    }
}

ordered_remove :: (complex_normals: *KodaComplexNormals($N), index: int) {
    assert(index < complex_normals.count, "Out of bounds normal access.");

    for i: index..complex_normals.count - 2 {
        complex_normals.fixed[i] = complex_normals.fixed[i + 1];
    }
    complex_normals.count -= 1;
}

get_view :: (complex_normals: *KodaComplexNormals) -> [] Vector4 {
    view: [] Vector3;
    view.data = complex_normals.fixed.data;
    view.count = complex_normals.count;
    return view;
}

#scope_export
