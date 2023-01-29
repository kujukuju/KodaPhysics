
KodaIndices :: struct (N: int = MAX_VERTICES) {
    count: int;
    fixed: [N] s32;
}

operator [] :: (indices: KodaIndices, index: int) -> s32 {
    assert(index < indices.count, "Out of bounds index access.");

    return indices.fixed[index];
}

operator *[] :: (indices: *KodaIndices, index: int) -> *s32 {
    assert(index < indices.count, "Out of bounds index access.");

    return *indices.fixed[index];
}

add :: (indices: *KodaIndices($N), i: s32) {
    if indices.count < N {
        indices.fixed[indices.count] = i;
        indices.count += 1;
    }
}

add :: (indices: *KodaIndices($N), i1: s32, i2: s32) {
    if indices.count < N - 1 {
        indices.fixed[indices.count] = i1;
        indices.fixed[indices.count + 1] = i2;
        indices.count += 2;
    }
}

add :: (indices: *KodaIndices($N), i1: s32, i2: s32, i3: s32) {
    if indices.count < N - 2 {
        indices.fixed[indices.count] = i1;
        indices.fixed[indices.count + 1] = i2;
        indices.fixed[indices.count + 2] = i3;
        indices.count += 3;
    }
}

ordered_remove :: (indices: *KodaIndices($N), index: int) {
    assert(index < indices.count, "Out of bounds index access.");
    
    for i: index..indices.count - 2 {
        indices.fixed[i] = indices.fixed[i + 1];
    }
    indices.count -= 1;
}

get_view :: (indices: *KodaIndices) -> [] s32 {
    view: [] s32;
    view.data = indices.fixed.data;
    view.count = indices.count;
    return view;
}
