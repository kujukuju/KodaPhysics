SimplexTracker :: struct {
    table: Table(SimplexKey, SimplexEntry, hash_function, compare_function);
}

// TODO I could make this whole thing faster if every "shape" has its own incrementing id so I can just ignore body pointer

SimplexKey :: struct {
    a: u64;
    b: u64;
    a_shape: u64;
    b_shape: u64;
    a_entry: u64;
    b_entry: u64;
    hash: u32;
}

SimplexEntry :: struct {
    simplex: Simplex;
    active: bool;
}

clear_active_simplexes :: (tracker: *SimplexTracker) {
    for *entry: tracker.table {
        entry.active = false;
    }
}

remove_inactive_simplexes :: (tracker: *SimplexTracker) {
    for *entry: tracker.table {
        if !entry.active {
            remove entry;
        }
    }
}

add_cached_simplex :: (tracker: *SimplexTracker, key: SimplexKey, simplex: Simplex) {
    entry := find_or_add(*tracker.table, key);
    entry.simplex = simplex;
    entry.active = true;
}

get_cached_simplex :: (tracker: *SimplexTracker, key: SimplexKey) -> *Simplex {
    entry := table_find_pointer(*tracker.table, key);
    return *entry.simplex;
}

get_simplex_key :: inline (a: u64, b: u64, a_shape: u64, b_shape: u64, a_entry: u64, b_entry: u64) -> SimplexKey {
    key: SimplexKey = ---;
    key.a = a;
    key.b = b;
    key.a_shape = a_shape;
    key.b_shape = b_shape;
    key.a_entry = a_entry;
    key.b_entry = b_entry;
    key.hash = 0;

    parts: [] u32;
    parts.data = xx *key.a;
    parts.count = 12;

    for part: parts {
        key.hash ^= part;
    }

    return key;
}

#scope_file

hash_function :: (key: SimplexKey) -> u32 {
    return key.hash;
}

compare_function :: (k1: SimplexKey, k2: SimplexKey) -> bool {
    return k1.a == k2.a && k1.b == k2.b && k1.a_shape == k2.a_shape && k1.b_shape == k2.b_shape && k1.a_entry == k2.a_entry && k1.b_entry == k2.b_entry;
}

#scope_export
