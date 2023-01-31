
solve_gjk :: (a: KodaVertices, b: KodaVertices, dir: Vector3) -> bool {
    simplex: KodaVertices(4);
    return solve_gjk(a, b, dir, *simplex);
}

solve_gjk :: (a: KodaVertices, b: KodaVertices, dir: Vector3, simplex: *KodaVertices(4)) -> bool {
    search_dir := dir;
    if search_dir == .{0, 0, 0} {
        search_dir = a[0] - b[0];
    }

    MAX_ITERATIONS :: 64;

    simplex.count = 4;
    simplex.fixed[2] = get_support(b, search_dir) - get_support(a, -search_dir);
    search_dir = -simplex.fixed[2];

    simplex.fixed[1] = get_support(b, search_dir) - get_support(a, -search_dir);

    if dot(simplex.fixed[1], search_dir) < 0 {
        return false;
    }

    search_dir = cross(cross(simplex.fixed[2] - simplex.fixed[1], -simplex.fixed[1]), simplex.fixed[2] - simplex.fixed[1]);
    if search_dir == .{0, 0, 0} {
        // the origin is on this line segment
        search_dir = cross(simplex.fixed[2] - simplex.fixed[1], .{1, 0, 0});
        if search_dir == .{0, 0, 0} {
            search_dir = cross(simplex.fixed[2] - simplex.fixed[1], .{0, 0, -1});
        }
    }

    simp_dim := 2;

    for i: 0..MAX_ITERATIONS - 1 {
        simplex.fixed[0] = get_support(b, search_dir) - get_support(a, -search_dir);
        if dot(simplex.fixed[0], search_dir) < 0 {
            return false;
        }

        simp_dim += 1;
        if simp_dim == 3 {
            simp_dim, search_dir = solve_simplex3(simplex, simp_dim, search_dir);
        } else {
            found: bool;
            found, simp_dim, search_dir = solve_simplex4(simplex, simp_dim, search_dir);
            if found {
                return true;
            }
        }
    }

    return false;
}

solve_gjk_open :: (a: KodaVertices, b: KodaVertices, dir: Vector3) -> bool {
    simplex: KodaVertices(4);
    return solve_gjk_open(a, b, dir, *simplex);
}

solve_gjk_open :: (a: KodaVertices, b: KodaVertices, dir: Vector3, simplex: *KodaVertices(4)) -> bool {
    search_dir := dir;
    if search_dir == .{0, 0, 0} {
        search_dir = a[0] - b[0];
    }

    MAX_ITERATIONS :: 64;

    simplex.count = 4;
    simplex.fixed[2] = get_support(b, search_dir) - get_support(a, -search_dir);
    search_dir = -simplex.fixed[2];

    simplex.fixed[1] = get_support(b, search_dir) - get_support(a, -search_dir);

    // if dot(simplex.fixed[1], search_dir) < 0 {
    //     return false;
    // }

    search_dir = cross(cross(simplex.fixed[2] - simplex.fixed[1], -simplex.fixed[1]), simplex.fixed[2] - simplex.fixed[1]);
    if search_dir == .{0, 0, 0} {
        // the origin is on this line segment
        search_dir = cross(simplex.fixed[2] - simplex.fixed[1], .{1, 0, 0});
        if search_dir == .{0, 0, 0} {
            search_dir = cross(simplex.fixed[2] - simplex.fixed[1], .{0, 0, -1});
        }
    }

    simp_dim := 2;

    for i: 0..MAX_ITERATIONS - 1 {
        simplex.fixed[0] = get_support(b, search_dir) - get_support(a, -search_dir);
        // if dot(simplex.fixed[0], search_dir) < 0 {
        //     return false;
        // }

        simp_dim += 1;
        if simp_dim == 3 {
            simp_dim, search_dir = solve_simplex3(simplex, simp_dim, search_dir);
        } else {
            found: bool;
            found, simp_dim, search_dir = solve_simplex4(simplex, simp_dim, search_dir);
            if found {
                return true;
            }
        }
    }

    return false;
}

#scope_file

solve_simplex3 :: (simplex: *KodaVertices(4), simp_dim: int, search_direction: Vector3) -> int, Vector3 {
    search_dir := search_direction;
    
    n := cross(simplex.fixed[1] - simplex.fixed[0], simplex.fixed[2] - simplex.fixed[0]);
    ao := -simplex.fixed[0];

    simp_dim = 2;
    if dot(cross(simplex.fixed[1] - simplex.fixed[0], n), ao) > 0 {
        simplex.fixed[2] = simplex.fixed[0];
        search_dir = cross(cross(simplex.fixed[1] - simplex.fixed[0], ao), simplex.fixed[1] - simplex.fixed[0]);
        return simp_dim, search_dir;
    }

    if dot(cross(n, simplex.fixed[2] - simplex.fixed[0]), ao) > 0 {
        simplex.fixed[1] = simplex.fixed[0];
        search_dir = cross(cross(simplex.fixed[2] - simplex.fixed[0], ao), simplex.fixed[2] - simplex.fixed[0]);
        return simp_dim, search_dir;
    }

    simp_dim = 3;
    if dot(n, ao) > 0 {
        simplex.fixed[3] = simplex.fixed[2];
        simplex.fixed[2] = simplex.fixed[1];
        simplex.fixed[1] = simplex.fixed[0];
        search_dir = n;
        return simp_dim, search_dir;
    }

    simplex.fixed[3] = simplex.fixed[1];
    simplex.fixed[1] = simplex.fixed[0];
    search_dir = -n;
    return simp_dim, search_dir;
}

solve_simplex4 :: (simplex: *KodaVertices(4), simp_dim: int, search_direction: Vector3) -> bool, int, Vector3 {
    search_dir := search_direction;

    abc := cross(simplex.fixed[1] - simplex.fixed[0], simplex.fixed[2] - simplex.fixed[0]);
    acd := cross(simplex.fixed[2] - simplex.fixed[0], simplex.fixed[3] - simplex.fixed[0]);
    adb := cross(simplex.fixed[3] - simplex.fixed[0], simplex.fixed[1] - simplex.fixed[0]);

    ao := -simplex.fixed[0];
    simp_dim = 3;

    if dot(abc, ao) > 0 {
        simplex.fixed[3] = simplex.fixed[2];
        simplex.fixed[2] = simplex.fixed[1];
        simplex.fixed[1] = simplex.fixed[0];
        search_dir = abc;
        return false, simp_dim, search_dir;
    }
    if dot(acd, ao) > 0 {
        simplex.fixed[1] = simplex.fixed[0];
        search_dir = acd;
        return false, simp_dim, search_dir;
    }
    if dot(adb, ao) > 0 {
        simplex.fixed[2] = simplex.fixed[3];
        simplex.fixed[3] = simplex.fixed[1];
        simplex.fixed[1] = simplex.fixed[0];
        search_dir = adb;
        return false, simp_dim, search_dir;
    }

    return true, simp_dim, search_dir;
}

get_support :: (vertices: KodaVertices, direction: Vector3) -> Vector3 {
    if vertices.count == 0 {
        return .{};
    }

    largest_dot := dot(vertices[0], direction);
    largest_vertex := vertices[0];
    for i: 1..vertices.count - 1 {
        vertex := vertices[i];

        current_dot := dot(vertex, direction);
        if current_dot > largest_dot {
            largest_dot = current_dot;
            largest_vertex = vertex;
        }
    }

    return largest_vertex;
}
