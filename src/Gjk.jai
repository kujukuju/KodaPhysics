
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
    simplex.c = get_support(b, search_dir) - get_support(a, -search_dir);
    search_dir = -simplex.c;

    simplex.b = get_support(b, search_dir) - get_support(a, -search_dir);

    if dot(simplex.b, search_dir) < 0 {
        return false;
    }

    search_dir = cross(cross(simplex.c - simplex.b, -simplex.b), simplex.c - simplex.b);
    if search_dir == .{0, 0, 0} {
        // the origin is on this line segment
        search_dir = cross(simplex.c - simplex.b, .{1, 0, 0});
        if search_dir == .{0, 0, 0} {
            search_dir = cross(simplex.c - simplex.b, .{0, 0, -1});
        }
    }

    simp_dim := 2;

    for i: 0..MAX_ITERATIONS - 1 {
        simplex.a = get_support(b, search_dir) - get_support(a, -search_dir);
        if dot(simplex.a, search_dir) < 0 {
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

solve_gjk_open :: (a: KodaVertices, b: KodaVertices) -> bool, Simplex {
    simplex: Simplex;
    search_dir := a[0] - b[0];

    MAX_ITERATIONS :: 64;

    simplex.c1 = get_support(a, -search_dir);
    simplex.c2 = get_support(b, search_dir);
    simplex.c = simplex.c2 - simplex.c1;
    search_dir = -simplex.c;

    simplex.b1 = get_support(a, -search_dir);
    simplex.b2 = get_support(b, search_dir);
    simplex.b = simplex.b2 - simplex.b1;

    // if dot(simplex.b, search_dir) < 0 {
    //     points_a: [4] Vector3;
    //     points_b: [4] Vector3;
    //     return false, points_a, points_b;
    // }

    search_dir = cross(cross(simplex.c - simplex.b, -simplex.b), simplex.c - simplex.b);
    if search_dir == .{0, 0, 0} {
        // the origin is on this line segment
        search_dir = cross(simplex.c - simplex.b, .{1, 0, 0});
        if search_dir == .{0, 0, 0} {
            search_dir = cross(simplex.c - simplex.b, .{0, 0, -1});
        }
    }

    simp_dim := 2;

    // TODO instead of this I should be able to check if the simplex is updating, at a certain point is should converge
    for i: 0..MAX_ITERATIONS - 1 {
        simplex.a1 = get_support(a, -search_dir);
        simplex.a2 = get_support(b, search_dir);
        simplex.a = simplex.a2 - simplex.a1;
        // if dot(simplex.a, search_dir) < 0 {
        //     points_a: [4] Vector3;
        //     points_b: [4] Vector3;
        //     return false, points_a, points_b;
        // }

        simp_dim += 1;
        if simp_dim == 3 {
            simp_dim, search_dir = solve_simplex3(*simplex, simp_dim, search_dir);
        } else {
            if remove_duplicates(*simplex) {
                return false, simplex;
            }

            found: bool;
            found, simp_dim, search_dir = solve_simplex4(*simplex, simp_dim, search_dir);
            if found {
                return true, simplex;
            }
        }
    }
    
    remove_duplicates(*simplex);
    return false, simplex;
}

Simplex :: struct {
    a: Vector3;
    b: Vector3;
    c: Vector3;
    d: Vector3;

    a1: Vector3;
    b1: Vector3;
    c1: Vector3;
    d1: Vector3;

    a2: Vector3;
    b2: Vector3;
    c2: Vector3;
    d2: Vector3;

    count: int;

#place a1;
    s1: [4] Vector3 = ---;

#place a2;
    s2: [4] Vector3 = ---;
}

#scope_file

solve_simplex3 :: (simplex: *Simplex, simp_dim: int, search_direction: Vector3) -> int, Vector3 {
    search_dir := search_direction;

    n := cross(simplex.b - simplex.a, simplex.c - simplex.a);
    ao := -simplex.a;

    simp_dim = 2;
    if dot(cross(simplex.b - simplex.a, n), ao) >= 0 {
        simplex.c1 = simplex.a1;
        simplex.c2 = simplex.a2;
        simplex.c = simplex.a;
        search_dir = cross(cross(simplex.b - simplex.a, ao), simplex.b - simplex.a);
        return simp_dim, search_dir;
    }

    if dot(cross(n, simplex.c - simplex.a), ao) >= 0 {
        simplex.b1 = simplex.a1;
        simplex.b2 = simplex.a2;
        simplex.b = simplex.a;
        search_dir = cross(cross(simplex.c - simplex.a, ao), simplex.c - simplex.a);
        return simp_dim, search_dir;
    }

    simp_dim = 3;
    if dot(n, ao) >= 0 {
        simplex.d1 = simplex.c1;
        simplex.d2 = simplex.c2;
        simplex.d = simplex.c;
        simplex.c1 = simplex.b1;
        simplex.c2 = simplex.b2;
        simplex.c = simplex.b;
        simplex.b1 = simplex.a1;
        simplex.b2 = simplex.a2;
        simplex.b = simplex.a;
        search_dir = n;
        return simp_dim, search_dir;
    }

    simplex.d1 = simplex.b1;
    simplex.d2 = simplex.b2;
    simplex.d = simplex.b;
    simplex.b1 = simplex.a1;
    simplex.b2 = simplex.a2;
    simplex.b = simplex.a;
    search_dir = -n;
    return simp_dim, search_dir;
}

solve_simplex4 :: (simplex: *Simplex, simp_dim: int, search_direction: Vector3) -> bool, int, Vector3 {
    search_dir := search_direction;

    abc := cross(simplex.b - simplex.a, simplex.c - simplex.a);
    acd := cross(simplex.c - simplex.a, simplex.d - simplex.a);
    adb := cross(simplex.d - simplex.a, simplex.b - simplex.a);
    cbd := cross(simplex.b - simplex.c, simplex.d - simplex.c);

    {
        #import "KodaJai";
        depth_test := get_depth_test();
        defer set_depth_test(depth_test);
        set_depth_test(false);
        cull_face := get_cull_face();
        defer set_cull_face(cull_face);
        set_cull_face(false);

        draw_triangle(simplex.a, simplex.b, simplex.c, .{1, 1, 1, 0.5});
        draw_triangle(simplex.a, simplex.c, simplex.d, .{1, 1, 1, 0.5});
        draw_triangle(simplex.a, simplex.d, simplex.b, .{1, 1, 1, 0.5});
        draw_triangle(simplex.b, simplex.c, simplex.d, .{1, 1, 1, 0.5});
        draw_line((simplex.a + simplex.b + simplex.c) / 3.0, (simplex.a + simplex.b + simplex.c) / 3.0 + normalize(abc) * 0.5, color = .{1.0, 0.0, 0.0, 0.5});
        draw_line((simplex.a + simplex.c + simplex.d) / 3.0, (simplex.a + simplex.c + simplex.d) / 3.0 + normalize(acd) * 0.5, color = .{1.0, 0.0, 0.0, 0.5});
        draw_line((simplex.a + simplex.d + simplex.b) / 3.0, (simplex.a + simplex.d + simplex.b) / 3.0 + normalize(adb) * 0.5, color = .{1.0, 0.0, 0.0, 0.5});
        draw_line((simplex.b + simplex.c + simplex.d) / 3.0, (simplex.b + simplex.c + simplex.d) / 3.0 + normalize(cbd) * 0.5, color = .{1.0, 0.0, 0.0, 0.5});
    }

    ao := -simplex.a;
    simp_dim = 3;

    if dot(abc, ao) >= 0 {
        simplex.d1 = simplex.c1;
        simplex.d2 = simplex.c2;
        simplex.d = simplex.c;
        simplex.c1 = simplex.b1;
        simplex.c2 = simplex.b2;
        simplex.c = simplex.b;
        simplex.b1 = simplex.a1;
        simplex.b2 = simplex.a2;
        simplex.b = simplex.a;
        search_dir = abc;
        print("returned abc\n");
        return false, simp_dim, search_dir;
    }
    if dot(acd, ao) >= 0 {
        simplex.b1 = simplex.a1;
        simplex.b2 = simplex.a2;
        simplex.b = simplex.a;
        search_dir = acd;
        print("returned acd\n");
        return false, simp_dim, search_dir;
    }
    if dot(adb, ao) >= 0 {
        simplex.c1 = simplex.d1;
        simplex.c2 = simplex.d2;
        simplex.c = simplex.d;
        simplex.d1 = simplex.b1;
        simplex.d2 = simplex.b2;
        simplex.d = simplex.b;
        simplex.b1 = simplex.a1;
        simplex.b2 = simplex.a2;
        simplex.b = simplex.a;
        search_dir = adb;
        print("returned adb\n");
        return false, simp_dim, search_dir;
    }

    print("returned inside\n");
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

remove_duplicates :: (simplex: *Simplex) -> bool {
    simplex.count = 4;

    dupe := false;

    check_index := 0;
    while check_index < simplex.count {
        if check_index == 0 {
            dupe_a := false;
            dupe_a = dupe_a || simplex.a == simplex.d;
            dupe_a = dupe_a || simplex.a == simplex.c;
            dupe_a = dupe_a || simplex.a == simplex.b;
            if dupe_a {
                dupe = true;
                simplex.a1 = simplex.b1;
                simplex.a2 = simplex.b2;
                simplex.a = simplex.b;
                simplex.b1 = simplex.c1;
                simplex.b2 = simplex.c2;
                simplex.b = simplex.c;
                simplex.c1 = simplex.d1;
                simplex.c2 = simplex.d2;
                simplex.c = simplex.d;
                simplex.count -= 1;
            } else {
                check_index += 1;
            }
        } else if check_index == 1 {
            dupe_b := false;
            dupe_b = dupe_b || simplex.b == simplex.d;
            dupe_b = dupe_b || simplex.b == simplex.c;
            if dupe_b {
                dupe = true;
                simplex.b1 = simplex.c1;
                simplex.b2 = simplex.c2;
                simplex.b = simplex.c;
                simplex.c1 = simplex.d1;
                simplex.c2 = simplex.d2;
                simplex.c = simplex.d;
                simplex.count -= 1;
            } else {
                check_index += 1;
            }
        } else if check_index == 2 {
            dupe_c := simplex.c == simplex.d;
            if dupe_c {
                dupe = true;
                simplex.c1 = simplex.d1;
                simplex.c2 = simplex.d2;
                simplex.c = simplex.d;
                simplex.count -= 1;
            } else {
                check_index += 1;
            }
        } else {
            check_index += 1;
        }
    }

    return dupe;
}
