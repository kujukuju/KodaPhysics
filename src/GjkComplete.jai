
debug_steps: int;
max_debug_steps: int;

solve_gjk_complete :: (v1: KodaVertices, v2: KodaVertices) -> bool, ComplexSimplex {
    simplex: ComplexSimplex;

    direction := v1[0] - v2[0];
    simplex.a1 = get_support(v1, -direction);
    simplex.a2 = get_support(v2, direction);
    simplex.a = simplex.a2 - simplex.a1;
    simplex.count = 1;

    {
        #import "KodaJai";
        if input.keys[Key.B] & .START {
            max_debug_steps -= 1;
        }
        if input.keys[Key.M] & .START {
            max_debug_steps = 0;
        }
        if input.keys[Key.N] & .START {
            max_debug_steps += 1;
        }
    }

    debug_steps = 0;

    while true {
        if check_debug(simplex, v1, v2, .{1000, 1000, 1000}, .{1000, 1000, 1000}) {
            print("Exiting % %\n", debug_steps, max_debug_steps);
            return false, simplex;
        }

        p: Vector3;
        if simplex.count == {
            case 1;
                p = simplex.a;

            case 2;
                p = solve_simplex2(*simplex);
                
            case 3;
                p = solve_simplex3(*simplex);

            case 4;
                p = solve_simplex4(*simplex);

            case;
                assert(false, "Invalid simplex dimension.");
                return false, simplex;
        }

        // checking if distance squared from p to 0 is less than epsilon rather than comparing it to 0
        // because I'm not entirely sure how it would work together with the tetrahedron cross product
        // check and the support point directional check below
        if dot(p, p) < EPSILON {
            check_debug(simplex, v1, v2, p, .{1000, 1000, 1000}, true);
            return true, simplex;
        }

        direction = -p;

        support1 := get_support(v1, -direction);
        support2 := get_support(v2, direction);
        support := support2 - support1;

        if check_debug(simplex, v1, v2, p, support) {
            print("Exiting % %\n", debug_steps, max_debug_steps);
            return false, simplex;
        }

        // if p is more than or equivalently extremal in the direction of the origin, we can't find a better support point
        if dot(p, direction) >= dot(support, direction) - EPSILON {
            check_debug(simplex, v1, v2, p, support, true);
            return false, simplex;
        }

        assert(simplex.count < 4, "You cannot have a simplex that's a tetrahedron at this point.");

        simplex.d1 = simplex.c1;
        simplex.d2 = simplex.c2;
        simplex.d = simplex.c;
        simplex.c1 = simplex.b1;
        simplex.c2 = simplex.b2;
        simplex.c = simplex.b;
        simplex.b1 = simplex.a1;
        simplex.b2 = simplex.a2;
        simplex.b = simplex.a;

        simplex.count += 1;
        simplex.a1 = support1;
        simplex.a2 = support2;
        simplex.a = support;
    }

    return false, simplex;
}

#scope_file

EPSILON :: 0.000001;

ComplexSimplex :: struct {
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

#place a;
    s: [4] Vector3 = ---;

#place a1;
    s1: [4] Vector3 = ---;

#place a2;
    s2: [4] Vector3 = ---;
}

debug_points: [..] Vector3;
check_debug :: (simplex: ComplexSimplex, v1: KodaVertices, v2: KodaVertices, p: Vector3, support: Vector3, force_draw: bool = false) -> bool {
    if debug_steps >= max_debug_steps || force_draw {
        #import "KodaJai";

        draw_text(tprint("GJK Step: %", debug_steps), .{400, 50});

        depth_test := get_depth_test();
        defer set_depth_test(depth_test);
        set_depth_test(false);
        cull_face := get_cull_face();
        defer set_cull_face(cull_face);
        set_cull_face(false);

        array_resize(*debug_points, v1.count * v2.count);
        for i: 0..v1.count - 1 {
            for j: 0..v2.count - 1 {
                point := v2[j] - v1[i];
                debug_points[i * v2.count + j] = point;
            }
        }

        draw_spheres(debug_points, .{0.2, 0.2, 0.2}, color = .{0.8, 0.8, 0.8, 0.1});

        draw_cube(p - .{0.04, 0.04, 0.04}, p + .{0.04, 0.04, 0.04}, color = .{0.8, 0.8, 0.8, 1.0});
        draw_cube(support - .{0.04, 0.04, 0.04}, support + .{0.04, 0.04, 0.04}, color = .{0.4, 0.4, 0.4, 1.0});

        if simplex.count == 1 {
            draw_cube(simplex.a - .{0.02, 0.02, 0.02}, simplex.a + .{0.02, 0.02, 0.02}, color = .{1.0, 0.0, 0.0, 1.0});
        } else if simplex.count == 2 {
            draw_line(simplex.a, simplex.b, color = .{1.0, 1.0, 0.0, 1.0});
            draw_cube(simplex.a - .{0.02, 0.02, 0.02}, simplex.a + .{0.02, 0.02, 0.02}, color = .{1.0, 0.0, 0.0, 1.0});
            draw_cube(simplex.b - .{0.02, 0.02, 0.02}, simplex.b + .{0.02, 0.02, 0.02}, color = .{0.0, 1.0, 0.0, 1.0});
        } else if simplex.count == 3 {
            draw_triangle(simplex.a, simplex.b, simplex.c, color = .{1.0, 0.5, 0.2, 0.5});
            draw_line(simplex.a, simplex.b, color = .{1.0, 1.0, 0.0, 1.0});
            draw_line(simplex.b, simplex.c, color = .{0.0, 1.0, 1.0, 1.0});
            draw_line(simplex.c, simplex.a, color = .{1.0, 0.0, 1.0, 1.0});
            draw_cube(simplex.a - .{0.02, 0.02, 0.02}, simplex.a + .{0.02, 0.02, 0.02}, color = .{1.0, 0.0, 0.0, 1.0});
            draw_cube(simplex.b - .{0.02, 0.02, 0.02}, simplex.b + .{0.02, 0.02, 0.02}, color = .{0.0, 1.0, 0.0, 1.0});
            draw_cube(simplex.c - .{0.02, 0.02, 0.02}, simplex.c + .{0.02, 0.02, 0.02}, color = .{0.0, 0.0, 1.0, 1.0});
        } else if simplex.count == 4 {
            draw_triangle(simplex.a, simplex.b, simplex.c, color = .{1.0, 0.5, 0.2, 0.5});
            draw_triangle(simplex.a, simplex.c, simplex.d, color = .{0.2, 1.0, 0.5, 0.5});
            draw_triangle(simplex.a, simplex.d, simplex.b, color = .{0.5, 0.2, 1.0, 0.5});
            // draw_line(simplex.a, simplex.b, color = .{1.0, 1.0, 0.0, 1.0});
            // draw_line(simplex.b, simplex.c, color = .{0.0, 1.0, 1.0, 1.0});
            // draw_line(simplex.c, simplex.a, color = .{1.0, 0.0, 1.0, 1.0});
            // draw_line(simplex.b, simplex.c, color = .{1.0, 0.0, 1.0, 1.0});
            draw_cube(simplex.a - .{0.02, 0.02, 0.02}, simplex.a + .{0.02, 0.02, 0.02}, color = .{1.0, 0.0, 0.0, 1.0});
            draw_cube(simplex.b - .{0.02, 0.02, 0.02}, simplex.b + .{0.02, 0.02, 0.02}, color = .{0.0, 1.0, 0.0, 1.0});
            draw_cube(simplex.c - .{0.02, 0.02, 0.02}, simplex.c + .{0.02, 0.02, 0.02}, color = .{0.0, 0.0, 1.0, 1.0});
            draw_cube(simplex.d - .{0.02, 0.02, 0.02}, simplex.d + .{0.02, 0.02, 0.02}, color = .{1.0, 0.0, 1.0, 1.0});
        }

        return true;
    }

    debug_steps += 1;

    return false;
}

// every simplex solvers job is to reduce the simplex to the minimum subsimplex supporting the origin, and return the point closest to the origin

// I'm worried that it may be able to create an invalid simplex by, for example, attempting to find a third point for a line and it falls on the line
// or attempting to find a fourth point for a triangle and it falls inside the triangle
// but I think the extremal directional check in the loop would prevent this, so you can never have an invalid simplex
// so for that reason, I won't ever try to resolve any edge cases for invalid simplexes

solve_simplex2 :: (simplex: *ComplexSimplex) -> Vector3 {
    ab := simplex.b - simplex.a;
    ao := -simplex.a;

    if same_direction(ab, ao) {
        // the origin falls on the line
        // project the origin onto the line and return
        t := dot(ao, ab) / dot(ab, ab);
        return simplex.a + t * ab;
    } else {
        // the origin is closer to a
        simplex.count = 1;
        return simplex.a;
    }
}

solve_simplex3 :: (simplex: *ComplexSimplex) -> Vector3 {
    abc := cross(simplex.b - simplex.a, simplex.c - simplex.a);

    ac := simplex.c - simplex.a;
    ao := -simplex.a;

    if same_direction(cross(abc, ac), ao) {
        if same_direction(ac, ao) {
            // the origin is nearest to the line ac
            // simplex c remains as c
            simplex.b1 = simplex.c1;
            simplex.b2 = simplex.c2;
            simplex.b = simplex.c;
            simplex.count = 2;
            t := dot(ao, ac) / dot(ac, ac);
            return simplex.a + t * ac;
        } else {
            ab := simplex.b - simplex.a;
            if same_direction(ab, ao) {
                // the origin is nearest to the line ab
                simplex.count = 2;
                t := dot(ao, ab) / dot(ab, ab);
                return simplex.a + t * ab;
            } else {
                // the origin is nearest to the point a
                simplex.count = 1;
                return simplex.a;
            }
        }
    } else {
        ab := simplex.b - simplex.a;
        if same_direction(cross(ab, abc), ao) {
            ab := simplex.b - simplex.a;
            if same_direction(ab, ao) {
                // the origin is nearest to the line ab
                simplex.count = 2;
                t := dot(ao, ab) / dot(ab, ab);
                return simplex.a + t * ab;
            } else {
                // the origin is nearest to the point a
                simplex.count = 1;
                return simplex.a;
            }
        } else {
            if same_direction(abc, ao) {
                // the origin is nearest to the triangle abc
                bo := -simplex.b;
                co := -simplex.c;
                d1 := dot(ab, ao);
                d2 := dot(ac, ao);
                d3 := dot(ab, bo);
                d4 := dot(ac, bo);
                d5 := dot(ab, co);
                d6 := dot(ac, co);
                va := d3 * d6 - d5 * d4;
                vb := d5 * d2 - d1 * d6;
                vc := d1 * d4 - d3 * d2;
                denom := 1.0 / (va + vb + vc);
                v := vb * denom;
                w := vc * denom;
                return simplex.a + ab * v + ac * w;
            } else {
                // the origin is nearest to the triangle acb
                simplex.b1, simplex.c1 = swap(simplex.b1, simplex.c1);
                simplex.b2, simplex.c2 = swap(simplex.b2, simplex.c2);
                simplex.b, simplex.c = swap(simplex.b, simplex.c);
                bo := -simplex.b;
                co := -simplex.c;
                d1 := dot(ac, ao);
                d2 := dot(ab, ao);
                d3 := dot(ac, co);
                d4 := dot(ab, co);
                d5 := dot(ac, bo);
                d6 := dot(ab, bo);
                va := d3 * d6 - d5 * d4;
                vb := d5 * d2 - d1 * d6;
                vc := d1 * d4 - d3 * d2;
                denom := 1.0 / (va + vb + vc);
                v := vb * denom;
                w := vc * denom;
                return simplex.a + ab * v + ac * w;
            }
        }
    }
}

solve_simplex4 :: (simplex: *ComplexSimplex) -> Vector3 {
    abc := cross(simplex.b - simplex.a, simplex.c - simplex.a);
    acd := cross(simplex.c - simplex.a, simplex.d - simplex.a);
    adb := cross(simplex.d - simplex.a, simplex.b - simplex.a);
    ao := -simplex.a;

    if !same_direction(abc, ao) && !same_direction(acd, ao) && !same_direction(adb, ao) {
        // the origin is inside the simplex
        return .{0, 0, 0};
    }

    simplex_abc := << simplex;
    simplex_abc.count = 3;

    simplex_acd := << simplex;
    simplex_acd.b1 = simplex_acd.c1;
    simplex_acd.b2 = simplex_acd.c2;
    simplex_acd.b = simplex_acd.c;
    simplex_acd.c1 = simplex_acd.d1;
    simplex_acd.c2 = simplex_acd.d2;
    simplex_acd.c = simplex_acd.d;
    simplex_acd.count = 3;

    simplex_adb := << simplex;
    simplex_adb.c1 = simplex_adb.b1;
    simplex_adb.c2 = simplex_adb.b2;
    simplex_adb.c = simplex_adb.b;
    simplex_adb.b1 = simplex_adb.d1;
    simplex_adb.b2 = simplex_adb.d2;
    simplex_adb.b = simplex_adb.d;
    simplex_adb.count = 3;

    p_abc := solve_simplex3(*simplex_abc);
    p_acd := solve_simplex3(*simplex_acd);
    p_adb := solve_simplex3(*simplex_adb);

    abc_d2 := dot(p_abc, p_abc);
    acd_d2 := dot(p_acd, p_acd);
    adb_d2 := dot(p_adb, p_adb);

    if abc_d2 <= acd_d2 && abc_d2 <= adb_d2 {
        << simplex = simplex_abc;
        return p_abc;
    } else if acd_d2 <= abc_d2 && acd_d2 <= adb_d2 {
        << simplex = simplex_acd;
        return p_acd;
    } else if adb_d2 <= abc_d2 && adb_d2 <= acd_d2 {
        << simplex = simplex_adb;
        return p_adb;
    }

    assert(false, "This shouldn't be reachable.");

    // the origin isn't outside of any plane, so it's inside the tretrahedron
    return .{0, 0, 0};

}

get_support :: (vertices: KodaVertices, direction: Vector3) -> Vector3 {
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

same_direction :: (a: Vector3, b: Vector3) -> bool {
    return dot(a, b) > 0;
}
