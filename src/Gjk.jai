Simplex :: struct {
    a: Vector3;
    b: Vector3;
    c: Vector3;
    d: Vector3;

    count: int;

#place a;
    s: [4] Vector3 = ---;
}

solve_gjk :: (v1: KodaVertices($N1), v2: KodaVertices($N2), velocity1: Vector3 = .{0, 0, 0}, velocity2: Vector3 = .{0, 0, 0}) -> bool, Simplex {
    MAX_ITERATIONS :: 32;

    simplex: Simplex;
    simplex.count = 1;

    direction := v1[0] - v2[0];
    simplex.a = get_support(v2, direction, velocity2) - get_support(v1, -direction, velocity1);
    direction = -simplex.a;

    #if GJK_DEBUG {
        {
            #import "KodaJai";
            if input.keys[Key.B] & .START {
                gjk_debug_max_steps -= 1;
            }
            if input.keys[Key.M] & .START {
                gjk_debug_max_steps = 0;
            }
            if input.keys[Key.N] & .START {
                gjk_debug_max_steps += 1;
            }
        }

        gjk_debug_steps = 0;
    }

    for i: 0..MAX_ITERATIONS - 1 {
        #if GJK_DEBUG {
            if step_debug(simplex, v1, v2, get_support(v2, direction, velocity2) - get_support(v1, -direction, velocity1)) {
                return false;
            }
        }

        simplex.d = simplex.c;
        simplex.c = simplex.b;
        simplex.b = simplex.a;
        simplex.count += 1;

        simplex.a = get_support(v2, direction, velocity2) - get_support(v1, -direction, velocity1);

        #if GJK_DEBUG {
            if step_debug(simplex, v1, v2, .{0, 1000, 0}) {
                return false, simplex;
            }
        }

        // if the new support isn't on the opposite side of the origin, it's impossible that we enclose the origin, so exit
        if dot(simplex.a, direction) < 0 {
            #if GJK_DEBUG {
                step_debug(simplex, v1, v2, .{0, 1000, 0}, true);
            }
            return false, simplex;
        }

        if simplex.count == {
            case 2;
                direction = solve_simplex2(*simplex, direction);

            case 3;
                direction = solve_simplex3(*simplex, direction);

            case 4;
                collide: bool;
                collide, direction = solve_simplex4(*simplex, direction);
                if collide {
                    #if GJK_DEBUG {
                        step_debug(simplex, v1, v2, .{0, 1000, 0}, true);
                    }
                    return true, simplex;
                }

            case;
                assert(false, "Invalid simplex dimension.");
                return false, simplex;
        }
    }

    return false, simplex;
}

#scope_file

solve_simplex2 :: (simplex: *Simplex, direction: Vector3) -> Vector3 {
    ab := simplex.b - simplex.a;
    ao := -simplex.a;

    if same_direction(ab, ao) {
        // the origin falls on the line
        return cross(cross(ab, ao), ab);
    } else {
        // the origin is closer to a
        simplex.count = 1;
        return ao;
    }
}

solve_simplex3 :: (simplex: *Simplex, direction: Vector3) -> Vector3 {
    abc := cross(simplex.b - simplex.a, simplex.c - simplex.a);
    ac := simplex.c - simplex.a;
    ao := -simplex.a;

    if same_direction(cross(abc, ac), ao) {
        if same_direction(ac, ao) {
            // the origin is nearest to the line ac
            simplex.b = simplex.c;
            simplex.count = 2;
            return cross(cross(ac, ao), ac);
        } else {
            ab := simplex.b - simplex.a;
            if same_direction(ab, ao) {
                // the origin is nearest to the line ab
                simplex.count = 2;
                return cross(cross(ab, ao), ab);
            } else {
                // the origin is nearest to the point a
                simplex.count = 1;
                return ao;
            }
        }
    } else {
        ab := simplex.b - simplex.a;
        if same_direction(cross(ab, abc), ao) {
            ab := simplex.b - simplex.a;
            if same_direction(ab, ao) {
                // the origin is nearest to the line ab
                simplex.count = 2;
                return cross(cross(ab, ao), ab);
            } else {
                // the origin is nearest to the point a
                simplex.count = 1;
                return ao;
            }
        } else {
            if same_direction(abc, ao) {
                // the origin is nearest to the triangle abc
                return abc;
            } else {
                // the origin is nearest to the triangle acb
                simplex.b, simplex.c = swap(simplex.b, simplex.c);
                return -abc;
            }
        }
    }
}

solve_simplex4 :: (simplex: *Simplex, direction: Vector3) -> bool, Vector3 {
    abc := cross(simplex.b - simplex.a, simplex.c - simplex.a);
    acd := cross(simplex.c - simplex.a, simplex.d - simplex.a);
    adb := cross(simplex.d - simplex.a, simplex.b - simplex.a);
    ao := -simplex.a;

    if same_direction(abc, ao) {
        // the origin is nearest to the triangle abc
        simplex.count = 3;
        return false, solve_simplex3(simplex, direction);
    }

    if same_direction(acd, ao) {
        // the origin is nearest to the triangle acd
        simplex.b = simplex.c;
        simplex.c = simplex.d;
        simplex.count = 3;
        return false, solve_simplex3(simplex, direction);
    }

    if same_direction(adb, ao) {
        // the origin is nearest to the triangle adb
        simplex.c = simplex.b;
        simplex.b = simplex.d;
        simplex.count = 3;
        return false, solve_simplex3(simplex, direction);
    }

    return true, .{0, 0, 0};
}

get_support :: (vertices: KodaVertices($N), direction: Vector3, velocity: Vector3) -> Vector3 {
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

    if dot(velocity, velocity) > 0 {
        for i: 0..vertices.count - 1 {
            vertex := vertices[i] + velocity;

            current_dot := dot(vertex, direction);
            if current_dot > largest_dot {
                largest_dot = current_dot;
                largest_vertex = vertex;
            }
        }
    }

    return largest_vertex;
}

same_direction :: (a: Vector3, b: Vector3) -> bool {
    return dot(a, b) > 0;
}



// debug

GJK_DEBUG :: false;

gjk_debug_points: [..] Vector3;
gjk_debug_steps: int;
gjk_debug_max_steps: int;

step_debug :: (simplex: Simplex, v1: KodaVertices($N1), v2: KodaVertices($N2), support: Vector3, force_draw: bool = false) -> bool {
    if gjk_debug_steps >= gjk_debug_max_steps || force_draw {
        #import "KodaJai";

        draw_text(tprint("GJK Step: %", gjk_debug_steps), .{400, 80});

        depth_test := get_depth_test();
        defer set_depth_test(depth_test);
        set_depth_test(false);
        cull_face := get_cull_face();
        defer set_cull_face(cull_face);
        set_cull_face(false);

        array_resize(*gjk_debug_points, v1.count * v2.count);
        for i: 0..v1.count - 1 {
            for j: 0..v2.count - 1 {
                point := v2[j] - v1[i];
                gjk_debug_points[i * v2.count + j] = point;
            }
        }

        draw_spheres(gjk_debug_points, .{0.2, 0.2, 0.2}, color = .{0.8, 0.8, 0.8, 0.1});

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
            {
                set_cull_face(true);
                defer set_cull_face(false);
                draw_triangle(simplex.d, simplex.c, simplex.b, color = .{1.0, 1.0, 1.0, 1.0});
            }
            draw_cube(simplex.a - .{0.02, 0.02, 0.02}, simplex.a + .{0.02, 0.02, 0.02}, color = .{1.0, 0.0, 0.0, 1.0});
            draw_cube(simplex.b - .{0.02, 0.02, 0.02}, simplex.b + .{0.02, 0.02, 0.02}, color = .{0.0, 1.0, 0.0, 1.0});
            draw_cube(simplex.c - .{0.02, 0.02, 0.02}, simplex.c + .{0.02, 0.02, 0.02}, color = .{0.0, 0.0, 1.0, 1.0});
            draw_cube(simplex.d - .{0.02, 0.02, 0.02}, simplex.d + .{0.02, 0.02, 0.02}, color = .{1.0, 0.0, 1.0, 1.0});
        }

        return true;
    }

    gjk_debug_steps += 1;

    return false;
}

#scope_export
