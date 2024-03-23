Simplex :: struct {
    a: Vector3;
    b: Vector3;
    c: Vector3;
    d: Vector3;

    count: int;

#place a;
    s: [4] Vector3 = ---;
}

solve_gjk :: (v1: *KodaVertices, v2: *KodaVertices, transform1: Matrix4 = Matrix4Identity, transform2: Matrix4 = Matrix4Identity, cached_directions: *Simplex = null) -> bool, Simplex, Simplex {
    MAX_ITERATIONS :: 32;

    // not actually sure I need this
    is_valid_tetrahedron :: (simplex: Simplex) -> bool {
        s := cross(simplex.a, simplex.b);
        t := cross(simplex.c, simplex.d);
        u := simplex.a - simplex.b;
        v := simplex.c - simplex.d;

        return abs(dot(s, v) + dot(t, u)) > 0.0001;
    }

    // if objects rotate or move between frames youre not guaranteed that the previous simplex can be cached in such a way that
    // the given point is maximal for both shapes in a given direction, so we have to track that info for caching

    simplex: Simplex;
    directions: Simplex;
    next_direction: Vector3;
    if !cached_directions {
        directions.a = v1.fixed[0] - v2.fixed[0];
        directions.count = 1;
        simplex.a = get_support(v2, directions.a, transform2) - get_support(v1, -directions.a, transform1);
        simplex.count = 1;
        if dot(simplex.a, directions.a) < 0 {
            return false, .{}, .{};
        }

        next_direction = -simplex.a;
    } else {
        assert(cached_directions.count == 4, "You can only provide an initial simplex of count 4.");
        directions = << cached_directions;
        simplex.a = get_support(v2, directions.a, transform2) - get_support(v1, -directions.a, transform1);
        if dot(simplex.a, directions.a) < 0 {
            return false, .{}, .{};
        }
        simplex.b = get_support(v2, directions.b, transform2) - get_support(v1, -directions.b, transform1);
        if dot(simplex.b, directions.b) < 0 {
            return false, .{}, .{};
        }
        simplex.c = get_support(v2, directions.c, transform2) - get_support(v1, -directions.c, transform1);
        if dot(simplex.c, directions.c) < 0 {
            return false, .{}, .{};
        }
        simplex.d = get_support(v2, directions.d, transform2) - get_support(v1, -directions.d, transform1);
        if dot(simplex.d, directions.d) < 0 {
            return false, .{}, .{};
        }
        simplex.count = 4;

        if is_valid_tetrahedron(simplex) {
            collide: bool;
            collide, next_direction = solve_simplex4(*simplex, *directions);
            if collide {
                #if GJK_DEBUG {
                    step_debug(simplex, v1, v2, .{0, 1000, 0}, true);
                }
                return true, simplex, directions;
            }
        } else {
            directions.a = v1.fixed[0] - v2.fixed[0];
            directions.count = 1;
            simplex.a = get_support(v2, directions.a, transform2) - get_support(v1, -directions.a, transform1);
            simplex.count = 1;
            if dot(simplex.a, directions.a) < 0 {
                return false, .{}, .{};
            }

            next_direction = -simplex.a;
        }
    }

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
            if step_debug(simplex, v1, v2, get_support(v2, next_direction, transform2) - get_support(v1, -next_direction, transform1)) {
                return false, .{}, .{};
            }
        }

        directions.d = directions.c;
        directions.c = directions.b;
        directions.b = directions.a;
        directions.count += 1;
        simplex.d = simplex.c;
        simplex.c = simplex.b;
        simplex.b = simplex.a;
        simplex.count += 1;

        directions.a = next_direction;
        simplex.a = get_support(v2, directions.a, transform2) - get_support(v1, -directions.a, transform1);

        #if GJK_DEBUG {
            if step_debug(simplex, v1, v2, .{0, 1000, 0}) {
                return false, .{}, .{};
            }
        }

        // if the new support isn't on the opposite side of the origin, it's impossible that we enclose the origin, so exit
        if dot(simplex.a, next_direction) < 0 {
            #if GJK_DEBUG {
                step_debug(simplex, v1, v2, .{0, 1000, 0}, true);
            }
            return false, .{}, .{};
        }

        if simplex.count == {
            case 2;
                next_direction = solve_simplex2(*simplex, *directions);

            case 3;
                next_direction = solve_simplex3(*simplex, *directions);

            case 4;
                collide: bool;
                collide, next_direction = solve_simplex4(*simplex, *directions);
                if collide {
                    #if GJK_DEBUG {
                        step_debug(simplex, v1, v2, .{0, 1000, 0}, true);
                    }
                    return true, simplex, directions;
                }

            case;
                assert(false, "Invalid simplex dimension.");
                return false, .{}, .{};
        }
    }

    return false, .{}, .{};
}

get_support :: (vertices: *KodaVertices, direction: Vector3) -> Vector3 {
    largest_dot := dot(vertices.fixed[0], direction);
    largest_vertex := vertices.fixed[0];
    for i: 1..vertices.count - 1 {
        vertex := vertices.fixed[i];

        current_dot := dot(vertex, direction);
        if current_dot > largest_dot {
            largest_dot = current_dot;
            largest_vertex = vertex;
        }
    }

    return largest_vertex;
}

get_support :: (vertices: *KodaVertices, direction: Vector3, velocity: Vector3) -> Vector3 {
    largest_dot := dot(vertices.fixed[0], direction);
    largest_vertex := vertices.fixed[0];
    for i: 1..vertices.count - 1 {
        vertex := vertices.fixed[i];

        current_dot := dot(vertex, direction);
        if current_dot > largest_dot {
            largest_dot = current_dot;
            largest_vertex = vertex;
        }
    }

    if dot(velocity, velocity) > 0 {
        for i: 0..vertices.count - 1 {
            vertex := vertices.fixed[i] + velocity;

            current_dot := dot(vertex, direction);
            if current_dot > largest_dot {
                largest_dot = current_dot;
                largest_vertex = vertex;
            }
        }
    }

    return largest_vertex;
}

get_support :: (vertices: *KodaVertices, direction: Vector3, transform: Matrix4) -> Vector3 {
    transformed_direction := (inverse(transform) * Vector4.{direction.x, direction.y, direction.z, 0}).xyz;

    largest_dot := dot(vertices.fixed[0], transformed_direction);
    largest_vertex := vertices.fixed[0];
    for i: 1..vertices.count - 1 {
        vertex := vertices.fixed[i];

        current_dot := dot(vertex, transformed_direction);
        if current_dot > largest_dot {
            largest_dot = current_dot;
            largest_vertex = vertex;
        }
    }

    largest_vertex = (transform * Vector4.{largest_vertex.x, largest_vertex.y, largest_vertex.z, 1}).xyz;

    return largest_vertex;
}

// would be easy to add one with both velocity and transform but I haven't yet

#scope_file

solve_simplex2 :: (simplex: *Simplex, directions: *Simplex) -> Vector3 {
    ab := simplex.b - simplex.a;
    ao := -simplex.a;

    if same_direction(ab, ao) {
        // the origin falls on the line
        return cross(cross(ab, ao), ab);
    } else {
        // the origin is closer to a
        directions.count = 1;
        simplex.count = 1;
        return ao;
    }
}

solve_simplex3 :: (simplex: *Simplex, directions: *Simplex) -> Vector3 {
    abc := cross(simplex.b - simplex.a, simplex.c - simplex.a);
    ac := simplex.c - simplex.a;
    ao := -simplex.a;

    if same_direction(cross(abc, ac), ao) {
        if same_direction(ac, ao) {
            // the origin is nearest to the line ac
            directions.b = directions.c;
            directions.count = 2;
            simplex.b = simplex.c;
            simplex.count = 2;
            return cross(cross(ac, ao), ac);
        } else {
            ab := simplex.b - simplex.a;
            if same_direction(ab, ao) {
                // the origin is nearest to the line ab
                directions.count = 2;
                simplex.count = 2;
                return cross(cross(ab, ao), ab);
            } else {
                // the origin is nearest to the point a
                directions.count = 1;
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
                directions.count = 2;
                simplex.count = 2;
                return cross(cross(ab, ao), ab);
            } else {
                // the origin is nearest to the point a
                directions.count = 1;
                simplex.count = 1;
                return ao;
            }
        } else {
            if same_direction(abc, ao) {
                // the origin is nearest to the triangle abc
                return abc;
            } else {
                // the origin is nearest to the triangle acb
                directions.b, directions.c = swap(directions.b, directions.c);
                simplex.b, simplex.c = swap(simplex.b, simplex.c);
                return -abc;
            }
        }
    }
}

solve_simplex4 :: (simplex: *Simplex, directions: *Simplex) -> bool, Vector3 {
    abc := cross(simplex.b - simplex.a, simplex.c - simplex.a);
    ao := -simplex.a;

    if same_direction(abc, ao) {
        // the origin is nearest to the triangle abc
        directions.count = 3;
        simplex.count = 3;
        return false, solve_simplex3(simplex, directions);
    }

    acd := cross(simplex.c - simplex.a, simplex.d - simplex.a);

    if same_direction(acd, ao) {
        // the origin is nearest to the triangle acd
        directions.b = directions.c;
        directions.c = directions.d;
        directions.count = 3;
        simplex.b = simplex.c;
        simplex.c = simplex.d;
        simplex.count = 3;
        return false, solve_simplex3(simplex, directions);
    }

    adb := cross(simplex.d - simplex.a, simplex.b - simplex.a);

    if same_direction(adb, ao) {
        // the origin is nearest to the triangle adb
        directions.c = directions.b;
        directions.b = directions.d;
        directions.count = 3;
        simplex.c = simplex.b;
        simplex.b = simplex.d;
        simplex.count = 3;
        return false, solve_simplex3(simplex, directions);
    }

    bdc := cross(simplex.d - simplex.b, simplex.c - simplex.b);
    bo := -simplex.b;

    // we normally wouldn't have to check bdc, but since we're allowing cached simplexes we do
    if same_direction(bdc, bo) {
        directions.a = directions.b;
        directions.b = directions.d;
        directions.count = 3;
        simplex.a = simplex.b;
        simplex.b = simplex.d;
        simplex.count = 3;
        return false, solve_simplex3(simplex, directions);
    }

    return true, .{0, 0, 0};
}

same_direction :: (a: Vector3, b: Vector3) -> bool {
    return dot(a, b) > 0;
}



// debug

GJK_DEBUG :: false;

gjk_debug_points: [..] Vector3;
gjk_debug_steps: int;
gjk_debug_max_steps: int;

step_debug :: (simplex: Simplex, v1: KodaVertices, v2: KodaVertices, support: Vector3, force_draw: bool = false) -> bool {
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
