
debug_steps: int;
max_debug_steps: int;

solve_gjk_complete :: (v1: KodaVertices, v2: KodaVertices) -> bool, ComplexSimplex {
    simplex: ComplexSimplex;
    simplex.count = 2;  

    direction := v1[0] - v2[0];
    simplex.b1 = get_support(v1, -direction);
    simplex.b2 = get_support(v2, direction);
    simplex.b = simplex.b2 - simplex.b1;
    direction = -simplex.b;

    {
        #import "KodaJai";
        if input.keys[Key.B] * .START {
            max_debug_steps -= 1;
        }
        if input.keys[Key.M] * .START {
            max_debug_steps = 0;
        }
        if input.keys[Key.N] & .START {
            max_debug_steps += 1;
        }
    }

    debug_steps = 0;

    while true {
        simplex.a1 = get_support(v1, -direction);
        simplex.a2 = get_support(v2, direction);
        simplex.a = simplex.a2 - simplex.a1;

        count := 0;

        // valid := correct_validity(*simplex);
        // while !valid {
        //     valid = correct_validity(*simplex);
        //     if !valid {

        //         check_debug(simplex, v1, v2);
        //         break;
        //     }
        // }
        // while !valid_vertex(simplex) && count < 1000 {
        //     count += 1;
        //     largest_dot1 := dot(simplex.a1, -direction);
        //     largest_dot2 := dot(simplex.a2, direction);

        //     simplex.a1 = get_support(v1, -direction, largest_dot1);
        //     simplex.a2 = get_support(v2, direction, largest_dot2);
        //     simplex.a = simplex.a2 - simplex.a1;
        // }

        // closest_index := 0;
        // closest_distance := FLOAT32_MAX;
        // farthest_index := 0;
        // farthest_distance := 0.0;
        // for i: 0..simplex.count - 1 {
        //     d2 := length_squared(simplex.s[i]);
        //     if d2 < closest_distance {
        //         closest_index = i;
        //         closest_distance = d2;
        //     }
        //     if d2 > farthest_distance {
        //         farthest_index = i;
        //         farthest_distance = d2;
        //     }
        // }

        // simplex.s1[closest_index], simplex.s1[farthest_index] = swap(simplex.s1[closest_index], simplex.s1[farthest_index]);
        // simplex.s2[closest_index], simplex.s2[farthest_index] = swap(simplex.s2[closest_index], simplex.s2[farthest_index]);
        // simplex.s[closest_index], simplex.s[farthest_index] = swap(simplex.s[closest_index], simplex.s[farthest_index]);

        // while !valid_closest(simplex) {
        //     first := simplex.s[0];
        //     for i: 0..simplex.count - 2 {
        //         simplex.s[i] = simplex.s[i + 1];
        //     }
        //     simplex.s[simplex.count - 1] = first;
        // }

        if count == 1000 {
            print("It's 1000.\n");
        }

        if valid_vertex(simplex) {
            draw_text("All valid", .{400, 80});
        } else {
            draw_text("Found invalid", .{400, 80});
        }

        if check_debug(simplex, v1, v2) {
            print("Exiting % %\n", debug_steps, max_debug_steps);
            return false, simplex;
        }

        if has_duplicate(simplex) {
            return false, simplex;
        }

        // cull_simplex(*simplex);

        // if simplex.count == 3 {
        //     ab := simplex.b - simplex.a;
        //     ac := simplex.c - simplex.a;
        //     if length_squared(cross(ab, ac)) == 0 {
        //         // if all 3 lines fall on the same line, delete c
        //         simplex.count = 2;
        //     }
        // }

        // if a didnt go past the origin from our previous position, there's no intersection
        // if dot(simplex.a, direction) < 0 {
        //     return;
        // }

        if simplex.count == {
            case 1;
                solve_simplex1(*simplex, *direction);

            case 2;
                solve_simplex2(*simplex, *direction);
                
            case 3;
                solve_simplex3(*simplex, *direction);

            case 4;
                if solve_simplex4(*simplex, *direction) {
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
check_debug :: (simplex: ComplexSimplex, v1: KodaVertices, v2: KodaVertices) -> bool {
    if debug_steps >= max_debug_steps {
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

solve_simplex1 :: (simplex: *ComplexSimplex, direction: *Vector3) {
    ao := -simplex.a;

    print("I don't think this can ever get called.\n");

    simplex.count = 2;
    simplex.b1 = simplex.a1;
    simplex.b2 = simplex.a2;
    simplex.b = simplex.a;
    << direction = ao;
}

solve_simplex2 :: (simplex: *ComplexSimplex, direction: *Vector3) {
    ab := simplex.b - simplex.a;
    ao := -simplex.a;

    if same_direction(ab, ao) {
        simplex.count = 3;
        simplex.c1 = simplex.b1;
        simplex.c2 = simplex.b2;
        simplex.c = simplex.b;
        simplex.b1 = simplex.a1;
        simplex.b2 = simplex.a2;
        simplex.b = simplex.a;
        << direction = cross(cross(ab, ao), ab);
    } else {
        simplex.b1 = simplex.a1;
        simplex.b2 = simplex.a2;
        simplex.b = simplex.a;
        << direction = ao;
    }
}

solve_simplex3 :: (simplex: *ComplexSimplex, direction: *Vector3) {
    abc := cross(simplex.b - simplex.a, simplex.c - simplex.a);
    ac := simplex.c - simplex.a;
    ao := -simplex.a;

    if same_direction(cross(abc, ac), ao) {
        if same_direction(ac, ao) {
            // the origin is nearest to the line ac
            // simplex c remains as c
            simplex.b1 = simplex.a1;
            simplex.b2 = simplex.a2;
            simplex.b = simplex.a;
            << direction = cross(cross(ac, ao), ac);
        } else {
            ab := simplex.b - simplex.a;
            if same_direction(ab, ao) {
                // the origin is nearest to the line ab
                simplex.c1 = simplex.b1;
                simplex.c2 = simplex.b2;
                simplex.c = simplex.b;
                simplex.b1 = simplex.a1;
                simplex.b2 = simplex.a2;
                simplex.b = simplex.a;
                << direction = cross(cross(ab, ao), ab);
            } else {
                // the origin is nearest to the point a
                simplex.count = 2;
                simplex.b1 = simplex.a1;
                simplex.b2 = simplex.a2;
                simplex.b = simplex.a;
                << direction = ao;
            }
        }
    } else {
        ab := simplex.b - simplex.a;
        if same_direction(cross(ab, abc), ao) {
            ab := simplex.b - simplex.a;
            if same_direction(ab, ao) {
                // the origin is nearest to the line ab
                simplex.c1 = simplex.b1;
                simplex.c2 = simplex.b2;
                simplex.c = simplex.b;
                simplex.b1 = simplex.a1;
                simplex.b2 = simplex.a2;
                simplex.b = simplex.a;
                << direction = cross(cross(ab, ao), ab);
            } else {
                // the origin is nearest to the point a
                simplex.count = 2;
                simplex.b1 = simplex.a1;
                simplex.b2 = simplex.a2;
                simplex.b = simplex.a;
                << direction = ao;
            }
        } else {
            if same_direction(abc, ao) {
                // the origin is nearest to the triangle abc
                simplex.count =  4;
                simplex.d1 = simplex.c1;
                simplex.d2 = simplex.c2;
                simplex.d = simplex.c;
                simplex.c1 = simplex.b1;
                simplex.c2 = simplex.b2;
                simplex.c = simplex.b;
                simplex.b1 = simplex.a1;
                simplex.b2 = simplex.a2;
                simplex.b = simplex.a;
                << direction = abc;
            } else {
                // the origin is nearest to the triangle acb
                // simplex c remains as c
                simplex.count =  4;
                simplex.d1 = simplex.b1;
                simplex.d2 = simplex.b2;
                simplex.d = simplex.b;
                simplex.b1 = simplex.a1;
                simplex.b2 = simplex.a2;
                simplex.b = simplex.a;
                << direction = -abc;
            }
        }
    }
}

solve_simplex4 :: (simplex: *ComplexSimplex, direction: *Vector3) -> bool {
    abc := cross(simplex.b - simplex.a, simplex.c - simplex.a);
    acd := cross(simplex.c - simplex.a, simplex.d - simplex.a);
    adb := cross(simplex.d - simplex.a, simplex.b - simplex.a);
    ao := -simplex.a;

    if same_direction(abc, ao) {
        // the origin is nearest to the triangle abc
        solve_simplex3(simplex, direction);
        return false;
    }
    
    if same_direction(acd, ao) {
        // the origin is nearest to the triangle acd
        // simplex a remains as a
        simplex.c1 = simplex.d1;
        simplex.c2 = simplex.d2;
        simplex.c = simplex.d;
        simplex.b1 = simplex.c1;
        simplex.b2 = simplex.c2;
        simplex.b = simplex.c;
        solve_simplex3(simplex, direction);
        return false;
    }

    if same_direction(adb, ao) {
        // the origin is nearest to the triangle adb
        // simplex a remains as a
        simplex.c1 = simplex.b1;
        simplex.c2 = simplex.b2;
        simplex.c = simplex.b;
        simplex.b1 = simplex.d1;
        simplex.b2 = simplex.d2;
        simplex.b = simplex.d;
        solve_simplex3(simplex, direction);
        return false;
    }

    return true;
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

get_support :: (vertices: KodaVertices, direction: Vector3, below_dot_product: float) -> Vector3 {
    largest_dot := dot(vertices[0], direction);
    largest_vertex := vertices[0];
    for i: 1..vertices.count - 1 {
        vertex := vertices[i];

        current_dot := dot(vertex, direction);
        if current_dot > largest_dot && current_dot < below_dot_product - EPSILON {
            largest_dot = current_dot;
            largest_vertex = vertex;
        }
    }

    return largest_vertex;
}

has_duplicate :: (simplex: ComplexSimplex) -> bool {
    for i: 0..simplex.count - 2 {
        current := simplex.s[i];

        for j: i + 1..simplex.count - 1 {
            next := simplex.s[j];

            if current == next {
                return true;
            }
        }
    }

    return false;
}

correct_validity :: (simplex: *ComplexSimplex) -> bool {
    if simplex.count == {
        case 3;
            if abs(length_squared(cross(simplex.b - simplex.a, simplex.c - simplex.a))) < EPSILON {
                b_dist := length_squared(simplex.b);
                c_dist := length_squared(simplex.c);

                if b_dist < c_dist {
                    simplex.count = 2;
                    return true;
                } else {
                    simplex.count = 2;
                    simplex.b1 = simplex.c1;
                    simplex.b2 = simplex.c2;
                    simplex.b = simplex.c;
                    return true;
                }
            }

        case 4;
            if abs(dot(simplex.d - simplex.a, cross(simplex.b - simplex.a, simplex.c - simplex.a))) < EPSILON {
                // b_dist := length_squared(simplex.b);
                // c_dist := length_squared(simplex.c);
                // d_dist := length_squared(simplex.d);

                // TODO im not sure what to do here, so I'm just going to cut out d maybe?

                // simplex.count = 3;
                return false;
            }
    }

    return true;
}

valid_vertex :: (simplex: ComplexSimplex) -> bool {
    // were simplex a
    // we disallow vertice that fall onto the edge of a simplex, but allow vertices that fall onto an existing vertex
    if simplex.count == {
        case 3;
            if abs(length_squared(cross(simplex.b - simplex.a, simplex.c - simplex.a))) < EPSILON {
                if simplex.a != simplex.b && simplex.a != simplex.c {
                    return false;
                }
            }

        case 4;
            if abs(dot(simplex.d - simplex.a, cross(simplex.b - simplex.a, simplex.c - simplex.a))) < EPSILON {
                if simplex.a != simplex.b && simplex.a != simplex.c && simplex.a != simplex.d {
                    return false;
                }
            }
    }

    return true;
}

valid_closest :: (simplex: ComplexSimplex) -> bool {
    a_dist2 := length_squared(simplex.a);
    for i: 1..simplex.count - 1 {
        if length_squared(simplex.s[i]) < a_dist2 {
            return false;
        }
    }

    return true;
}

// cull_simplex :: (simplex: *ComplexSimplex) {

// }

same_direction :: (a: Vector3, b: Vector3) -> bool {
    return dot(a, b) > 0;
}

// same_direction_or_plane  :: (a: Vector3, b: Vector3) -> bool {
//     return dot(a, b) >= 0;
// }
