
solve_epa :: (v1: KodaVertices($N1), v2: KodaVertices($N2), simplex: Simplex, d: Vector3) -> Vector3 {
    direction := -d;
    assert(length_squared(direction) > 0, "The direction provided must be greater than 0.");

    #if EPA_DEBUG {
        {
            #import "KodaJai";
            if input.keys[Key.B] & .START {
                epa_debug_max_steps -= 1;
            }
            if input.keys[Key.M] & .START {
                epa_debug_max_steps = 0;
            }
            if input.keys[Key.N] & .START {
                epa_debug_max_steps += 1;
            }
        }

        epa_debug_steps = 0;
    }

    draw_line(.{0, 0.1, 0}, direction * 10.0 + .{0, 0.1, 0});

    EPSILON :: 0.0001;
    SMALL_EPSILON :: 0.000001;

    simplex: Simplex;
    simplex.count = 1;

    search_direction := direction;
    simplex.a = get_support(v2, search_direction) - get_support(v1, -search_direction);

    search_direction = cross(cross(simplex.a, direction), simplex.a);
    if step_debug(simplex, v1, v2, search_direction) {
        return .{};
    }
    if dot(search_direction, search_direction) < EPSILON {
        // simplex a is exactly in the desired direction
        return simplex.a;
    }

    simplex.count = 2;
    simplex.b = get_support(v2, search_direction) - get_support(v1, -search_direction);

    furthest_triangle: Triangle3;
    furthest_normal: Vector3;
    furthest_direction: float;

    // this loop tries to get the direction ray to be inside the found triangle
    while true {
        if simplex.count == {
            case 2;
                a := simplex.a;
                b := simplex.b;
                ab := b - a;
                abdir := cross(ab, a);
                if dot(abdir, abdir) < EPSILON {
                    print("early1\n");
                    // this means a and b are the same so return a
                    return -simplex.a;
                }
                abc := cross(abdir, ab);
                normal := normalize(abc);

                projected := dot(a, normal) / dot(normal, direction) * direction;

                projected_normal := cross(ab, projected - a);
                if dot(projected_normal, projected_normal) < SMALL_EPSILON {
                    t := dot(projected - a, ab) / dot(ab, ab);
                    if t >= 0 && t <= 1 {
                        // this means the projected point basically falls on the line
                        // so we'll continue on to the second loop I guess?
                        search_direction = cross(normal, ab);
                        if step_debug(simplex, v1, v2, search_direction) {
                            print("early3\n");
                            return .{};
                        }
                        
                        simplex.count = 3;
                        simplex.c = get_support(v2, search_direction) - get_support(v1, -search_direction);

                        if dot(cross(simplex.c - simplex.a, simplex.b - simplex.a), direction) > 0 {
                            simplex.a = b;
                            simplex.b = a;
                        }

                        print("early out\n");

                        furthest_triangle = create_triangle3(simplex.a, simplex.b, simplex.c);
                        furthest_normal = normal;
                        furthest_direction = dot(furthest_normal, furthest_triangle.a) / dot(furthest_normal, direction);
                        break;
                    } else {
                        // the point is aligned with the line, but doesn't fall on the line
                    search_direction = cross(normal, ab);
                    }
                } else {
                    search_direction = cross(projected_normal, ab);
                }

                if step_debug(simplex, v1, v2, search_direction) {
                    print("early3\n");
                    return .{};
                }
                
                simplex.count = 3;
                simplex.c = get_support(v2, search_direction) - get_support(v1, -search_direction);

                if dot(cross(simplex.c - simplex.a, simplex.b - simplex.a), direction) > 0 {
                    simplex.a = b;
                    simplex.b = a;
                }

            case 3;
                a := simplex.a;
                b := simplex.b;
                c := simplex.c;
                abc := cross(b - a, c - a);

                if step_debug(simplex, v1, v2, .{0, 0, 0}) {
                    print("early4\n");
                    return .{};
                }

                if dot(abc, abc) < EPSILON {
                    // this means the triangle is basically a line, so we'll return whatever we've found so far because if this happens presumably we're at the end
                    print("This is bad 1.\n");
                    return .{};
                }

                normal := normalize(abc);

                if dot(abc, direction) < EPSILON {
                    step_debug(simplex, v1, v2, .{0, 0, 0}, true);
                    // the triangle is vertically aligned with desired direction
                    // or the triangle is facing the opposite direction
                    print("This is bad 2.\n");
                    return .{};
                }

                projected := dot(a, normal) / dot(normal, direction) * direction;

                ab := b - a;
                bc := c - b;
                ca := a - c;
                if dot(cross(ab, normal), projected - a) > EPSILON {
                    // the simplex is ba
                    simplex.a = b;
                    simplex.b = a;
                    simplex.count = 2;
                } else if dot(cross(bc, normal), projected - b) > EPSILON {
                    // the simplex is cb
                    simplex.a = c;
                    simplex.b = b;
                    simplex.count = 2;
                } else if dot(cross(ca, normal), projected - c) > EPSILON {
                    // the simplex is ac
                    simplex.a = a;
                    simplex.b = c;
                    simplex.count = 2;
                } else {
                    print("immediate exit\n");
                    // the point is inside the triangle
                    furthest_triangle = create_triangle3(simplex.a, simplex.b, simplex.c);
                    furthest_normal = normal;
                    furthest_direction = dot(furthest_normal, furthest_triangle.a) / dot(furthest_normal, direction);
                    break;
                }
        }
    }

    // this loop takes the triangle with the ray inside and continues pushing forward until it's reached the end
    while true {
        if step_debug(simplex, v1, v2, search_direction) {
            return .{};
        }

        b := simplex.a;
        c := simplex.b;
        d := simplex.c;
        bcd := cross(c - b, d - b);

        if dot(bcd, bcd) < EPSILON {
            print("exit1\n");
            // this means the triangle is basically a line, so we'll return whatever we've found so far because if this happens presumably we're at the end
            return -furthest_direction * direction;
        }

        normal := normalize(bcd);

        if dot(bcd, direction) < EPSILON {
            print("exit2\n");
            // the triangle is vertically aligned with desired direction
            // or the triangle is facing the wrong way
            return -furthest_direction * direction;
        }

        search_direction = bcd;
        if step_debug(simplex, v1, v2, search_direction) {
            return .{};
        }
        a := get_support(v2, search_direction) - get_support(v1, -search_direction);

        if step_debug(simplex, v1, v2, search_direction) {
            draw_cube(a - .{0.02, 0.02, 0.02}, a + .{0.02, 0.02, 0.02}, color = .{1.0, 1.0, 1.0, 1.0});

            draw_triangle(b, c, a, color = .{0.5, 0.2, 0.8, 0.2});
            draw_triangle(c, d, a, color = .{0.5, 0.2, 0.8, 0.2});
            draw_triangle(d, b, a, color = .{0.5, 0.2, 0.8, 0.2});
            draw_line(b, a, color = .{1, 1, 1, 1});
            draw_line(c, a, color = .{1, 1, 1, 1});
            draw_line(d, a, color = .{1, 1, 1, 1});
            return .{};
        }

        // if the point in front of the triangle is about on the plane of the triangle, return the triangle
        normal_dist := dot(normal, a) - dot(normal, b);
        if abs(normal_dist) < EPSILON {
            print("exit3\n");
            step_debug(simplex, v1, v2, search_direction, true);
            // print("should be approx equal? % %\n", -normal * dot(b, normal), -(a - normal_dist * normal));
            return -furthest_direction * direction;
            // return -normal * dot(b, normal);
            // nearest := a - normal_dist * normal;
        }

        abc := cross(b - a, c - a);
        normal_abc := normalize(abc);
        acd := cross(c - a, d - a);
        normal_acd := normalize(acd);
        adb := cross(d - a, b - a);
        normal_adb := normalize(adb);

        projected_abc := dot(a, normal_abc) / dot(normal_abc, direction) * direction;
        projected_acd := dot(a, normal_acd) / dot(normal_acd, direction) * direction;
        projected_adb := dot(a, normal_adb) / dot(normal_adb, direction) * direction;

        // the point is inside one of the 3 triangles
        ab := b - a;
        ac := c - a;
        ad := d - a;
        if dot(normal_abc, direction) > 0 && dot(cross(a - b, normal_abc), projected_abc - a) >= -EPSILON && dot(cross(ac, normal_abc), projected_abc - a) >= -EPSILON {
            // were in triangle abc
            simplex.a = a;
            simplex.b = b;
            simplex.c = c;

            furthest_triangle = create_triangle3(a, b, c);
            furthest_normal = normal_abc;
            furthest_direction = dot(furthest_normal, furthest_triangle.a) / dot(furthest_normal, direction);
        } else if dot(normal_acd, direction) > 0 && dot(cross(a - c, normal_acd), projected_acd - c) >= -EPSILON && dot(cross(ad, normal_acd), projected_acd - d) >= -EPSILON {
            // were in triangle acd
            simplex.a = a;
            simplex.b = c;
            simplex.c = d;

            furthest_triangle = create_triangle3(a, c, d);
            furthest_normal = normal_acd;
            furthest_direction = dot(furthest_normal, furthest_triangle.a) / dot(furthest_normal, direction);
        } else if dot(normal_adb, direction) > 0 && dot(cross(a - d, normal_adb), projected_adb - d) >= -EPSILON && dot(cross(ab, normal_adb), projected_adb - b) >= -EPSILON {
            // were in triangle adb
            simplex.a = a;
            simplex.b = d;
            simplex.c = b;

            furthest_triangle = create_triangle3(a, d, b);
            furthest_normal = normal_adb;
            furthest_direction = dot(furthest_normal, furthest_triangle.a) / dot(furthest_normal, direction);
        } else {
            // here I don't know what's happening so return the furthest triangle
            print("This is bad 3.\n");
            return furthest_direction * direction;
        }
    }

    print("exit4\n");
    return -furthest_direction * direction;
}

#scope_file

get_support :: (vertices: KodaVertices($N), direction: Vector3) -> Vector3 {
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



// debug

EPA_DEBUG :: true;

epa_debug_points: [..] Vector3;
epa_debug_steps: int;
epa_debug_max_steps: int;

step_debug :: (simplex: Simplex, v1: KodaVertices($N1), v2: KodaVertices($N2), direction: Vector3, force_draw: bool = false) -> bool {

    print_stack_trace :: () {
        node := context.stack_trace;
        while node {
            // There are two different line numbers available.
            // node.info.location has a line number, but this is the location of
            // the start of the procedure. That may be useful in some cases, but often
            // what we really want is the line number of the procedure call, which is
            // stored in node.line_number. That's what we print in this case.
            
            if node.info print("'%' at %:% ; call depth %, hash 0x%\n", node.info.name, node.info.location.fully_pathed_filename, node.line_number, node.call_depth, formatInt(node.hash, base=16));
            node = node.next;
        }
    }

    if epa_debug_steps >= epa_debug_max_steps || force_draw {
        #import "KodaJai";

        print_stack_trace();

        draw_text(tprint("EPA Step: %", epa_debug_steps), .{400, 80});

        depth_test := get_depth_test();
        defer set_depth_test(depth_test);
        set_depth_test(false);
        // cull_face := get_cull_face();
        // defer set_cull_face(cull_face);
        // set_cull_face(false);

        array_resize(*epa_debug_points, v1.count * v2.count);
        for i: 0..v1.count - 1 {
            for j: 0..v2.count - 1 {
                point := v2[j] - v1[i];
                epa_debug_points[i * v2.count + j] = point;
            }
        }

        draw_line(.{0, 0.1, 0}, -direction * 1.0 + .{0, 0.1, 0}, width = 2.0, color = .{1.0, 1.0, 1.0, 1.0});

        draw_spheres(epa_debug_points, .{0.2, 0.2, 0.2}, color = .{0.8, 0.8, 0.8, 0.1});

        if simplex.count == 1 {
            draw_cube(simplex.a - .{0.02, 0.02, 0.02}, simplex.a + .{0.02, 0.02, 0.02}, color = .{1.0, 0.0, 0.0, 1.0});
            draw_line(simplex.a, simplex.a + direction / 4.0, color = .{0.5, 0.5, 0.5, 1.0});
        } else if simplex.count == 2 {
            draw_line(simplex.a, simplex.b, color = .{1.0, 1.0, 1.0, 1.0});
            draw_cube(simplex.a - .{0.02, 0.02, 0.02}, simplex.a + .{0.02, 0.02, 0.02}, color = .{1.0, 0.0, 0.0, 1.0});
            draw_cube(simplex.b - .{0.02, 0.02, 0.02}, simplex.b + .{0.02, 0.02, 0.02}, color = .{0.0, 1.0, 0.0, 1.0});
            draw_line((simplex.a + simplex.b) / 2.0, (simplex.a + simplex.b) / 2.0 + direction / 4.0, color = .{0.5, 0.5, 0.5, 1.0});
        } else if simplex.count == 3 {
            draw_triangle(simplex.a, simplex.b, simplex.c, color = .{1.0, 1.0, 1.0, 0.2});
            draw_line(simplex.a, simplex.b, color = .{1.0, 1.0, 1.0, 1.0});
            draw_line(simplex.b, simplex.c, color = .{1.0, 1.0, 1.0, 1.0});
            draw_line(simplex.c, simplex.a, color = .{1.0, 1.0, 1.0, 1.0});
            draw_cube(simplex.a - .{0.02, 0.02, 0.02}, simplex.a + .{0.02, 0.02, 0.02}, color = .{1.0, 0.0, 0.0, 1.0});
            draw_cube(simplex.b - .{0.02, 0.02, 0.02}, simplex.b + .{0.02, 0.02, 0.02}, color = .{0.0, 1.0, 0.0, 1.0});
            draw_cube(simplex.c - .{0.02, 0.02, 0.02}, simplex.c + .{0.02, 0.02, 0.02}, color = .{0.0, 0.0, 1.0, 1.0});
            draw_line((simplex.a + simplex.b + simplex.c) / 3.0, (simplex.a + simplex.b + simplex.c) / 3.0 + direction / 4.0, color = .{0.5, 0.5, 0.5, 1.0});
        }

        return true;
    }

    epa_debug_steps += 1;

    return false;
}

#scope_export
