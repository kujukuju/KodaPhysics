
// this algorithm starts off with a random line, and makes an attempt to not align the line with the directional vector in any way
// it 
// solve_gjk_dir :: (v1: KodaVertices($N1), v2: KodaVertices($N2), simplex: Simplex, d: Vector3) -> Vector3 {
solve_gjk_dir :: (v1: *KodaVertices($N1), v2: *KodaVertices($N2), dir: Vector3, transform1: Matrix4 = Matrix4Identity, transform2: Matrix4 = Matrix4Identity) -> Vector3, Vector3 {
    direction := -dir;
    assert(length_squared(direction) > 0, "The direction provided must be greater than 0.");

    #if GJK_DIR_DEBUG {
        {
            #import "KodaJai";
            if input.keys[Key.B] & .START {
                gjk_dir_debug_max_steps -= 1;
            }
            if input.keys[Key.M] & .START {
                gjk_dir_debug_max_steps = 0;
            }
            if input.keys[Key.N] & .START {
                gjk_dir_debug_max_steps += 1;
            }
        }

        gjk_dir_debug_steps = 0;

        draw_line(.{0, 0.1, 0}, direction * 10.0 + .{0, 0.1, 0});
    }

    EPSILON :: 0.0001;
    SMALL_EPSILON :: 0.000001;
    MAX_ITERATIONS :: 64;

    simplex: Simplex;
    simplex1: Simplex;
    simplex2: Simplex;
    search_direction := cross(direction, .{0, 1, 0});
    if dot(search_direction, search_direction) < EPSILON {
        search_direction = cross(direction, .{0, 0, -1});
    }
    search_direction = (search_direction + direction) / 2.0;

    #if GJK_DIR_DEBUG {
        if step_debug(simplex, v1, v2, search_direction) {
            return .{}, .{};
        }
    }
    simplex1.count = 2;
    simplex1.a = get_support(v1, -search_direction, transform1);
    simplex2.count = 2;
    simplex2.a = get_support(v2, search_direction, transform2);
    simplex.count = 2;
    simplex.a = simplex2.a - simplex1.a;

    search_normal := cross(search_direction, direction);
    search_direction = search_normal;
    search_direction = (search_direction + direction) / 2.0;
    simplex1.b = get_support(v1, -search_direction, transform1);
    simplex2.b = get_support(v2, search_direction, transform2);
    simplex.b = simplex2.b - simplex1.b;

    furthest_direction: float;
    furthest_simplex1: Simplex;
    furthest_simplex2: Simplex;

    // this loop tries to get the direction ray to be inside the found triangle
    iterations := 0;
    while true {
        if simplex.count == {
            case 2;
                a1 := simplex1.a;
                b1 := simplex1.b;
                a2 := simplex2.a;
                b2 := simplex2.b;
                a := simplex.a;
                b := simplex.b;
                ab := b - a;
                abdir := cross(ab, a);
                if dot(abdir, abdir) < EPSILON {
                    // this means ab is aligned with ao
                    return .{}, .{};
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
                        #if GJK_DIR_DEBUG {
                            if step_debug(simplex, v1, v2, search_direction) {
                                return .{}, .{};
                            }
                        }
                        
                        simplex1.count = 3;
                        simplex1.c = get_support(v1, -search_direction, transform1);
                        simplex2.count = 3;
                        simplex2.c = get_support(v2, search_direction, transform2);
                        simplex.count = 3;
                        simplex.c = simplex2.c - simplex1.c;

                        if dot(cross(simplex.c - simplex.a, simplex.b - simplex.a), direction) > 0 {
                            simplex.a = b;
                            simplex.b = a;
                        }

                        furthest_direction = dot(normal, simplex.a) / dot(normal, direction);
                        furthest_simplex1 = simplex1;
                        furthest_simplex2 = simplex2;
                        break;
                    } else {
                        // the point is aligned with the line, but doesn't fall on the line
                        search_direction = cross(normal, ab);
                    }
                } else {
                    search_direction = cross(projected_normal, ab);
                }

                #if GJK_DIR_DEBUG {
                    if step_debug(simplex, v1, v2, search_direction) {
                        return .{}, .{};
                    }
                }
                
                simplex1.count = 3;
                simplex1.c = get_support(v1, -search_direction, transform1);
                simplex2.count = 3;
                simplex2.c = get_support(v2, search_direction, transform2);
                simplex.count = 3;
                simplex.c = simplex2.c - simplex1.c;

                if dot(cross(simplex.c - simplex.a, simplex.b - simplex.a), direction) > 0 {
                    simplex1.a = b1;
                    simplex1.b = a1;
                    simplex2.a = b2;
                    simplex2.b = a2;
                    simplex.a = b;
                    simplex.b = a;
                }

            case 3;
                a1 := simplex1.a;
                b1 := simplex1.b;
                c1 := simplex1.c;
                a2 := simplex2.a;
                b2 := simplex2.b;
                c2 := simplex2.c;
                a := simplex.a;
                b := simplex.b;
                c := simplex.c;
                abc := cross(b - a, c - a);

                #if GJK_DIR_DEBUG {
                    if step_debug(simplex, v1, v2, .{0, 0, 0}) {
                        return .{}, .{};
                    }
                }

                if dot(abc, abc) < EPSILON {
                    // this means the triangle is basically a line, so we'll return whatever we've found so far because if this happens presumably we're at the end
                    return .{}, .{};
                }

                normal := normalize(abc);

                if dot(abc, direction) < EPSILON {
                    #if GJK_DIR_DEBUG {
                        step_debug(simplex, v1, v2, .{0, 0, 0}, true);
                    }
                    // the triangle is vertically aligned with desired direction
                    // or the triangle is facing the opposite direction
                    return .{}, .{};
                }

                projected := dot(a, normal) / dot(normal, direction) * direction;

                ab := b - a;
                bc := c - b;
                ca := a - c;
                if dot(cross(ab, normal), projected - a) > EPSILON {
                    // the simplex is ba
                    simplex1.a = b1;
                    simplex1.b = a1;
                    simplex1.count = 2;
                    simplex2.a = b2;
                    simplex2.b = a2;
                    simplex2.count = 2;
                    simplex.a = b;
                    simplex.b = a;
                    simplex.count = 2;
                } else if dot(cross(bc, normal), projected - b) > EPSILON {
                    // the simplex is cb
                    simplex1.a = c1;
                    simplex1.b = b1;
                    simplex1.count = 2;
                    simplex2.a = c2;
                    simplex2.b = b2;
                    simplex2.count = 2;
                    simplex.a = c;
                    simplex.b = b;
                    simplex.count = 2;
                } else if dot(cross(ca, normal), projected - c) > EPSILON {
                    // the simplex is ac
                    simplex1.a = a1;
                    simplex1.b = c1;
                    simplex1.count = 2;
                    simplex2.a = a2;
                    simplex2.b = c2;
                    simplex2.count = 2;
                    simplex.a = a;
                    simplex.b = c;
                    simplex.count = 2;
                } else {
                    // the point is inside the triangle
                    furthest_direction = dot(normal, simplex.a) / dot(normal, direction);
                    furthest_simplex1 = simplex1;
                    furthest_simplex2 = simplex2;
                    break;
                }
        }

        iterations += 1;
        if iterations >= MAX_ITERATIONS {
            // we haven't even found a furthest triangle yet
            // print("Exiting without finding a triangle.\n");
            return .{}, .{};
        }
    }

    // this loop takes the triangle with the ray inside and continues pushing forward until it's reached the end
    for i: 0..MAX_ITERATIONS - 1 {
        #if GJK_DIR_DEBUG {
            if step_debug(simplex, v1, v2, search_direction) {
                return .{}, .{};
            }
        }

        b1 := simplex1.a;
        c1 := simplex1.b;
        d1 := simplex1.c;
        b2 := simplex2.a;
        c2 := simplex2.b;
        d2 := simplex2.c;
        b := simplex.a;
        c := simplex.b;
        d := simplex.c;
        bcd := cross(c - b, d - b);

        if dot(bcd, bcd) < EPSILON {
            // this means the triangle is basically a line, so we'll return whatever we've found so far because if this happens presumably we're at the end
            return -furthest_direction * direction, get_best_normal(furthest_simplex1, furthest_simplex2, dir);
        }

        normal := normalize(bcd);

        if dot(bcd, direction) < EPSILON {
            // the triangle is vertically aligned with desired direction
            // or the triangle is facing the wrong way
            return -furthest_direction * direction, get_best_normal(furthest_simplex1, furthest_simplex2, dir);
        }

        search_direction = bcd;
        
        #if GJK_DIR_DEBUG {
            if step_debug(simplex, v1, v2, search_direction) {
                return .{}, .{};
            }
        }
        a1 := get_support(v1, -search_direction, transform1);
        a2 := get_support(v2, search_direction, transform2);
        a := a2 - a1;
        
        #if GJK_DIR_DEBUG {
            if step_debug(simplex, v1, v2, search_direction) {
                draw_cube(a - .{0.02, 0.02, 0.02}, a + .{0.02, 0.02, 0.02}, color = .{1.0, 1.0, 1.0, 1.0});

                draw_triangle(b, c, a, color = .{0.5, 0.2, 0.8, 0.2});
                draw_triangle(c, d, a, color = .{0.5, 0.2, 0.8, 0.2});
                draw_triangle(d, b, a, color = .{0.5, 0.2, 0.8, 0.2});
                draw_line(b, a, color = .{1, 1, 1, 1});
                draw_line(c, a, color = .{1, 1, 1, 1});
                draw_line(d, a, color = .{1, 1, 1, 1});
                return .{}, .{};
            }
        }

        // if the point in front of the triangle is about on the plane of the triangle, return the triangle
        normal_dist := dot(normal, a) - dot(normal, b);
        if abs(normal_dist) < EPSILON {
            #if GJK_DIR_DEBUG {
                step_debug(simplex, v1, v2, search_direction, true);
            }

            return -furthest_direction * direction, get_best_normal(furthest_simplex1, furthest_simplex2, dir);
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
            simplex1.a = a1;
            simplex1.b = b1;
            simplex1.c = c1;
            simplex2.a = a2;
            simplex2.b = b2;
            simplex2.c = c2;
            simplex.a = a;
            simplex.b = b;
            simplex.c = c;

            furthest_direction = dot(normal_abc, a) / dot(normal_abc, direction);
            furthest_simplex1 = simplex1;
            furthest_simplex2 = simplex2;
        } else if dot(normal_acd, direction) > 0 && dot(cross(a - c, normal_acd), projected_acd - c) >= -EPSILON && dot(cross(ad, normal_acd), projected_acd - d) >= -EPSILON {
            // were in triangle acd
            simplex1.a = a1;
            simplex1.b = c1;
            simplex1.c = d1;
            simplex2.a = a2;
            simplex2.b = c2;
            simplex2.c = d2;
            simplex.a = a;
            simplex.b = c;
            simplex.c = d;

            furthest_direction = dot(normal_acd, a) / dot(normal_acd, direction);
            furthest_simplex1 = simplex1;
            furthest_simplex2 = simplex2;
        } else if dot(normal_adb, direction) > 0 && dot(cross(a - d, normal_adb), projected_adb - d) >= -EPSILON && dot(cross(ab, normal_adb), projected_adb - b) >= -EPSILON {
            // were in triangle adb
            simplex1.a = a1;
            simplex1.b = d1;
            simplex1.c = b1;
            simplex2.a = a2;
            simplex2.b = d2;
            simplex2.c = b2;
            simplex.a = a;
            simplex.b = d;
            simplex.c = b;

            furthest_direction = dot(normal_adb, a) / dot(normal_adb, direction);
            furthest_simplex1 = simplex1;
            furthest_simplex2 = simplex2;
        } else {
            // here I don't know what's happening so return the furthest triangle
            return -furthest_direction * direction, get_best_normal(furthest_simplex1, furthest_simplex2, dir);
        }
    }

    return -furthest_direction * direction, get_best_normal(furthest_simplex1, furthest_simplex2, dir);
}

#scope_file

get_best_normal :: (s1: Simplex, s2: Simplex, direction: Vector3) -> Vector3 {
    // get the most restricting normal of the two simplexes
    // a point normal is just direction
    // a line normal is the orthogonal to the line towards direction
    // a plane normal is the plane normal

    remove_duplicate_entries :: (simplex: *Simplex) {
        i := 0;
        while i < simplex.count {
            defer i += 1;

            a := i + 1;
            while a < simplex.count {
                defer a += 1;

                if simplex.s[i] == simplex.s[a] {
                    b := a;
                    while b + 1 < simplex.count {
                        defer b += 1;

                        simplex.s[b] = simplex.s[b + 1];
                    }

                    simplex.count -= 1;
                }
            }
        }
    }

    get_simplex_normal :: (simplex: Simplex, direction: Vector3) -> Vector3 {
        if simplex.count == {
            case 1;
                return normalize(direction);
            case 2;
                ab := simplex.b - simplex.a;
                return normalize(cross(cross(ab, direction), ab));
            case 3;
                abc := cross(simplex.b - simplex.a, simplex.c - simplex.a);
                return normalize(abc);
            case;
                print("Simplex has invalid number of vertices.\n");
                return .{};
        }
    }

    simplex1 := s1;
    simplex2 := s2;

    remove_duplicate_entries(*simplex1);
    remove_duplicate_entries(*simplex2);

    if simplex1.count >= simplex2.count {
        return -get_simplex_normal(simplex1, -direction);
    } else {
        return -get_simplex_normal(simplex2, -direction);
    }
}



// debug

GJK_DIR_DEBUG :: false;

gjk_dir_debug_points: [..] Vector3;
gjk_dir_debug_steps: int;
gjk_dir_debug_max_steps: int;

step_debug :: (simplex: Simplex, v1: KodaVertices($N1), v2: KodaVertices($N2), direction: Vector3, force_draw: bool = false) -> bool {
    if gjk_dir_debug_steps >= gjk_dir_debug_max_steps || force_draw {
        #import "KodaJai";

        // print_stack_trace();

        draw_text(tprint("Directional Step: %", gjk_dir_debug_steps), .{400, 80});

        depth_test := get_depth_test();
        defer set_depth_test(depth_test);
        set_depth_test(false);
        // cull_face := get_cull_face();
        // defer set_cull_face(cull_face);
        // set_cull_face(false);

        array_resize(*gjk_dir_debug_points, v1.count * v2.count);
        for i: 0..v1.count - 1 {
            for j: 0..v2.count - 1 {
                point := v2[j] - v1[i];
                gjk_dir_debug_points[i * v2.count + j] = point;
            }
        }

        draw_line(.{0, 0.1, 0}, -direction * 1.0 + .{0, 0.1, 0}, width = 2.0, color = .{1.0, 1.0, 1.0, 1.0});

        draw_spheres(gjk_dir_debug_points, .{0.2, 0.2, 0.2}, color = .{0.8, 0.8, 0.8, 0.1});

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

    gjk_dir_debug_steps += 1;

    return false;
}

#scope_export
