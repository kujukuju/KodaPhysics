
ComplexSimplex :: struct {
    a: Vector3;
    a1: Vector3;
    a2: Vector3;

    b: Vector3;
    b1: Vector3;
    b2: Vector3;

    c: Vector3;
    c1: Vector3;
    c2: Vector3;

    d: Vector3;
    d1: Vector3;
    d2: Vector3;

    count: int;

#place a;
    a_all: [3] Vector3 = ---;

#place b;
    b_all: [3] Vector3 = ---;

#place c;
    c_all: [3] Vector3 = ---;

#place d;
    d_all: [3] Vector3 = ---;
}

// returns whether or not the shapes intersect, the simplex, and the distance between shapes if allowed to resolve completely
solve_gjk :: (v1: KodaVertices, v2: KodaVertices, $resolve_completely: bool = false) -> bool, ComplexSimplex, Vector3, Vector3 {
    simplex: ComplexSimplex;

    direction := v1[0] - v2[0];
    simplex.a1 = get_support(v1, -direction);
    simplex.a2 = get_support(v2, direction);
    simplex.a = simplex.a2 - simplex.a1;
    simplex.count = 1;

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

    while true {
        #if GJK_DEBUG {
            if step_debug(simplex, v1, v2, .{0, 1000, 0}, .{0, 1000, 0}) {
                print("Exiting % %\n", gjk_debug_steps, gjk_debug_max_steps);
                return false, simplex, .{}, .{};
            }
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
                return false, simplex, .{}, .{};
        }

        // checking if distance squared from p to 0 is less than epsilon rather than comparing it to 0
        // because I'm not entirely sure how it would work together with the tetrahedron cross product
        // check and the support point directional check below
        if dot(p, p) < EPSILON {
            #if GJK_DEBUG {
                step_debug(simplex, v1, v2, p, .{0, 1000, 0}, true);
            }
            return true, simplex, .{}, .{};
        }

        direction = -p;

        support1 := get_support(v1, -direction);
        support2 := get_support(v2, direction);
        support := support2 - support1;
        
        #if !resolve_completely {
            // if we didn't find a point on the other side of the origin, it's not intersecting, so just exit
            // but we'll skip this step if we want the algorithm to finish so that we can use the nearest feature information
            if dot(support, direction) < 0 {
                return false, simplex, .{}, .{};
            }
        }

        #if GJK_DEBUG {
            if step_debug(simplex, v1, v2, p, support) {
                return false, simplex, .{}, .{};
            }
        }

        // if p is more than or equivalently extremal in the direction of the origin, we can't find a better support point
        if dot(p, direction) >= dot(support, direction) - EPSILON {
            #if GJK_DEBUG {
                step_debug(simplex, v1, v2, p, support, true);
            }
            closest1, closest2 := get_local_points(simplex, p);
            return false, simplex, closest1, closest2;
        }

        assert(simplex.count < 4, "You cannot have a simplex that's a tetrahedron at this point.");

        simplex.d_all = simplex.c_all;
        simplex.c_all = simplex.b_all;
        simplex.b_all = simplex.a_all;
        simplex.count += 1;

        simplex.a1 = support1;
        simplex.a2 = support2;
        simplex.a = support;
    }

    return false, simplex, .{}, .{};
}

#scope_file

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
            simplex.b_all = simplex.c_all;
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
                simplex.b_all, simplex.c_all = swap(simplex.b_all, simplex.c_all);
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

    abc_dir := same_direction(abc, ao);
    acd_dir := same_direction(acd, ao);
    adb_dir := same_direction(adb, ao);

    if !abc_dir && !acd_dir && !adb_dir {
        // the origin is inside the simplex
        return .{0, 0, 0};
    }

    if abc_dir && !acd_dir && !adb_dir {
        // the origin falls somewhere near abc
        simplex.count = 3;
        return solve_simplex3(simplex);
    }

    if !abc_dir && acd_dir && !adb_dir {
        // the origin falls somewhere near acd
        simplex.b_all = simplex.c_all;
        simplex.c_all = simplex.d_all;
        simplex.count = 3;
        return solve_simplex3(simplex);
    }

    if !abc_dir && !acd_dir && adb_dir {
        // the origin falls somewhere near adb
        simplex.c_all = simplex.b_all;
        simplex.b_all = simplex.d_all;
        simplex.count = 3;
        return solve_simplex3(simplex);
    }

    // the origin potentially falls on multiple triangles
    simplex_abc := << simplex;
    simplex_abc.count = 3;

    simplex_acd := << simplex;
    simplex_acd.b_all = simplex_acd.c_all;
    simplex_acd.c_all = simplex_acd.d_all;
    simplex_acd.count = 3;

    simplex_adb := << simplex;
    simplex_adb.c_all = simplex_adb.b_all;
    simplex_adb.b_all = simplex_adb.d_all;
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

get_local_points :: (simplex: ComplexSimplex, p: Vector3) -> Vector3, Vector3 {
    if simplex.count == {
        case 1;
            return simplex.a1, simplex.a2;

        case 2;
            barycentric := convert_barycentric(simplex.a, simplex.b, p);
            cartesian1 := convert_cartesian(simplex.a1, simplex.b1, barycentric);
            cartesian2 := convert_cartesian(simplex.a2, simplex.b2, barycentric);
            return cartesian1, cartesian2;

        case 3;
            barycentric := convert_barycentric(simplex.a, simplex.b, simplex.c, p);
            cartesian1 := convert_cartesian(simplex.a1, simplex.b1, simplex.c1, barycentric);
            cartesian2 := convert_cartesian(simplex.a2, simplex.b2, simplex.c2, barycentric);
            return cartesian1, cartesian2;

        case;
            assert(false, "Invalid simplex dimension for nearest feature resolution.");
            return .{}, .{};
    }
}

convert_barycentric :: (a: Vector3, b: Vector3, c: Vector3, p: Vector3) -> Vector3 {
    triangle_area_2d :: (x1: float, y1: float, x2: float, y2: float, x3: float, y3: float) -> float {
        return (x1 - x2) * (y2 - y3) - (x2 - x3) * (y1 - y2);
    }

    abc := cross(b - a, c - a);

    nu: float;
    nv: float;
    ood: float;

    x := abs(abc.x);
    y := abs(abc.y);
    z := abs(abc.z);

    if x >= y && x >= z {
        // x is the largest so project onto the yz plane
        nu = triangle_area_2d(p.y, p.z, b.y, b.z, c.y, c.z);
        nv = triangle_area_2d(p.y, p.z, c.y, c.z, a.y, a.z);
        ood = 1.0 / abc.x;
    } else if y >= x && y >= z {
        // y is the largest so project onto the xz plane
        nu = triangle_area_2d(p.x, p.z, b.x, b.z, c.x, c.z);
        nv = triangle_area_2d(p.x, p.z, c.x, c.z, a.x, a.z);
        ood = 1.0 / -abc.y;
    } else {
        // z is the largest so project onto the xy plane
        nu = triangle_area_2d(p.x, p.y, b.x, b.y, c.x, c.y);
        nv = triangle_area_2d(p.x, p.y, c.x, c.y, a.x, a.y);
        ood = 1.0 / abc.z;
    }

    result: Vector3 = ---;
    result.x = nu * ood;
    result.y = nv * ood;
    result.z = 1.0 - result.x - result.y;
    return result;
}

convert_cartesian :: (a: Vector3, b: Vector3, c: Vector3, barycentric: Vector3) -> Vector3 {
    result: Vector3 = ---;
    result.x = a.x * barycentric.x + b.x * barycentric.y + c.x * barycentric.z;
    result.y = a.y * barycentric.x + b.y * barycentric.y + c.y * barycentric.z;
    result.z = a.z * barycentric.x + b.z * barycentric.y + c.z * barycentric.z;
    return result;
}

convert_barycentric :: (a: Vector3, b: Vector3, p: Vector3) -> float {
    ab := b - a;
    ap := p - a;

    t := dot(ap, ab) / dot(ab, ab);
    return t;
}

convert_cartesian :: (a: Vector3, b: Vector3, barycentric: float) -> Vector3 {
    result: Vector3 = ---;
    result.x = a.x + barycentric * (b.x - a.x);
    result.y = a.y + barycentric * (b.y - a.y);
    result.z = a.z + barycentric * (b.z - a.z);
    return result;
}



// debug

EPSILON :: 0.000001;
GJK_DEBUG :: true;

gjk_debug_points: [..] Vector3;
gjk_debug_steps: int;
gjk_debug_max_steps: int;

step_debug :: (simplex: ComplexSimplex, v1: KodaVertices, v2: KodaVertices, p: Vector3, support: Vector3, force_draw: bool = false) -> bool {
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