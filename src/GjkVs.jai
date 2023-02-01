
// solve_gjk_complete :: (v1: KodaVertices, v2: KodaVertices) -> ComplexSimplex {
//     possible_a: KodaVertices;
//     possible_b: KodaVertices;
//     possible_a_neg_norm: KodaVertices;
//     possible_b_neg_norm: KodaVertices;

//     // positive direction
//     for ai: 0..a.count / 3 - 1 {
//         v1 := a[ai * 3];
//         v2 := a[ai * 3 + 1];
//         v3 := a[ai * 3 + 2];

//         norm := cross(v2 - v1, v3 - v1);
//         if dot(norm, direction) > 0 {
//             add(*possible_a, v1, v2, v3);
//             add(*possible_a_neg_norm, -norm);
//         }
//     }
    
//     // negative direction
//     for bi: 0..b.count / 3 - 1 {
//         v1 := b[bi * 3];
//         v2 := b[bi * 3 + 1];
//         v3 := b[bi * 3 + 2];

//         norm := cross(v2 - v1, v3 - v1);
//         if dot(norm, direction) < 0 {
//             add(*possible_b, v1, v2, v3);
//             add(*possible_b_neg_norm, -norm);
//         }
//     }

//     // {
//     //     #import "KodaJai";
//     //     depth_test := get_depth_test();
//     //     defer set_depth_test(depth_test);
//     //     set_depth_test(false);
        
//     //     draw_triangles(get_view(*possible_a));
//     //     draw_triangles(get_view(*possible_b));
//     // }

//     // at this point, possible vertices are only faces that are not parallel and are correct facing
//     maximum_exit_length: float;
//     maximum_exit_vector: Vector3;
//     maximum_exit_point: Vector3;

//     for bi: 0..b.count - 1 {
//         bv := b[bi];
            
//         for ai: 0..possible_a_neg_norm.count - 1 {
//             neg_normal := possible_a_neg_norm[ai];
//             v1 := possible_a[ai * 3];

//             if dot(neg_normal, bv - v1) > 0 {
//                 v2 := possible_a[ai * 3 + 1];
//                 v3 := possible_a[ai * 3 + 2];

//                 inside_volume := true;
//                 inside_volume = inside_volume && dot(cross(v2 - v1, -direction), bv - v1) > 0;
//                 inside_volume = inside_volume && dot(cross(v3 - v2, -direction), bv - v2) > 0;
//                 inside_volume = inside_volume && dot(cross(v1 - v3, -direction), bv - v3) > 0;
//                 // inside_volume = inside_volume && solve_gjk(a, bv, direction);
//                 if inside_volume {
//                     triangle_plane := create_plane3(v1, neg_normal);
//                     ray := create_ray3(bv, direction);

//                     t := (triangle_plane.d - dot(triangle_plane.n, ray.p)) / dot(triangle_plane.n, ray.dir);
//                     assert(t >= 0, "I didn't think there could be a negative t value.");
//                     // p := ray.p + t * ray.dir;

//                     if t > maximum_exit_length {
//                         maximum_exit_length = t;
//                         maximum_exit_vector = t * ray.dir;
//                         maximum_exit_point = bv;
//                     }
//                 }
//             }
//         }
//     }
    
//     for ai: 0..a.count - 1 {
//         av := a[ai];
            
//         for bi: 0..possible_b_neg_norm.count - 1 {
//             neg_normal := possible_b_neg_norm[bi];
//             v1 := possible_b[bi * 3];

//             if dot(neg_normal, av - v1) > 0 {
//                 v2 := possible_b[bi * 3 + 1];
//                 v3 := possible_b[bi * 3 + 2];

//                 inside_volume := true;
//                 inside_volume = inside_volume && dot(cross(v2 - v1, direction), av - v1) > 0;
//                 inside_volume = inside_volume && dot(cross(v3 - v2, direction), av - v2) > 0;
//                 inside_volume = inside_volume && dot(cross(v1 - v3, direction), av - v3) > 0;
//                 // inside_volume = inside_volume && solve_gjk(b, av, -direction);
//                 if inside_volume {
//                     triangle_plane := create_plane3(v1, neg_normal);
//                     ray := create_ray3(av, -direction);

//                     t := (triangle_plane.d - dot(triangle_plane.n, ray.p)) / dot(triangle_plane.n, ray.dir);
//                     assert(t >= 0, "I didn't think there could be a negative t value.");
//                     // p := ray.p + t * ray.dir;

//                     if t > maximum_exit_length {
//                         maximum_exit_length = t;
//                         maximum_exit_vector = -t * ray.dir;
//                         maximum_exit_point = av - maximum_exit_vector;
//                     }
//                 }
//             }
//         }
//     }

//     if maximum_exit_length > 0 {
//         return maximum_exit_point, maximum_exit_vector, true;
//     }

//     return .{}, .{}, false;
// }