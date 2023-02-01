// // point convergence expansion

// // * NOTE: if I want to, I can choose a, b, or c, depending on which line the origin is most outside of
// // * if the origin projects outside ab, move c to be closer to the normal of the triangle abo
// //   * if the closer point is already claimed, check if the origin projects onto the line ab
// //   * if so, ab is the closest feature
// //   * if not, if a is closer to the origin then b, then a is the closest feature
// //   * if not, then b is the closest feature
// // * if the origin projects outside ac, move b to be closer to the normal of the triangle aco
// //   * if the closer point is already claimed, check if the origin projects onto the line ac
// //   * if so, ac is the closest feature
// //   * if not, if a is closer to the origin then c, then a is the closest feature
// //   * if not, then c is the closest feature
// // * if the origin projects outside bc, move a to be closer to the normal of the triangle bco
// //   * if the closer point is already claimed, check if the origin projects onto the line bc
// //   * if so, bc is the closest feature
// //   * if not, if b is closer to the origin then c, then b is the closest feature
// //   * if not, then c is the closest feature
// // * if the origin projects onto abc, move the furthest point from the projection closer to the normal of the triangle
// //   * if the closer point is already claimed, abc is the closest feature
// solve_pce :: (v1: KodaVertices, v2: KodaVertices) -> ComplexSimplex {
//     // choose a starting extreme triangle of the minkowski difference
//     // the starting triangle should be the 3 points that are closest to the direction of a - b (or b - a?) without being the same

//     // TODO check a case where the origin starts out parallel to the starting simplex triangle

//     simplex: ComplexSimplex;

//     direction := v1[0] - v2[0];
//     simplex.a1 = get_support(v1, -direction);
//     simplex.a2 = get_support(v2, direction);
//     simplex.a = simplex.a2 - simplex.a1;

//     direction = cross(direction, -simplex.a);

//     simplex.b1 = get_support(v1, -direction);
//     simplex.b2 = get_support(v2, direction);
//     simplex.b = simplex.b2 - simplex.b1;

// }

// ComplexSimplex :: struct {
//     a: Vector3;
//     b: Vector3;
//     c: Vector3;

//     a1: Vector3;
//     b1: Vector3;
//     c1: Vector3;

//     a2: Vector3;
//     b2: Vector3;
//     c2: Vector3;

//     count: int;

// #place a1;
//     s1: [3] Vector3 = ---;

// #place a2;
//     s2: [3] Vector3 = ---;
// }

// #scope_file

// // get_triangle_support :: (vertices: KodaVertices, direction: Vector3) -> Vector3 {

// // }

// get_support :: (vertices: KodaVertices, direction: Vector3) -> Vector3 {
//     largest_dot := dot(vertices[0], direction);
//     largest_vertex := vertices[0];
//     for i: 1..vertices.count - 1 {
//         vertex := vertices[i];

//         current_dot := dot(vertex, direction);
//         if current_dot > largest_dot {
//             largest_dot = current_dot;
//             largest_vertex = vertex;
//         }
//     }

//     return largest_vertex;
// }