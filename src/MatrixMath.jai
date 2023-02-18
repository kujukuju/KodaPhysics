
/*
 when you end up with a system of equations like
 x = 3
 y + 2z = 4
 then the answer is x is 3
 y, z = unit(1, 2) * 4 * 1, unit(1, 2) * 4 * 2

 if you have 0 = z, then z can be anything, so just set z to 0

 if you have x0 y0 z0 = something non 0, you have no solution
 */

Matrix :: struct {
    values: [..] float;
    width: int;
}

operator [] :: (m: Matrix, index: int) -> float {
    return m.values[index];
}

operator *[] :: (m: *Matrix, index: int) -> *float {
    return m.values.data + index;
}

resize :: (m: *Matrix, rows: int, columns: int) {
    array_resize(*m.values, width * height);
    m.width = width;
}

index :: (matrix: Matrix, row: int, column: int) -> int {
    return matrix.width * row + column;
}

vector_inner_product :: (a: [..] float, b: [..] float) -> float {
    ab := 0.0;
    for i: 0..a.count - 1 {
        ab += a[i] * b[i];
    }
    return ab;
}

solve_minimal_gaussian_elimination :: (m: [..] [$n] float, b: [..] float) -> [n] float {
    EPSILON :: 0.000001;

    a: [..] [n + 1] float;
    array_resize(*a, m.count);
    for row: 0..m.count - 1 {
        for column: 0..n - 1 {
            a[row][column] = m[row][column];
        }

        a[row][n] = b[row];
    }

    // forward propagation gaussian elimination
    row := 0;
    column := 0;
    last_valid_row := 0;
    last_valid_column := 0;
    while row < a.count - 1 && column < n {
        largest_row := row;
        largest_value := abs(a[row][column]);

        // find the largest row for this column partial rotation or whatever
        for next_row: row + 1..a.count - 1 {
            current_value := abs(a[next_row][column]);

            if current_value > largest_value {
                largest_row = next_row;
                largest_value = current_value;
            }
        }

        // make this row the remaining one with the largest column value
        if row != largest_row {
            a[row], a[largest_row] = swap(a[row], a[largest_row]);
        }

        if largest_value < EPSILON {
            a[row][column] = 0;
            column += 1;
            continue;
        }

        last_valid_row = row;
        last_valid_column = column;

        for next_row: row + 1..a.count - 1 {
            // if the current column in the next row is basically 0 just set it to 0 and skip it
            if abs(a[next_row][column]) < EPSILON {
                a[next_row][column] = 0;
                continue;
            }

            // we'll explicitly set the next_row column value here to avoid any precision errors since this is the critical function
            normalize_mul := a[next_row][column] / a[row][column];
            a[next_row][column] = 0;

            // last column inclusive here
            for next_column: column + 1..n {
                cancelling_column_value := normalize_mul * a[row][next_column];

                next_column_value := a[next_row][next_column];
                next_column_value -= cancelling_column_value;
                if abs(next_column_value) < EPSILON {
                    next_column_value = 0;
                }

                a[next_row][next_column] = next_column_value;
            }
        }

        row += 1;
        column += 1;
    }

    if last_valid_row < a.count - 1 && last_valid_column < n - 1 {
        potential_diagonal := a[last_valid_row + 1][last_valid_column + 1];
        if abs(potential_diagonal) < EPSILON {
            a[last_valid_row + 1][last_valid_column + 1] = 0;
        } else {
            last_valid_row += 1;
            last_valid_column += 1;
        }
    }

    print("Forward reduced matrix:\n");
    for row: a {
        print("%\n", row);
    }
    print("\n");

    // at this point we've forward propagated, we want to check if there are any rows below here that are inherently no solutions
    // every column in the row has to be 0 while the solution to the column isn't 0
    no_solution := false;
    for next_row: last_valid_row + 1..a.count - 1 {
        if a[next_row][n] != 0 {
            no_solution_row := true;
            for columns: 0..n - 1 {
                if a[next_row][column] != 0 {
                    no_solution_row = false;
                    break;
                }
            }

            if no_solution_row {
                no_solution = true;
                break;
            }
        }
    }

    if no_solution {
        print("No solution.\n");
        // solve using least squares to minimize ax-b
    }

    // at this point we have not yet found a no solution matrix so we continue with backwards propagation
    row = last_valid_row;
    column = last_valid_column;
    while row >= 1 && column >= 1 {
        // not checking epsilon here because the implication is if theres a value then it's part of the weird diagonal
        // so if we were to find an epsilon, we'd have to also move back a row I guess and other weird stuff
        if a[row][column] == 0 {
            column -= 1;
            continue;
        }

        previous_row := row - 1;
        while previous_row >= 0 {
            // if the current column in the next row is basically 0 just set it to 0 and skip it
            if abs(a[previous_row][column]) < EPSILON {
                a[previous_row][column] = 0;
                previous_row -= 1;
                continue;
            }

            // we'll explicitly set the next_row column value here to avoid any precision errors since this is the critical function
            normalize_mul := a[previous_row][column] / a[row][column];
            a[previous_row][column] = 0;

            // last column inclusive here
            previous_column := column - 1;
            while previous_column >= 0 {
                cancelling_column_value := normalize_mul * a[row][previous_column];

                previous_column_value := a[previous_row][previous_column];
                previous_column_value -= cancelling_column_value;
                if abs(previous_column_value) < EPSILON {
                    previous_column_value = 0;
                }

                a[previous_row][previous_column] = previous_column_value;

                previous_column -= 1;
            }

            // explicitly cancel out the b vector since we're not going in that direction
            cancelling_column_value := normalize_mul * a[row][n];

            previous_column_value := a[previous_row][n];
            previous_column_value -= cancelling_column_value;
            if abs(previous_column_value) < EPSILON {
                previous_column_value = 0;
            }
            a[previous_row][n] = previous_column_value;

            previous_row -= 1;
        }

        row -= 1;
        column -= 1;
    }

    // now the matrix has been reduced
    print("Reduced matrix:\n");
    for row: a {
        print("%\n", row);
    }
    print("\n");
}

// singular value decomposition
// calculate_svd :: (a: [] [$n] float) -> u: [][n] float, v: [n][n] float, w: [n] float {
//     u := NewArray(a.count, [n] float);
//     v: [n][n] float;
//     w: [n] float;

//     mOrN 

//   const mOrN = (withu === 'f') ? m : n

//   // Initialize u
//   for (i = 0; i < m; i++) {
//     u[i] = new Array(mOrN).fill(0)
//   }

//   // Initialize v
//   for (i = 0; i < n; i++) {
//     v[i] = new Array(n).fill(0)
//   }

//   // Initialize q
//   const q = new Array(n).fill(0)

//   // Copy array a in u
//   for (i = 0; i < m; i++) {
//     for (j = 0; j < n; j++) {
//       u[i][j] = a[i][j]
//     }
//   }

//   for (i = 0; i < n; i++) {
//     e[i] = g
//     s = 0
//     l = i + 1
//     for (j = i; j < m; j++) {
//       s += Math.pow(u[j][i], 2)
//     }
//     if (s < tol) {
//       g = 0
//     } else {
//       f = u[i][i]
//       g = f < 0 ? Math.sqrt(s) : -Math.sqrt(s)
//       h = f * g - s
//       u[i][i] = f - g
//       for (j = l; j < n; j++) {
//         s = 0
//         for (k = i; k < m; k++) {
//           s += u[k][i] * u[k][j]
//         }
//         f = s / h
//         for (k = i; k < m; k++) {
//           u[k][j] = u[k][j] + f * u[k][i]
//         }
//       }
//     }
//     q[i] = g
//     s = 0
//     for (j = l; j < n; j++) {
//       s += Math.pow(u[i][j], 2)
//     }
//     if (s < tol) {
//       g = 0
//     } else {
//       f = u[i][i + 1]
//       g = f < 0 ? Math.sqrt(s) : -Math.sqrt(s)
//       h = f * g - s
//       u[i][i + 1] = f - g
//       for (j = l; j < n; j++) {
//         e[j] = u[i][j] / h
//       }
//       for (j = l; j < m; j++) {
//         s = 0
//         for (k = l; k < n; k++) {
//           s += u[j][k] * u[i][k]
//         }
//         for (k = l; k < n; k++) {
//           u[j][k] = u[j][k] + s * e[k]
//         }
//       }
//     }
//     y = Math.abs(q[i]) + Math.abs(e[i])
//     if (y > x) {
//       x = y
//     }
//   }

//   // Accumulation of right-hand transformations
//   if (withv) {
//     for (i = n - 1; i >= 0; i--) {
//       if (g !== 0) {
//         h = u[i][i + 1] * g
//         for (j = l; j < n; j++) {
//           v[j][i] = u[i][j] / h
//         }
//         for (j = l; j < n; j++) {
//           s = 0
//           for (k = l; k < n; k++) {
//             s += u[i][k] * v[k][j]
//           }
//           for (k = l; k < n; k++) {
//             v[k][j] = v[k][j] + s * v[k][i]
//           }
//         }
//       }
//       for (j = l; j < n; j++) {
//         v[i][j] = 0
//         v[j][i] = 0
//       }
//       v[i][i] = 1
//       g = e[i]
//       l = i
//     }
//   }

//   // Accumulation of left-hand transformations
//   if (withu) {
//     if (withu === 'f') {
//       for (i = n; i < m; i++) {
//         for (j = n; j < m; j++) {
//           u[i][j] = 0
//         }
//         u[i][i] = 1
//       }
//     }
//     for (i = n - 1; i >= 0; i--) {
//       l = i + 1
//       g = q[i]
//       for (j = l; j < mOrN; j++) {
//         u[i][j] = 0
//       }
//       if (g !== 0) {
//         h = u[i][i] * g
//         for (j = l; j < mOrN; j++) {
//           s = 0
//           for (k = l; k < m; k++) {
//             s += u[k][i] * u[k][j]
//           }
//           f = s / h
//           for (k = i; k < m; k++) {
//             u[k][j] = u[k][j] + f * u[k][i]
//           }
//         }
//         for (j = i; j < m; j++) {
//           u[j][i] = u[j][i] / g
//         }
//       } else {
//         for (j = i; j < m; j++) {
//           u[j][i] = 0
//         }
//       }
//       u[i][i] = u[i][i] + 1
//     }
//   }

//   // Diagonalization of the bidiagonal form
//   eps = eps * x
//   let testConvergence
//   for (k = n - 1; k >= 0; k--) {
//     for (let iteration = 0; iteration < 50; iteration++) {
//       // test-f-splitting
//       testConvergence = false
//       for (l = k; l >= 0; l--) {
//         if (Math.abs(e[l]) <= eps) {
//           testConvergence = true
//           break
//         }
//         if (Math.abs(q[l - 1]) <= eps) {
//           break
//         }
//       }

//       if (!testConvergence) { // cancellation of e[l] if l>0
//         c = 0
//         s = 1
//         l1 = l - 1
//         for (i = l; i < k + 1; i++) {
//           f = s * e[i]
//           e[i] = c * e[i]
//           if (Math.abs(f) <= eps) {
//             break // goto test-f-convergence
//           }
//           g = q[i]
//           q[i] = Math.sqrt(f * f + g * g)
//           h = q[i]
//           c = g / h
//           s = -f / h
//           if (withu) {
//             for (j = 0; j < m; j++) {
//               y = u[j][l1]
//               z = u[j][i]
//               u[j][l1] = y * c + (z * s)
//               u[j][i] = -y * s + (z * c)
//             }
//           }
//         }
//       }

//       // test f convergence
//       z = q[k]
//       if (l === k) { // convergence
//         if (z < 0) {
//           // q[k] is made non-negative
//           q[k] = -z
//           if (withv) {
//             for (j = 0; j < n; j++) {
//               v[j][k] = -v[j][k]
//             }
//           }
//         }
//         break // break out of iteration loop and move on to next k value
//       }

//       // Shift from bottom 2x2 minor
//       x = q[l]
//       y = q[k - 1]
//       g = e[k - 1]
//       h = e[k]
//       f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2 * h * y)
//       g = Math.sqrt(f * f + 1)
//       f = ((x - z) * (x + z) + h * (y / (f < 0 ? (f - g) : (f + g)) - h)) / x

//       // Next QR transformation
//       c = 1
//       s = 1
//       for (i = l + 1; i < k + 1; i++) {
//         g = e[i]
//         y = q[i]
//         h = s * g
//         g = c * g
//         z = Math.sqrt(f * f + h * h)
//         e[i - 1] = z
//         c = f / z
//         s = h / z
//         f = x * c + g * s
//         g = -x * s + g * c
//         h = y * s
//         y = y * c
//         if (withv) {
//           for (j = 0; j < n; j++) {
//             x = v[j][i - 1]
//             z = v[j][i]
//             v[j][i - 1] = x * c + z * s
//             v[j][i] = -x * s + z * c
//           }
//         }
//         z = Math.sqrt(f * f + h * h)
//         q[i - 1] = z
//         c = f / z
//         s = h / z
//         f = c * g + s * y
//         x = -s * g + c * y
//         if (withu) {
//           for (j = 0; j < m; j++) {
//             y = u[j][i - 1]
//             z = u[j][i]
//             u[j][i - 1] = y * c + z * s
//             u[j][i] = -y * s + z * c
//           }
//         }
//       }
//       e[l] = 0
//       e[k] = f
//       q[k] = x
//     }
//   }

//   // Number below eps should be zero
//   for (i = 0; i < n; i++) {
//     if (q[i] < eps) q[i] = 0
//   }

//   return { u, q, v }
// }

// solve_least_squares_svd :: (a: [..] [$n] float, b: [..] float) -> [n] float {
//     index :: (row: int, column: int, columns: int) -> int {
//         return row * columns + column;
//     }

//     mul :: (m: [] float, v: [] float) -> [n] float {
//         columns := v.count;
//         rows :: n;
//         assert(m.count == rows * columns);

//         result: [n] float;
//         for row: 0..rows - 1 {
//             for column: 0..columns - 1 {
//                 result[row] += m[index(row, column, columns)] * v[column];
//             }
//         }

//         return result;
//     }

//     mul :: (a: [n][n] float, b: [n][n] float) -> [n][n] float {
//         result: [n][n] float;
//         for final_row: 0..n - 1 {
//             for final_column: 0..n - 1 {
//                 for i: 0..n - 1 {
//                     result[final_row][final_column] += a[final_row][i] * b[i][final_column];
//                 }
//             }
//         }

//         return result;
//     }

//     mul_transpose :: (a: [n][n] float, b: [][n] float, out: *[] float) {
//         // transpose b during the mul operation
//         rows :: n;
//         columns := b.count;
//         assert(out.count == rows * columns);

//         for final_row: 0..rows - 1 {
//             for final_column: 0..columns - 1 {
//                 for i: 0..n - 1 {
//                     out.data[index(final_row, final_column, columns)] += a[final_row][i] * b[final_column][i];
//                 }
//             }
//         }
//     }

//     mul :: (a: [][n] float, b: [n] float, out: *[] float) {
//         rows := a.count;
//         columns :: n;
//         assert(out.count == rows);

//         for row: 0..rows - 1 {
//             for column: 0..columns - 1 {
//                 out.data[row] += a[row][column] * b[column];
//             }
//         }
//     }

//     sub_norm :: (a: [] float, b: [] float) -> float {
//         // subtract a - b then calculate the euclidian norm of the result
//         result := 0.0;
//         for i: 0..a.count - 1 {
//             subbed := a[i] - b[i];
//             result += subbed * subbed;
//         }
//         return sqrt(result);
//     }

//     MAX_ITER :: 1000;
//     // MAX_ITER :: 40;
//     EPSILON :: 0.000001;

//     m := a.count;

//     u, v, w := calculate_svd(a);
//     defer array_free(u);

//     x: [n] float;
//     x_prev: [n] float;
//     error := 0.0;
//     error_prev := 0.0;

//     // this is n by m
//     final: [..] float;
//     defer array_free(final);
//     array_resize(*final, m * n);

//     b_temp: [..] float;
//     defer array_free(b_temp);
//     array_resize(*b_temp, b.count);

//     for i: 0..MAX_ITER - 1 {
//         diagonal: [n][n] float;
//         for di: 0..n - 1 {
//             diagonal[di][di] = w[di] / (w[di] * w[di] + error);
//         }
//         diagonal = mul(v, diagonal);
//         mul_transpose(diagonal, u, *final);
//         x_prev = x;
//         x = mul(final, b);
//         error_prev = error;
//         mul(a, x, *b_temp);
//         error = sub_norm(b_temp, b);

//         delta_error := abs(error - error_prev);
//         if delta_error < EPSILON {
//             break;
//         }
//     }

//     return x;
// }

// solve_gauss_seidel :: (matrix: [..] [$M] float, b: [..] float) -> [M] float {
//     MAX_ITERATIONS :: 100;
//     EPSILON :: 0.0001;
    
//     x: [M] float;
//     error := 0.0;

//     n := matrix.count;

//     // first, unlike all other partial swap methods, we'll sort our matrix such that the diagonal elements are smallest towards the top
//     // for row: 0..n - 1 {
//     //     smallest_index := row;
//     //     smallest_value := matrix[row][row];
//     // }

//     for iter: 0..MAX_ITERATIONS - 1 {
//         error = 0.0;

//         for i: 0..n - 1 {
//             xi := b[i];
            
//             for j: 0..M - 1 {
//                 if i != j {
//                     xi -= matrix[i][j] * x[j];
//                 }
//             }

//             xi /= matrix[i][i];
//             error += abs(x[i] - xi);
//             x[i] = xi;
//         }

//         if error < EPSILON {
//             break;
//         }
//     }

//     return x;
// }

// solve_gaussian_elim :: (matrix: Matrix, rhs: [] float) -> [..] float {
//     u: Matrix;
//     resize(*u, matrix.width, matrix.width + 1);

//     // TODO u shouldnt be necessary at all
//     for i: 0..matrix.width - 1 {
//         u[index(u, i, matrix.width)] = rhs[i];
//         for j: 0..matrix.width - 1 {
//             u[index(u, i, j)] = matrix[index(matrix, i, j)];
//         }
//     }
    
//     // factorisation stage
//     for k: 0..matrix.width - 1 {
//         // find the best pivot
//         p := k;
//         max_pivot := 0.0;
//         for i: k..matrix.width - 1 {
//             if abs(u[index(u, i, k)]) > max_pivot {
//                 max_pivot = abs(u[index(u, i, k)]);
//                 p = i;
//             }
//         }

//         // swap rows k and p
//         if p != k {
//             for i: k..matrix.width {
//                 a := u[index(u, p, i)];
//                 b := u[index(u, k, i)];
//                 u[index(u, p, i)] = b;
//                 u[index(u, k, i)] = a;
//             }
//         }

//         // elimination of variables
//         for i: k + 1..matrix.width - 1 {
//             if u[index(u, k, k)] == 0 {
//                 continue;
//             }

//             m := u[index(u, i, k)] / u[index(u, k, k)];
//             for j: k..matrix.width {
//                 u[index(u, i, j)] -= m * u[index(u, k, j)];
//             }
//         }
//     }

//     result: [..] float;
//     array_resize(*result, matrix.width);

//     // back substitution
//     k := matrix.width - 1;
//     while k >= 0 {
//         defer k -= 1;
//         if u[index(u, k, k)] == 0 {
//             continue;
//         }
        
//         sum := u[index(u, k, matrix.width)];
//         for j: k + 1..matrix.width - 1 {
//             sum -= u[index(u, k, j)] * result[j];
//         }

//         result[k] = sum / u[index(u, k, k)];
//     }

//     return result;
// }