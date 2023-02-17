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

// singular value decomposition
calculate_svd :: (a: *[] [$n] float) -> v: [n][n] float, w: [n] float {
    copysign :: (mag: float, sign: float) -> float {
        if sign > 0 {
            return abs(mag);
        } else if sign < 0 {
            return -abs(mag);
        }

        return mag;
    }

    hypot :: (a: float, b: float) -> float {
        return sqrt(a * a + b * b);
    }

    SVD_NMAX :: 40;
    EPSILON :: 0.000001;

    m := a.count;

    // u := a;
    w: [n] float;
    v: [n][n] float;

    rv1: *float;
    i := -1;
    j := -1;
    k := -1;
    l := -1;
    tst1: float;
    c: float;
    f: float;
    g: float;
    h: float;
    s: float;
    scale: float;
    
    assert(m > 0 && n > 0);

    rv1 = alloc(n * size_of(float));
    defer free(rv1);

    /*
     * householder reduction to bidiagonal form 
     */
    g = 0.0;
    scale = 0.0;
    tst1 = 0.0;
    for i: 0..n - 1 {
        l = i + 1;
        rv1[i] = scale * g;
        g = 0.0;
        s = 0.0;
        scale = 0.0;
        if i < m {
            for k: i..m - 1 {
                scale += abs(a.data[k][i]);
            }
            if scale != 0.0 {
                for k: i..m - 1 {
                    a.data[k][i] /= scale;
                    s += a.data[k][i] * a.data[k][i];
                }
                f = a.data[i][i];
                g = -copysign(sqrt(s), f);
                h = f * g - s;
                a.data[i][i] = f - g;
                if i < n - 1 {        /* no test in NR */
                    for j: l..n - 1 {
                        s = 0.0;
                        for k: i..m - 1 {
                            s += a.data[k][i] * a.data[k][j];
                        }
                        f = s / h;
                        for k: i..m - 1 {
                            a.data[k][j] += f * a.data[k][i];
                        }
                    }
                }
                for k: i..m - 1 {
                    a.data[k][i] *= scale;
                }
            }
        }
        w[i] = scale * g;
        g = 0.0;
        s = 0.0;
        scale = 0.0;
        if i < m && i < n - 1 {
            for k: l..n - 1 {
                scale += abs(a.data[i][k]);
            }
            if scale != 0.0 {
                for k: l..n - 1 {
                    a.data[i][k] /= scale;
                    s += a.data[i][k] * a.data[i][k];
                }
                f = a.data[i][l];
                g = -copysign(sqrt(s), f);
                h = f * g - s;
                a.data[i][l] = f - g;
                for k: l..n - 1 {
                    rv1[k] = a.data[i][k] / h;
                }
                for j: l..m - 1 {
                    s = 0.0;
                    for k: l..n - 1 {
                        s += a.data[j][k] * a.data[i][k];
                    }
                    for k: l..n - 1 {
                        a.data[j][k] += s * rv1[k];
                    }
                }
                for k: l..n - 1 {
                    a.data[i][k] *= scale;
                }
            }
        }
        {
            tmp := abs(w[i]) + abs(rv1[i]);
            tst1 = ifx tst1 > tmp then tst1 else tmp;
        }
    }

    /*
     * accumulation of right-hand transformations 
     */
    // TODO I think itd be faster, especially for something with so few entries like this, to use an actual for loop
    i = n - 1;
    while i >= 0 {
        defer i -= 1;

        if i < n - 1 {        /* no test in NR */
            if g != 0.0 {
                for j: l..n - 1 {
                    /*
                     * double division avoids possible underflow 
                     */
                    v[j][i] = (a.data[i][j] / a.data[i][l]) / g;
                }
                for j: l..n - 1 {
                    s = 0.0;
                    for k: l..n - 1 {
                        s += a.data[i][k] * v[k][j];
                    }
                    for k: l..n - 1 {
                        v[k][j] += s * v[k][i];
                    }
                }
            }
            for j: l..n - 1 {
                v[i][j] = 0.0;
                v[j][i] = 0.0;
            }
        }
        v[i][i] = 1.0;
        g = rv1[i];
        l = i;
    }

    /*
     * accumulation of left-hand transformations 
     */
    i = ifx m < n then m - 1 else n - 1;
    while i >= 0 {
        defer i -= 1;

        l = i + 1;
        g = w[i];
        if i != n - 1 {
            for j: l..n - 1 {
                a.data[i][j] = 0.0;
            }
        }
        if g != 0.0 {
            for j: l..n - 1 {
                s = 0.0;
                for k: l..m - 1 {
                    s += a.data[k][i] * a.data[k][j];
                }
                /*
                 * double division avoids possible underflow
                 */
                f = (s / a.data[i][i]) / g;
                for k: i..m - 1 {
                    a.data[k][j] += f * a.data[k][i];
                }
            }
            for j: i..m - 1 {
                a.data[j][i] /= g;
            }
        } else {
            for j: i..m - 1 {
                a.data[j][i] = 0.0;
            }
        }
        a.data[i][i] += 1.0;
    }

    /*
     * diagonalization of the bidiagonal form
     */
    k = n - 1;
    while k >= 0 {
        defer k -= 1;

        k1 := k - 1;
        its := 0;

        while true {
            docancellation := true;
            x: float;
            y: float;
            z: float;
            l1 := -1;

            its += 1;
            if its > SVD_NMAX {
                print("No convergence. %\n", << a);
                assert(false, "No convergence.");
            }

            l := k;
            while l >= 0 {  /* test for splitting */
                defer l -= 1;

                tst2 := abs(rv1[l]) + tst1;

                if tst2 == tst1 {
                    docancellation = false;
                    break;
                }
                l1 = l - 1;
                /*
                 * rv1(1) is always zero, so there is no exit through the
                 * bottom of the loop
                 */
                tst2 = abs(w[l - 1]) + tst1;
                if tst2 == tst1 {
                    break;
                }
            }
            /*
             * cancellation of rv1[l] if l > 1
             */
            if docancellation {
                c = 0.0;
                s = 1.0;
                for i: l..k {
                    f = s * rv1[i];
                    rv1[i] = c * rv1[i];
                    if abs(f) + tst1 == tst1 {
                        break;
                    }
                    g = w[i];
                    h = hypot(f, g);
                    w[i] = h;
                    c = g / h;
                    s = -f / h;
                    for j: 0..m - 1 {
                        y := a.data[j][l1];
                        z := a.data[j][i];

                        a.data[j][l1] = y * c + z * s;
                        a.data[j][i] = z * c - y * s;
                    }
                }
            }
            
            /*
             * test for convergence
             */
            z = w[k];
            if l != k {
                i1: int;

                /*
                 * shift from bottom 2 by 2 minor
                 */
                x = w[l];
                y = w[k1];
                g = rv1[k1];
                h = rv1[k];
                f = 0.5 * (((g + z) / h) * ((g - z) / y) + y / h - h / y);
                g = hypot(f, 1.0);
                f = x - (z / x) * z + (h / x) * (y / (f + copysign(g, f)) - h);
                /*
                 * next qr transformation 
                 */
                c = 1.0;
                s = 1.0;
                for i1: l..k - 1 {
                    i = i1 + 1;
                    g = rv1[i];
                    y = w[i];
                    h = s * g;
                    g = c * g;
                    z = hypot(f, h);
                    rv1[i1] = z;
                    c = f / z;
                    s = h / z;
                    f = x * c + g * s;
                    g = g * c - x * s;
                    h = y * s;
                    y *= c;
                    for j: 0..n - 1 {
                        x = v[j][i1];
                        z = v[j][i];
                        v[j][i1] = x * c + z * s;
                        v[j][i] = z * c - x * s;
                    }
                    z = hypot(f, h);
                    w[i1] = z;
                    /*
                     * rotation can be arbitrary if z = 0
                     */
                    if z != 0.0 {
                        c = f / z;
                        s = h / z;
                    }
                    f = c * g + s * y;
                    x = c * y - s * g;
                    for j: 0..m - 1 {
                        y = a.data[j][i1];
                        z = a.data[j][i];
                        a.data[j][i1] = y * c + z * s;
                        a.data[j][i] = z * c - y * s;
                    }
                }
                rv1[l] = 0.0;
                rv1[k] = f;
                w[k] = x;
            } else {
                /*
                 * w[k] is made non-negative
                 */
                if z < 0.0 {
                    w[k] = -z;
                    for j: 0..n - 1 {
                        v[j][k] = -v[j][k];
                    }
                }
                break;
            }
        }
    }

    return v, w;
}

solve_least_squares_svd :: (a: [..] [$n] float, b: [..] float) -> [n] float {
    index :: (row: int, column: int, columns: int) -> int {
        return row * columns + column;
    }

    mul :: (m: [] float, v: [] float) -> [n] float {
        columns := v.count;
        rows :: n;
        assert(m.count == rows * columns);

        result: [n] float;
        for row: 0..rows - 1 {
            for column: 0..columns - 1 {
                result[row] += m[index(row, column, columns)] * v[column];
            }
        }

        return result;
    }

    mul :: (a: [n][n] float, b: [n][n] float) -> [n][n] float {
        result: [n][n] float;
        for final_row: 0..n - 1 {
            for final_column: 0..n - 1 {
                for i: 0..n - 1 {
                    result[final_row][final_column] += a[final_row][i] * b[i][final_column];
                }
            }
        }

        return result;
    }

    mul_transpose :: (a: [n][n] float, b: [][n] float, out: *[] float) {
        // transpose b during the mul operation
        rows :: n;
        columns := b.count;
        assert(out.count == rows * columns);

        for final_row: 0..rows - 1 {
            for final_column: 0..columns - 1 {
                for i: 0..n - 1 {
                    out.data[index(final_row, final_column, columns)] += a[final_row][i] * b[final_column][i];
                }
            }
        }
    }

    mul :: (a: [][n] float, b: [n] float, out: *[] float) {
        rows := a.count;
        columns :: n;
        assert(out.count == rows);

        for row: 0..rows - 1 {
            for column: 0..columns - 1 {
                out.data[row] += a[row][column] * b[column];
            }
        }
    }

    sub_norm :: (a: [] float, b: [] float) -> float {
        // subtract a - b then calculate the euclidian norm of the result
        result := 0.0;
        for i: 0..a.count - 1 {
            subbed := a[i] - b[i];
            result += subbed * subbed;
        }
        return sqrt(result);
    }

    MAX_ITER :: 1000;
    // MAX_ITER :: 40;
    EPSILON :: 0.000001;

    m := a.count;

    u := array_copy(a);
    defer array_free(u);
    v, w := calculate_svd(*u);

    x: [n] float;
    x_prev: [n] float;
    error := 0.0;
    error_prev := 0.0;

    // this is n by m
    final: [..] float;
    defer array_free(final);
    array_resize(*final, m * n);

    b_temp: [..] float;
    defer array_free(b_temp);
    array_resize(*b_temp, b.count);

    for i: 0..MAX_ITER - 1 {
        diagonal: [n][n] float;
        for di: 0..n - 1 {
            diagonal[di][di] = w[di] / (w[di] * w[di] + error);
        }
        diagonal = mul(v, diagonal);
        mul_transpose(diagonal, u, *final);
        x_prev = x;
        x = mul(final, b);
        error_prev = error;
        mul(a, x, *b_temp);
        error = sub_norm(b_temp, b);

        delta_error := abs(error - error_prev);
        if delta_error < EPSILON {
            break;
        }
    }

    return x;
}

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