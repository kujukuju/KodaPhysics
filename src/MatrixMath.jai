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

resize :: (m: *Matrix, width: int, height: int) {
    array_resize(*m.values, width * height);
    m.width = width;
}

index :: (matrix: Matrix, row: int, column: int) -> int {
    return matrix.width * row + column;
}

solve_gaussian_elim :: (matrix: Matrix, rhs: [] float) -> [..] float {
    u: Matrix;
    resize(*u, matrix.width, matrix.width + 1);

    // TODO u shouldnt be necessary at all
    for i: 0..matrix.width - 1 {
        u[index(u, i, matrix.width)] = rhs[i];
        for j: 0..matrix.width - 1 {
            u[index(u, i, j)] = matrix[index(matrix, i, j)];
        }
    }
    
    // factorisation stage
    for k: 0..matrix.width - 1 {
        // find the best pivot
        p := k;
        max_pivot := 0.0;
        for i: k..matrix.width - 1 {
            if abs(u[index(u, i, k)]) > max_pivot {
                max_pivot = abs(u[index(u, i, k)]);
                p = i;
            }
        }

        // swap rows k and p
        if p != k {
            for i: k..matrix.width {
                a := u[index(u, p, i)];
                b := u[index(u, k, i)];
                u[index(u, p, i)] = b;
                u[index(u, k, i)] = a;
            }
        }

        // elimination of variables
        for i: k + 1..matrix.width - 1 {
            if u[index(u, k, k)] == 0 {
                continue;
            }

            m := u[index(u, i, k)] / u[index(u, k, k)];
            for j: k..matrix.width {
                u[index(u, i, j)] -= m * u[index(u, k, j)];
            }
        }
    }

    result: [..] float;
    array_resize(*result, matrix.width);

    // back substitution
    k := matrix.width - 1;
    while k >= 0 {
        defer k -= 1;
        if u[index(u, k, k)] == 0 {
            continue;
        }
        
        sum := u[index(u, k, matrix.width)];
        for j: k + 1..matrix.width - 1 {
            sum -= u[index(u, k, j)] * result[j];
        }

        result[k] = sum / u[index(u, k, k)];
    }

    return result;
}