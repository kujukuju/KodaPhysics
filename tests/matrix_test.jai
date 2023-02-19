main :: () {
    matrix: [..] [3] float;
    array_add(*matrix, Vector3.{1, 2, 1}.component);
    array_add(*matrix, Vector3.{0, -1, -1}.component);
    array_add(*matrix, Vector3.{-3, 0, -2}.component);
    array_add(*matrix, Vector3.{1, 0.5, 3.5}.component);

    b: [..] float;
    array_add(*b, 3, 2, -1, 4);

    solve_minimal_gaussian_elimination(matrix, b);
}

#import "Basic";
#import "Math";
#import "KodaPhysics";