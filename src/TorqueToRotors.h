
#include <limits>

namespace Detail
{
    float constexpr sqrtNewtonRaphson(const float x, const float curr, const float prev)
    {
        return curr == prev
            ? curr
            : sqrtNewtonRaphson(x, 0.5f * (curr + x / curr), curr);
    }
}

/*
* Constexpr version of the square root
* Return value:
*   - For a finite and non-negative value of "x", returns an approximation for the square root of "x"
*   - Otherwise, returns NaN
*/
float constexpr sqrtConstExpr(const float x)
{
    return x >= 0 && x < std::numeric_limits<float>::infinity()
        ? Detail::sqrtNewtonRaphson(x, x, 0)
        : std::numeric_limits<float>::quiet_NaN();
}

#include <array>

constexpr float determinant3x3(const std::array<std::array<float,3>,3> &mat) {
    return mat[0][0] * (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1]) -
        mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) +
        mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
}

// Function to calculate the cofactor of a 4x4 matrix
constexpr std::array<std::array<float,4>,4> cofactor4x4(const std::array<const std::array<float, 4>, 4> &mat) {
    std::array<std::array<float,3>,3> submat;
    std::array<std::array<float,4>,4> cofactors;

    for (unsigned int row = 0; row < 4; ++row) {
        for (unsigned int col = 0; col < 4; ++col) {
            // Build the 3x3 submatrix for element (row, col)
            unsigned int subi = 0;
            for (unsigned int i = 0; i < 4; ++i) {
                if (i == row) continue;
                int subj = 0;
                for (int j = 0; j < 4; ++j) {
                    if (j == col) continue;
                    submat[subi][subj] = mat[i][j];
                    subj++;
                }
                subi++;
            }
            cofactors[row][col] = ((row + col) % 2 == 0 ? 1 : -1) * determinant3x3(submat);
        }
    }
    return cofactors;
}

// Function to calculate the determinant of a 4x4 matrix
constexpr float determinant4x4(const std::array<const std::array<float, 4>, 4> &mat) {
    const std::array<std::array<float, 4>, 4> cofactors{ cofactor4x4(mat) };
    float det = 0.0;
    
	

    for (unsigned int col = 0; col < 4; ++col) {
        det += mat[0][col] * cofactors[0][col];
    }
    return det;
}

// Function to invert a 4x4 matrix
constexpr std::array<std::array<float,4>,4> invertMatrix4x4(const std::array<const std::array<float, 4>, 4> &mat) {
    const float det{ determinant4x4(mat) };
    std::array<std::array<float,4>,4> inv;
    // Calculate cofactors
    const std::array cofactors{ cofactor4x4(mat) };

    // Transpose cofactors to get adjugate
    for (unsigned int i = 0; i < 4; ++i) {
        for (unsigned int j = 0; j < 4; ++j) {
            inv[j][i] = cofactors[i][j] / det;
        }
    }
    return inv;
}

// Function to multiply a 4x4 matrix with a 1x4 vector
constexpr std::array<float, 4> multiplyMatrix4x4WithVector(
    const std::array<std::array<float, 4>, 4> &mat,
    const std::array<float, 4> &vec
) {
    std::array<float, 4> result;

    for (int i = 0; i < 4; ++i) {
        result[i] = 0.0;
        for (int j = 0; j < 4; ++j) {
            result[i] += mat[i][j] * vec[j];
        }
    }
    return result;
}
