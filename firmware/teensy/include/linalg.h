#pragma once
#include <cmath>
#include <cstring>
#include <algorithm>

namespace linalg {

template <int R, int C>
struct Mat {
    float d[R][C]{};

    float& operator()(int r, int c)       { return d[r][c]; }
    float  operator()(int r, int c) const { return d[r][c]; }

    static Mat zeros() { Mat m; return m; }

    static Mat identity() {
        static_assert(R == C, "identity requires square matrix");
        Mat m;
        for (int i = 0; i < R; i++) m.d[i][i] = 1.0f;
        return m;
    }

    Mat<C, R> transpose() const {
        Mat<C, R> t;
        for (int r = 0; r < R; r++)
            for (int c = 0; c < C; c++)
                t.d[c][r] = d[r][c];
        return t;
    }

    Mat operator+(const Mat& o) const {
        Mat res;
        for (int r = 0; r < R; r++)
            for (int c = 0; c < C; c++)
                res.d[r][c] = d[r][c] + o.d[r][c];
        return res;
    }

    Mat operator-(const Mat& o) const {
        Mat res;
        for (int r = 0; r < R; r++)
            for (int c = 0; c < C; c++)
                res.d[r][c] = d[r][c] - o.d[r][c];
        return res;
    }

    Mat operator*(float s) const {
        Mat res;
        for (int r = 0; r < R; r++)
            for (int c = 0; c < C; c++)
                res.d[r][c] = d[r][c] * s;
        return res;
    }

    template <int C2>
    Mat<R, C2> operator*(const Mat<C, C2>& o) const {
        Mat<R, C2> res;
        for (int r = 0; r < R; r++)
            for (int c2 = 0; c2 < C2; c2++) {
                float sum = 0;
                for (int k = 0; k < C; k++)
                    sum += d[r][k] * o.d[k][c2];
                res.d[r][c2] = sum;
            }
        return res;
    }
};

template <int N>
using Vec = Mat<N, 1>;

template <int N>
bool invert(const Mat<N, N>& src, Mat<N, N>& dst) {
    float aug[N][2 * N];
    for (int r = 0; r < N; r++) {
        for (int c = 0; c < N; c++) {
            aug[r][c] = src.d[r][c];
            aug[r][c + N] = (r == c) ? 1.0f : 0.0f;
        }
    }
    for (int col = 0; col < N; col++) {
        int max_row = col;
        float max_val = std::abs(aug[col][col]);
        for (int r = col + 1; r < N; r++) {
            float v = std::abs(aug[r][col]);
            if (v > max_val) { max_val = v; max_row = r; }
        }
        if (max_val < 1e-12f) return false;
        if (max_row != col) {
            for (int c = 0; c < 2 * N; c++)
                std::swap(aug[col][c], aug[max_row][c]);
        }
        float pivot = aug[col][col];
        for (int c = 0; c < 2 * N; c++)
            aug[col][c] /= pivot;
        for (int r = 0; r < N; r++) {
            if (r == col) continue;
            float factor = aug[r][col];
            for (int c = 0; c < 2 * N; c++)
                aug[r][c] -= factor * aug[col][c];
        }
    }
    for (int r = 0; r < N; r++)
        for (int c = 0; c < N; c++)
            dst.d[r][c] = aug[r][c + N];
    return true;
}

}
