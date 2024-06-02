
// clang-format on
// Created by goksu on 4/6/19.
//

#include "rasterizer.hpp"

#include <math.h>

#include <algorithm>
#include <array>
#include <opencv2/opencv.hpp>
#include <vector>

rst::pos_buf_id rst::rasterizer::load_positions(
    const std::vector<Eigen::Vector3f>& positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(
    const std::vector<Eigen::Vector3i>& indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(
    const std::vector<Eigen::Vector3f>& cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(const std::array<Eigen::Vector3f, 3>& triangleVec,
                           const std::array<Eigen::Vector3f, 3>& pointVec)
{
    // TODO : Implement this function to check if the point (x, y) is inside the
    // triangle represented by _v[0], _v[1], _v[2]

    float z1 = triangleVec[0].cross(pointVec[0]).z();
    float z2 = triangleVec[1].cross(pointVec[1]).z();
    float z3 = triangleVec[2].cross(pointVec[2]).z();

    auto isSameSign = [](float a, float b) -> bool {
        return (a > 0.0f && b > 0.0f) || (a < 0.0f && b < 0.0f);
    };
    return isSameSign(z1, z2) && isSameSign(z1, z3);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y,
                                                            const Vector3f* v)
{
    float c1 =
        (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y +
         v[1].x() * v[2].y() - v[2].x() * v[1].y()) /
        (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() +
         v[1].x() * v[2].y() - v[2].x() * v[1].y());
    float c2 =
        (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y +
         v[2].x() * v[0].y() - v[0].x() * v[2].y()) /
        (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() +
         v[2].x() * v[0].y() - v[0].x() * v[2].y());
    float c3 =
        (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y +
         v[0].x() * v[1].y() - v[1].x() * v[0].y()) /
        (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() +
         v[0].x() * v[1].y() - v[1].x() * v[0].y());
    return {c1, c2, c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer,
                           col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind) {
        Triangle t;
        Eigen::Vector4f v[] = {mvp * to_vec4(buf[i[0]], 1.0f),
                               mvp * to_vec4(buf[i[1]], 1.0f),
                               mvp * to_vec4(buf[i[2]], 1.0f)};
        // Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        // Viewport transformation
        for (auto& vert : v) {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int j = 0; j < 3; ++j) {
            t.setVertex(j, v[j].head<3>());
            t.setVertex(j, v[j].head<3>());
            t.setVertex(j, v[j].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

// Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t)
{
    auto v = t.toVector4();
    // auto v = t.v;

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the
    // triangle
    int xleft = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    int xright = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
    int ybottom = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
    int ytop = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));

    std::array<Eigen::Vector3f, 3> triangleVec{
        Eigen::Vector3f{v[1].x() - v[0].x(), v[1].y() - v[0].y(),
                        v[1].z() - v[0].z()},
        Eigen::Vector3f{v[2].x() - v[1].x(), v[2].y() - v[1].y(),
                        v[2].z() - v[1].z()},
        Eigen::Vector3f{v[0].x() - v[2].x(), v[0].y() - v[2].y(),
                        v[0].z() - v[2].z()}};

    std::array<Eigen::Vector3f, 3> pointVec{Eigen::Vector3f{0, 0, -v[0].z()},
                                            Eigen::Vector3f{0, 0, -v[1].z()},
                                            Eigen::Vector3f{0, 0, -v[2].z()}};

    for (int y = ytop; y >= ybottom; --y) {
        pointVec[0].y() = y - v[0].y();
        pointVec[1].y() = y - v[1].y();
        pointVec[2].y() = y - v[2].y();
        for (int x = xleft; x <= xright; ++x) {
            pointVec[0].x() = x - v[0].x();
            pointVec[1].x() = x - v[1].x();
            pointVec[2].x() = x - v[2].x();
            if (insideTriangle(triangleVec, pointVec)) {
                // If so, use the following code to get the interpolated z
                // value.
                auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() +
                                            gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() +
                                       beta * v[1].z() / v[1].w() +
                                       gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                if (z_interpolated < depth_buf[get_index(x, y)]) {
                    // set the current pixel (use the set_pixel function) to the
                    // color of
                    // the triangle (use getColor function) if it should be
                    // painted.
                    set_pixel({(float)x, (float)y, 1.0f}, t.getColor());
                    depth_buf[get_index(x, y)] = z_interpolated;
                }
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m) { model = m; }

void rst::rasterizer::set_view(const Eigen::Matrix4f& v) { view = v; }

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color) {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth) {
        std::fill(depth_buf.begin(), depth_buf.end(),
                  std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point,
                                const Eigen::Vector3f& color)
{
    // old index: auto ind = point.y() + point.x() * width;
    auto ind = (height - 1 - point.y()) * width + point.x();
    frame_buf[ind] = color;
}

// clang-format on