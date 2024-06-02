//
// Created by goksu on 4/6/19.
//

#include "rasterizer.hpp"

#include <math.h>

#include <algorithm>
#include <opencv2/opencv.hpp>

#include "Eigen/src/Core/Matrix.h"

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

rst::col_buf_id rst::rasterizer::load_normals(
    const std::vector<Eigen::Vector3f>& normals)
{
    auto id = get_next_id();
    nor_buf.emplace(id, normals);

    normal_id = id;

    return {id};
}

// Bresenham's line drawing algorithm
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 255, 255};

    int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;

    dx = x2 - x1;
    dy = y2 - y1;
    dx1 = fabs(dx);
    dy1 = fabs(dy);
    px = 2 * dy1 - dx1;
    py = 2 * dx1 - dy1;

    if (dy1 <= dx1) {
        if (dx >= 0) {
            x = x1;
            y = y1;
            xe = x2;
        } else {
            x = x2;
            y = y2;
            xe = x1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point, line_color);
        for (i = 0; x < xe; i++) {
            x = x + 1;
            if (px < 0) {
                px = px + 2 * dy1;
            } else {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0)) {
                    y = y + 1;
                } else {
                    y = y - 1;
                }
                px = px + 2 * (dy1 - dx1);
            }
            //            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point, line_color);
        }
    } else {
        if (dy >= 0) {
            x = x1;
            y = y1;
            ye = y2;
        } else {
            x = x2;
            y = y2;
            ye = y1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point, line_color);
        for (i = 0; y < ye; i++) {
            y = y + 1;
            if (py <= 0) {
                py = py + 2 * dx1;
            } else {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0)) {
                    x = x + 1;
                } else {
                    x = x - 1;
                }
                py = py + 2 * (dx1 - dy1);
            }
            //            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point, line_color);
        }
    }
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

// static bool insideTriangle(int x, int y, const Vector4f* _v)
// {
//     Vector3f v[3];
//     for (int i = 0; i < 3; i++) v[i] = {_v[i].x(), _v[i].y(), 1.0};
//     Vector3f f0, f1, f2;
//     f0 = v[1].cross(v[0]);
//     f1 = v[2].cross(v[1]);
//     f2 = v[0].cross(v[2]);
//     Vector3f p(x, y, 1.);
//     if ((p.dot(f0) * f0.dot(v[2]) > 0) && (p.dot(f1) * f1.dot(v[0]) > 0) &&
//         (p.dot(f2) * f2.dot(v[1]) > 0))
//         return true;
//     return false;
// }

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
                                                            const Vector4f* v)
{
    auto& a = v[0];
    auto& b = v[1];
    auto& c = v[2];

    // float alpha = (x * (b.y() - c.y()) + (c.x() - b.x()) * y + b.x() * c.y()
    // -
    //                c.x() * b.y()) /
    //               (a.x() * (b.y() - c.y()) + (c.x() - b.x()) * a.y() +
    //                b.x() * c.y() - c.x() * b.y());
    float beta = (x * (c.y() - a.y()) + (a.x() - c.x()) * y + c.x() * a.y() -
                  a.x() * c.y()) /
                 (b.x() * (c.y() - a.y()) + (a.x() - c.x()) * b.y() +
                  c.x() * a.y() - a.x() * c.y());
    float gamma = (x * (a.y() - b.y()) + (b.x() - a.x()) * y + a.x() * b.y() -
                   b.x() * a.y()) /
                  (c.x() * (a.y() - b.y()) + (b.x() - a.x()) * c.y() +
                   a.x() * b.y() - b.x() * a.y());

    float alpha = 1.0f - beta - gamma;
    return {alpha, beta, gamma};
}

void rst::rasterizer::draw(std::vector<Triangle*>& TriangleList)
{
    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    Eigen::Matrix4f mv = view * model;
    // 法线变换矩阵
    Eigen::Matrix4f inv_trans = mv.inverse().transpose();
    for (const auto& t : TriangleList) {
        Triangle newtri = *t;

        std::array<Eigen::Vector4f, 3> mm{(mv * t->v[0]), (mv * t->v[1]),
                                          (mv * t->v[2])};

        std::array<Eigen::Vector3f, 3> viewspace_pos;

        std::transform(mm.begin(), mm.end(), viewspace_pos.begin(),
                       [](auto& v) { return v.template head<3>(); });

        Eigen::Vector4f v[] = {mvp * t->v[0], mvp * t->v[1], mvp * t->v[2]};
        // Homogeneous division
        for (auto& vec : v) {
            vec.x() /= vec.w();
            vec.y() /= vec.w();
            vec.z() /= vec.w();
        }

        Eigen::Vector4f n[] = {inv_trans * to_vec4(t->normal[0], 0.0f),
                               inv_trans * to_vec4(t->normal[1], 0.0f),
                               inv_trans * to_vec4(t->normal[2], 0.0f)};

        // Viewport transformation
        for (auto& vert : v) {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i) {
            // screen space coordinates
            newtri.setVertex(i, v[i]);
        }

        for (int i = 0; i < 3; ++i) {
            // view space normal
            newtri.setNormal(i, n[i].head<3>());
        }

        newtri.setColor(0, 148, 121.0, 92.0);
        newtri.setColor(1, 148, 121.0, 92.0);
        newtri.setColor(2, 148, 121.0, 92.0);

        // Also pass view space vertice position
        rasterize_triangle(newtri, viewspace_pos);
    }
}

static Eigen::Vector3f interpolate(float alpha, float beta, float gamma,
                                   const Eigen::Vector3f& vert1,
                                   const Eigen::Vector3f& vert2,
                                   const Eigen::Vector3f& vert3, float weight)
{
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector2f interpolate(float alpha, float beta, float gamma,
                                   const Eigen::Vector2f& vert1,
                                   const Eigen::Vector2f& vert2,
                                   const Eigen::Vector2f& vert3, float weight)
{
    auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
    auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

    u /= weight;
    v /= weight;

    return Eigen::Vector2f(u, v);
}

// Screen space rasterization
void rst::rasterizer::rasterize_triangle(
    const Triangle& t, const std::array<Eigen::Vector3f, 3>& view_pos)
{
    // TODO: From your HW3, get the triangle rasterization code.
    auto& v = t.v;
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
                auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);

                // 透视z插值矫正
                float w_reciprocal = 1.0f / (alpha / v[0].w() + beta / v[1].w() +
                                            gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() +
                                       beta * v[1].z() / v[1].w() +
                                       gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                alpha /= v[0].w();
                beta /= v[1].w();
                gamma /= v[2].w();
                float weight = 1.0f / w_reciprocal;

                int pixel_idx = get_index(x, y);
                if (z_interpolated < depth_buf[pixel_idx]) {
                    auto interpolated_color =
                        interpolate(alpha, beta, gamma, t.color[0], t.color[1],
                                    t.color[2], weight);
                    auto interpolated_normal =
                        interpolate(alpha, beta, gamma, t.normal[0],
                                    t.normal[1], t.normal[2], weight);
                    auto interpolated_texcoords =
                        interpolate(alpha, beta, gamma, t.tex_coords[0],
                                    t.tex_coords[1], t.tex_coords[2], weight);
                    auto interpolated_shadingcoords =
                        interpolate(alpha, beta, gamma, view_pos[0],
                                    view_pos[1], view_pos[2], weight);

                    fragment_shader_payload payload(
                        interpolated_color, interpolated_normal.normalized(),
                        interpolated_texcoords, texture ? &*texture : nullptr);

                    payload.view_pos = interpolated_shadingcoords;

                    auto pixel_color = fragment_shader(payload);

                    set_pixel({(float)x, (float)y}, pixel_color);
                    depth_buf[pixel_idx] = z_interpolated;
                }
            }
        }
    }
    /* clang-format off
    // TODO: Inside your rasterization loop:
    //    * v[i].w() is the vertex view space depth value z.
    //    * Z is interpolated view space depth for the current pixel
    //    * zp is depth between zNear and zFar, used for z-buffer

    // float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    // float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() +
    // gamma * v[2].z() / v[2].w(); zp *= Z;

    // TODO: Interpolate the attributes:
    // auto interpolated_color
    // auto interpolated_normal
    // auto interpolated_texcoords
    // auto interpolated_shadingcoords

    
    Use: fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
    Use: payload.view_pos = interpolated_shadingcoords;
    Use: Instead of passing the triangle's color directly to the frame buffer, pass the color to the shaders first to get the final color;
    Use: auto pixel_color = fragment_shader(payload);
    clang-format on */
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

    texture = std::nullopt;
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height - y) * width + x;
}

void rst::rasterizer::set_pixel(const Vector2i& point,
                                const Eigen::Vector3f& color)
{
    // old index: auto ind = point.y() + point.x() * width;
    int ind = (height - point.y()) * width + point.x();
    frame_buf[ind] = color;
}

void rst::rasterizer::set_vertex_shader(
    std::function<Eigen::Vector3f(vertex_shader_payload&)> vert_shader)
{
    vertex_shader = vert_shader;
}

void rst::rasterizer::set_fragment_shader(
    std::function<Eigen::Vector3f(fragment_shader_payload&)> frag_shader)
{
    fragment_shader = frag_shader;
}
