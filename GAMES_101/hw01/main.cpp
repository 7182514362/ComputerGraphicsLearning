#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "Triangle.hpp"
#include "rasterizer.hpp"

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(Eigen::Vector3f& angle)
{
    // Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    // Eigen::Matrix4f model;
    // model << 2.0, 0.0, -2.0, 0, 0.0, 2.0, -2.0, 0, -2.0, 0.0, -2.0, 0;

    // Eigen::AngleAxisf r(rotation_angle * rad,
    // Eigen::Vector3f::UnitZ());

    Eigen::Matrix4f rotateX;
    rotateX << 1, 0, 0, 0, 0, std::cos(angle[0]), -std::sin(angle[0]), 0, 0,
        std::sin(angle[0]), std::cos(angle[0]), 0, 0, 0, 0, 1;

    Eigen::Matrix4f rotateY;
    rotateY << std::cos(angle[1]), 0, std::sin(angle[1]), 0, 0, 1, 0, 0,
        -std::sin(angle[1]), 0, std::cos(angle[1]), 0, 0, 0, 0, 1;

    Eigen::Matrix4f rotateZ;
    rotateZ << std::cos(angle[2]), -std::sin(angle[2]), 0, 0,
        std::sin(angle[2]), std::cos(angle[2]), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

    return rotateX * rotateY * rotateZ;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float n, float f)
{
    // Students will implement this function

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float tan = std::tan(eye_fov / 2.0);

    Eigen::Matrix4f projection;
    projection << 1 / (aspect_ratio * tan), 0, 0, 0, 0, 1 / tan, 0, 0, 0, 0,
        (n + f) / (n - f), (2 * n * f) / (n - f), 0, 0, -1, 0;

    return projection;
}

int main(int argc, const char** argv)
{
    Eigen::Vector3f angle(0, 0, 0);

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        constexpr float degree = 10;
        constexpr float rad = MY_PI / 180;
        switch (key) {
            case 'a':
                angle[2] += degree * rad;
                break;
            case 'd':
                angle[2] -= degree * rad;
                break;
            case 'w':
                angle[0] -= degree * rad;
                break;
            case 's':
                angle[0] += degree * rad;
                break;
            case 'q':
                angle[1] -= degree * rad;
                break;
            case 'e':
                angle[1] += degree * rad;
                break;
            default:
                break;
        }
    }

    return 0;
}
