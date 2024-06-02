#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

#define NUM_CONTROL_POINTS (4)

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN &&
        control_points.size() < NUM_CONTROL_POINTS) {
        std::cout << "Left button of the mouse is clicked - position (" << x
                  << ", " << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window)
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) {
        auto point = std::pow(1 - t, 3) * p_0 +
                     3 * t * std::pow(1 - t, 2) * p_1 +
                     3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points,
                             float t)
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 2) {
        return (1 - t) * control_points[0] + t * control_points[1];
    }
    std::vector<cv::Point2f> points;
    points.reserve(control_points.size() - 1);
    for (int i = 0; i < control_points.size() - 1; ++i) {
        auto point = (1 - t) * control_points[i] + t * control_points[i + 1];
        points.push_back(point);
    }

    return recursive_bezier(points, t);
}

inline float get_distance(float x1, float y1, float x2, float y2)
{
    float x_dis = x2 - x1;
    float y_dis = y2 - y1;
    return std::sqrt(x_dis * x_dis + y_dis * y_dis);
}

inline void set_color(cv::Mat &window, float x, float y, unsigned char color)
{
    auto &r = window.at<cv::Vec3b>(y, x)[2];
    r = std::max(r, color);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de
    // Casteljau's recursive Bezier algorithm.
    const float max_d = sqrt(8);

    for (double t = 0.0; t <= 1.0; t += 0.001) {
        auto point = recursive_bezier(control_points, t);

        // 3x3 anti-aliasing
        float p0_x = std::floor(point.x);
        float p0_y = std::floor(point.y);
        float p1_x = p0_x;
        float p1_y = p0_y + 1.0f;

        float p2_x = p0_x + 1.0f;
        float p2_y = p0_y + 1.0f;

        float p3_x = p0_x + 1.0f;
        float p3_y = p0_y;

        float p4_x = p0_x + 1.0f;
        float p4_y = p0_y - 1.0f;

        float p5_x = p0_x;
        float p5_y = p0_y - 1.0f;

        float p6_x = p0_x - 1.0f;
        float p6_y = p0_y - 1.0f;

        float p7_x = p0_x - 1.0f;
        float p7_y = p0_y;

        float p8_x = p0_x - 1.0f;
        float p8_y = p0_y + 1.0f;

        float d0 =
            max_d - get_distance(point.x, point.y, p0_x + 0.5, p0_y + 0.5);
        float d1 =
            max_d - get_distance(point.x, point.y, p1_x + 0.5, p1_y + 0.5);
        float d2 =
            max_d - get_distance(point.x, point.y, p2_x + 0.5, p2_y + 0.5);
        float d3 =
            max_d - get_distance(point.x, point.y, p3_x + 0.5, p3_y + 0.5);
        float d4 =
            max_d - get_distance(point.x, point.y, p4_x + 0.5, p4_y + 0.5);
        float d5 =
            max_d - get_distance(point.x, point.y, p5_x + 0.5, p5_y + 0.5);
        float d6 =
            max_d - get_distance(point.x, point.y, p6_x + 0.5, p6_y + 0.5);
        float d7 =
            max_d - get_distance(point.x, point.y, p7_x + 0.5, p7_y + 0.5);
        float d8 =
            max_d - get_distance(point.x, point.y, p8_x + 0.5, p8_y + 0.5);

        constexpr int exp = 3;
        float k0 = std::pow(d0 / max_d, exp);
        float k1 = std::pow(d1 / max_d, exp);
        float k2 = std::pow(d2 / max_d, exp);
        float k3 = std::pow(d3 / max_d, exp);
        float k4 = std::pow(d4 / max_d, exp);
        float k5 = std::pow(d5 / max_d, exp);
        float k6 = std::pow(d6 / max_d, exp);
        float k7 = std::pow(d7 / max_d, exp);
        float k8 = std::pow(d8 / max_d, exp);

        set_color(window, p0_x, p0_y, k0 * 255);
        set_color(window, p1_x, p1_y, k1 * 255);
        set_color(window, p2_x, p2_y, k2 * 255);
        set_color(window, p3_x, p3_y, k3 * 255);
        set_color(window, p4_x, p4_y, k4 * 255);
        set_color(window, p5_x, p5_y, k5 * 255);
        set_color(window, p6_x, p6_y, k6 * 255);
        set_color(window, p7_x, p7_y, k7 * 255);
        set_color(window, p8_x, p8_y, k8 * 255);
    }
}

int main()
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) {
        for (auto &point : control_points) {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == NUM_CONTROL_POINTS) {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
