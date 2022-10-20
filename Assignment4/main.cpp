#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    int num = control_points.size();
    std::vector<cv::Point2f> _control_points = control_points;

    while (num > 1)
    {
        for (int i = 0; i < num-1; ++i)
            _control_points[i] = t * _control_points[i] + (1 - t) * _control_points[i+1];
        
        --num;
    }

    return _control_points[0];
}

void curve_antialising(cv::Point2f point, cv::Mat &window, std::vector<std::vector<int>> &cnt)
{
    int x = point.x;
    int y = point.y;

    float dx = point.x - x - 0.5;
    float dy = point.y - y + 0.5;

    int neighbor_dx = (dx > 0) ? 1 : -1;
    int neighbor_dy = (dy < 0) ? 1 : -1;

    dx = fabs(dx);
    dy = fabs(dy);

    int tmp_x, tmp_y;

    // center
    tmp_x = x;
    tmp_y = y;
    float color = (1 - sqrt(dx*dx + dy*dy)) * 255 + window.at<cv::Vec3b>(tmp_y, tmp_x)[1] * cnt[tmp_y][tmp_x] / (cnt[tmp_y][tmp_x] + 1);
    ++cnt[tmp_y][tmp_x];
    window.at<cv::Vec3b>(tmp_y, tmp_x)[1] = color;

    // neighbors
    tmp_x = x + neighbor_dx;
    tmp_y = y;
    if (tmp_x >= 0 && tmp_x < window.cols)
    {
        float color = dx * 255 + window.at<cv::Vec3b>(tmp_y, tmp_x)[1] * cnt[tmp_y][tmp_x] / (cnt[tmp_y][tmp_x] + 1);
        ++cnt[tmp_y][tmp_x];
        window.at<cv::Vec3b>(tmp_y, tmp_x)[1] = color;
    }

    tmp_y = y + neighbor_dy;
    tmp_x = x;
    if (tmp_y >= 0 && tmp_y < window.rows)
    {
        float color = dy * 255 + window.at<cv::Vec3b>(tmp_y, tmp_x)[1] * cnt[tmp_y][tmp_x] / (cnt[tmp_y][tmp_x] + 1);
        ++cnt[tmp_y][tmp_x];
        window.at<cv::Vec3b>(tmp_y, tmp_x)[1] = color;
    }

    
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    std::vector<std::vector<int>> cnt(window.rows, std::vector<int>(window.cols, 0));

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = recursive_bezier(control_points, t);

        // window.at<cv::Vec3b>(point.y, point.x)[1] = 255; // green
        curve_antialising(point, window, cnt);
    }

    return;
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
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
