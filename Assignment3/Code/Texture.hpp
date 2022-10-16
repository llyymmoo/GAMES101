//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        float u_img = u * width;
        float v_img = (1 - v) * height;

        int u_id = std::floor(u_img - 0.5);
        int v_id = std::floor(v_img - 0.5);

        cv::Vec3b color;

        if (u_id < 0 || u_id == width-1 || v_id < 0 || v_id == height-1)
        {
            color = image_data.at<cv::Vec3b>(v_img, u_img);
        }
        else
        {
            cv::Vec3b c00 = image_data.at<cv::Vec3b>(v_id, u_id);
            cv::Vec3b c01 = image_data.at<cv::Vec3b>(v_id, u_id+1);
            cv::Vec3b c10 = image_data.at<cv::Vec3b>(v_id+1, u_id);
            cv::Vec3b c11 = image_data.at<cv::Vec3b>(v_id+1, u_id+1);

            cv::Vec3b c0 = c00 + (c01 - c00) * (u_img - u_id - 0.5);
            cv::Vec3b c1 = c10 + (c11 - c10) * (u_img - u_id - 0.5);
            color = c0 + (c1 - c0) * (v_img - v_id - 0.5);
        }

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
