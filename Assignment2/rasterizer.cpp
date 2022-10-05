// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        int j = 0;
        for (auto& vec : v) {
            t.z[j++] = vec.w();
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
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

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t)
{
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    // auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    // float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    // float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    // z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

    int min_x, min_y, max_x, max_y;
    min_x = std::floor(std::min(v[0][0], std::min(v[1][0], v[2][0])));
    max_x = std::floor(std::max(v[0][0], std::max(v[1][0], v[2][0])));
    min_y = std::floor(std::min(v[0][1], std::min(v[1][1], v[2][1])));
    max_y = std::floor(std::max(v[0][1], std::max(v[1][1], v[2][1])));

    // normal method
    // for (int i = min_x; i <= max_x; ++i)
    // {
    //     for (int j = min_y; j <= max_y; ++j)
    //     {
    //         std::pair<float, float> pixel = std::make_pair(i+0.5, j+0.5);
    //         int ind = get_index(i, j);
    //         float depth = get_triangle_depth(pixel, t);
    //         if (is_inside_triangle(pixel, t) && depth_buf[ind] >= depth)
    //         {
    //             Eigen::Vector3f point = Eigen::Vector3f(i, j, 1.0f);
    //             set_pixel(point, t.getColor()); 
    //             depth_buf[ind] = depth;
    //         }
    //     }
    // }

    // super sampling
    std::vector<float> super_x(sample_num, 0);
    std::vector<float> super_y(sample_num, 0);
    int len = sqrt(sample_num);

    for (int i = 0; i < sample_num; ++i)
    {
        super_x[i] = 1.0 / len / 2 + 1.0 / len * (i % len);
        super_y[i] = 1.0 / len / 2 + 1.0 / len * (i / len);
    }

    for (int i = min_x; i <= max_x; ++i)
    {
        for (int j = min_y; j <= max_y; ++j)
        {
            for (int k = 0; k < sample_num; ++k)
            {
                std::pair<float, float> pixel = std::make_pair(i+super_x[k], j+super_y[k]);
                int ind = get_super_index(i, j, k);
                float depth = get_triangle_depth(pixel, t);
                if (is_inside_triangle(pixel, t) && super_depth_buf[ind] >= depth)
                {
                    Eigen::Vector3f point = Eigen::Vector3f(i, j, k);
                    Eigen::Vector3f tmp = set_super_pixel(point, t.getColor()); 
                    super_depth_buf[ind] = depth;

                    int index = get_index(i, j);
                    frame_buf[index] = (frame_buf[index] * sample_num - tmp + t.getColor()) / sample_num;
                }
            }
        }
    }
}

bool rst::rasterizer::is_inside_triangle(std::pair<float, float> pixel, const Triangle& t)
{
    std::vector<int> next_id = {1, 2, 0};
    std::vector<float> cross_z(3);
    
    for (int i = 0; i < 3; ++i) 
    {
        Eigen::Vector3f edge = Eigen::Vector3f(t.v[next_id[i]][0] - t.v[i][0],
                                               t.v[next_id[i]][1] - t.v[i][1],
                                               1.0);
        Eigen::Vector3f v2p = Eigen::Vector3f(pixel.first - t.v[i][0],
                                              pixel.second - t.v[i][1],
                                              1.0);
        
        Eigen::Vector3f cross_ans = edge.cross(v2p);
        cross_z[i] = cross_ans(2);
    }

    if ((cross_z[0] >= 0 && cross_z[1] >= 0 && cross_z[2] >= 0) ||
        (cross_z[0] <= 0 && cross_z[1] <= 0 && cross_z[2] <= 0))
        return true;
    else
        return false;
}

float rst::rasterizer::get_triangle_depth(std::pair<float, float> pixel, const Triangle& t)
{
    auto[alpha, beta, gamma] = computeBarycentric2D(pixel.first, pixel.second, t.v);
    float z_interpolated = 1.0/(alpha / t.z[0] + beta / t.z[1] + gamma / t.z[2]);
    // float z_interpolated = alpha * t.v[0][2] + beta * t.v[1][2] + gamma * t.v[2][2];
    // z_interpolated *= w_reciprocal;
    return -z_interpolated;
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
    
    // super sample
    std::fill(super_frame_buf.begin(), super_frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    std::fill(super_depth_buf.begin(), super_depth_buf.end(), std::numeric_limits<float>::infinity());

}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    
    // super sample
    sample_num = 16;
    super_frame_buf.resize(w * h * sample_num);
    super_depth_buf.resize(w * h * sample_num);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_super_index(int x, int y, int k)
{
    return ((height-1-y)*width + x) * sample_num + k;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;
}

Eigen::Vector3f rst::rasterizer::set_super_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = ((height-1-point.y())*width + point.x()) * sample_num + point.z();
    Eigen::Vector3f tmp = super_frame_buf[ind];
    super_frame_buf[ind] = color;

    return tmp;
}

// clang-format on