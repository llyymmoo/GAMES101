#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

static float max(float a, float b)
{
    return a > b ? a : b;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Use the same projection matrix from the previous assignments
    Eigen::Matrix4f projection;

    float alpha = eye_fov * 0.5;
    float top = fabs(zNear * tan(alpha / 57.3));

    Eigen::Matrix4f scale = Eigen::Matrix4f::Zero();
    scale(0, 0) = 1 / top;
    scale(1, 1) = 1 / top;
    scale(2, 2) = 2 / (zNear - zFar);
    scale(3, 3) = 1;

    Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
    translate(2, 3) = -(zNear + zFar) / 2;

    Eigen::Matrix4f persp2ortho = Eigen::Matrix4f::Zero();
    persp2ortho(0, 0) = zNear;
    persp2ortho(1, 1) = zNear;
    persp2ortho(3, 2) = 1;
    persp2ortho(2, 2) = zNear + zFar;
    persp2ortho(2, 3) = -zNear * zFar;

    projection = scale * translate * persp2ortho;

    return projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        return_color = payload.texture->getColorBilinear(payload.tex_coords.x(), payload.tex_coords.y()); // bilinear interpolation
        // return_color = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y()); // normal color
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 0};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // light intensity
        Eigen::Matrix3f I, Ia;
        I(0, 0) = light.intensity(0);
        I(1, 1) = light.intensity(1);
        I(2, 2) = light.intensity(2);
        Ia(0, 0) = amb_light_intensity(0);
        Ia(1, 1) = amb_light_intensity(1);
        Ia(2, 2) = amb_light_intensity(2);

        // vector para
        Eigen::Vector3f l = (light.position - point).normalized();
        Eigen::Vector3f h = ((eye_pos - point).normalized() + l).normalized();
        float r2 = pow((light.position - point).norm(), 2);

        // ambient, diffuse & specular
        Eigen::Vector3f ambient = Ia * ka;
        Eigen::Vector3f diffuse = (max(0.0, normal.dot(l)) / r2) * I * kd;
        Eigen::Vector3f specular = (pow(max(0.0, normal.dot(h)), p) / r2) * I * ks;

        result_color += ambient + diffuse + specular;
    }

    return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 0}; // all the pos are in view_coordinates

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal.normalized();

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        // light intensity
        Eigen::Matrix3f I, Ia;
        I(0, 0) = light.intensity(0);
        I(1, 1) = light.intensity(1);
        I(2, 2) = light.intensity(2);
        Ia(0, 0) = amb_light_intensity(0);
        Ia(1, 1) = amb_light_intensity(1);
        Ia(2, 2) = amb_light_intensity(2);

        // vector para
        Eigen::Vector3f l = (light.position - point).normalized();
        Eigen::Vector3f h = ((eye_pos - point).normalized() + l).normalized();
        float r2 = pow((light.position - point).norm(), 2);

        // ambient, diffuse & specular
        Eigen::Vector3f ambient = Ia * ka;
        Eigen::Vector3f diffuse = (max(0.0, normal.dot(l)) / r2) * I * kd;
        Eigen::Vector3f specular = (pow(max(0.0, normal.dot(h)), p) / r2) * I * ks;

        result_color += ambient + diffuse + specular;
    }

    return result_color * 255.f;
}



Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 0};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;

    Eigen::Vector3f n = normal.normalized();
    Eigen::Vector3f t(n.x()*n.y() / sqrt(n.x()*n.x() + n.z()*n.z()),
                      sqrt(n.x()*n.x() + n.z()*n.z()),
                      n.z()*n.y() / sqrt(n.x()*n.x() + n.z()*n.z()));
    Eigen::Vector3f b = n.cross(t);
    Eigen::Matrix3f TBN;
    TBN.block<3, 1>(0, 0) = t;
    TBN.block<3, 1>(0, 1) = b;
    TBN.block<3, 1>(0, 2) = n;

    float h_u1v = payload.texture->getColor(payload.tex_coords.x() + 1.0 / payload.texture->width, payload.tex_coords.y()).norm();
    float h_uv1 = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y() + 1.0 / payload.texture->height).norm();
    float h_uv = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y()).norm();
    float dU = kh * kn * (h_u1v - h_uv);
    float dV = kh * kn * (h_uv1 - h_uv);

    Eigen::Vector3f ln(-dU, -dV, 1);
    point += kn * h_uv * n; // point_pos changes height h_uv along the normal direction
    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // light intensity
        Eigen::Matrix3f I, Ia;
        I(0, 0) = light.intensity(0);
        I(1, 1) = light.intensity(1);
        I(2, 2) = light.intensity(2);
        Ia(0, 0) = amb_light_intensity(0);
        Ia(1, 1) = amb_light_intensity(1);
        Ia(2, 2) = amb_light_intensity(2);

        // vector para
        Eigen::Vector3f l = (light.position - point).normalized();
        Eigen::Vector3f h = ((eye_pos - point).normalized() + l).normalized();
        float r2 = pow((light.position - point).norm(), 2);

        // ambient, diffuse & specular
        Eigen::Vector3f ambient = Ia * ka;
        Eigen::Vector3f diffuse = (max(0.0, normal.dot(l)) / r2) * I * kd;
        Eigen::Vector3f specular = (pow(max(0.0, normal.dot(h)), p) / r2) * I * ks;

        result_color += ambient + diffuse + specular;
    }

    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 0};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;

    Eigen::Vector3f n = normal.normalized();
    Eigen::Vector3f t(n.x()*n.y() / sqrt(n.x()*n.x() + n.z()*n.z()),
                      sqrt(n.x()*n.x() + n.z()*n.z()),
                      n.z()*n.y() / sqrt(n.x()*n.x() + n.z()*n.z()));
    Eigen::Vector3f b = n.cross(t);
    Eigen::Matrix3f TBN;
    TBN.block<3, 1>(0, 0) = t;
    TBN.block<3, 1>(0, 1) = b;
    TBN.block<3, 1>(0, 2) = n;

    float h_u1v = payload.texture->getColor(payload.tex_coords.x() + 1.0 / payload.texture->width, payload.tex_coords.y()).norm();
    float h_uv1 = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y() + 1.0 / payload.texture->height).norm();
    float h_uv = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y()).norm();
    float dU = kh * kn * (h_u1v - h_uv);
    float dV = kh * kn * (h_uv1 - h_uv);

    Eigen::Vector3f ln(-dU, -dV, 1);
    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = normal;

    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";

    // Load .obj File
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    for(auto mesh : Loader.LoadedMeshes)
    {
        for(int i = 0; i < mesh.Vertices.size(); i += 3)
        {
            Triangle* t = new Triangle();
            for(int j = 0; j < 3; j++)
            {
                t->setVertex(j, Vector4f(mesh.Vertices[i+j].Position.X, mesh.Vertices[i+j].Position.Y, mesh.Vertices[i+j].Position.Z, 1.0));
                t->setNormal(j, Vector3f(mesh.Vertices[i+j].Normal.X, mesh.Vertices[i+j].Normal.Y, mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j, Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    int num_of_super_sample = 16;
    rst::rasterizer r(700, 700, num_of_super_sample); // last para : num of super-sample points

    auto texture_path = "hmap.jpg";
    r.set_texture(Texture(obj_path + texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the displacement shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0,0,10};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, -0.1, -50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, -0.1, -50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 10;
        }
        else if (key == 'd')
        {
            angle += 20;
        }

    }
    return 0;
}
