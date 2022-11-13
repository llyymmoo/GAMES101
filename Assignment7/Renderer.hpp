//
// Created by goksu on 2/25/20.
//
#include "Scene.hpp"

#pragma once
struct hit_payload
{
    float tNear;
    uint32_t index;
    Vector2f uv;
    Object* hit_obj;
};

class Renderer
{
public:
    void Render(const Scene& scene);

private:
};

class ThreadRenderer
{
public:
    int row_beg;
    int row_end;
    int cols;
    int spp;
    Vector3f eye_pos;
    std::vector<Vector3f> frame_buff;
};
