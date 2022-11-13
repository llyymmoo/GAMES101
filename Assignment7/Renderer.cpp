//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <thread>

#define THREAD_SHADE

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.000001;

void render_in_thread(ThreadRenderer& data, const Scene& scene)
{
    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos = data.eye_pos;
    int spp = data.spp;

    int m = 0;
    for (int j = data.row_beg; j < data.row_end; ++j)
    {
        for (int i = 0; i < data.cols; ++i)
        {
            float x = scale * 2 * imageAspectRatio * ( i + 0.5 - scene.width / 2 ) / scene.width;
            float y = -scale * 2 * ( j + 0.5 - scene.height / 2 ) / scene.height;
            Vector3f dir = normalize(Vector3f(-x, y, 1)); // normalize direction
            
            for (int k = 0; k < spp; k++) {
                data.frame_buff[m] += (scene.castRay(Ray(eye_pos, dir), 0) / spp);
            }
            ++m;
        }
    }

    return;
}

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    // 1. data initialization
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    int spp = 256;
    std::cout << "SPP: " << spp << "\n";

    // 2. use single thread or multi thread to render the scene
    //    by annotation or not the HREAD_SHADE on the top of this file
#ifdef THREAD_SHADE
    // 2.1 multi thread data
    int thread_num = 12;
    std::vector<ThreadRenderer> thread_renders(thread_num);

    int block_size = scene.height / thread_num;
    int start_row = 0;
    for (int i = 0; i < thread_num; ++i)
    {
        thread_renders[i].eye_pos = eye_pos;
        thread_renders[i].spp = spp;
        thread_renders[i].cols = scene.width;
        thread_renders[i].row_beg = start_row;
        thread_renders[i].row_end = (start_row + block_size > scene.height) ? scene.height : start_row + block_size;
        thread_renders[i].frame_buff.resize(scene.width * (thread_renders[i].row_end - thread_renders[i].row_beg));

        start_row += block_size;
    }

    // 2.2 start threads
    std::vector<std::thread*> threads_ptr(thread_num);
    for (int i = 0; i < thread_num; ++i)
    {
         threads_ptr[i] = new std::thread(render_in_thread, std::ref(thread_renders[i]), std::ref(scene));
    }

    // 2.3 wait threads to join
    for (int i = 0; i < thread_num; ++i)
    {
        threads_ptr[i]->join();
    }

    // 2.4 merge thread info
    m = 0;
    for (auto& render : thread_renders)
    {
        for (auto& v : render.frame_buff)
        {
            framebuffer[m++] = v;
        }
    }

#else
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {

            float x = scale * 2 * imageAspectRatio * ( i + 0.5 - scene.width / 2 ) / scene.width;
            float y = -scale * 2 * ( j + 0.5 - scene.height / 2 ) / scene.height;
            Vector3f dir = Vector3f(-x, y, 1).normalized(); // normalize direction
            
            for (int k = 0; k < spp; k++) {
                framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
            }
            m++;
        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);
#endif

    // 3. save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
