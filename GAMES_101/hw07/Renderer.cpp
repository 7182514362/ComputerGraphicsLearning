//
// Created by goksu on 2/25/20.
//

#include "Renderer.hpp"

#include <atomic>
#include <cstdint>
#include <fstream>
#include <thread>

#include "Scene.hpp"
#include "ThreadPool.h"

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // change the spp value to change sample ammount
    // samples per pixel
    int spp = 16;
    std::cout << "SPP: " << spp << "\n";
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                      imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

            Vector3f dir = normalize(Vector3f(-x, y, 1));
            for (int k = 0; k < spp; k++) {
                framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
            }
            m++;
        }

        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x),
                                                  0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y),
                                                  0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z),
                                                  0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}

void Renderer::RenderMultiThread(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // change the spp value to change sample ammount
    // samples per pixel
    ThreadPool pool;
    std::atomic_uint progress{0};
    constexpr int spp = 16;
    std::cout << "SPP: " << spp << "\n";
    for (uint32_t j = 0; j < scene.height; ++j) {
        pool.submit([j, eye_pos, &scene, &framebuffer, &progress]() {
            float scale = tan(deg2rad(scene.fov * 0.5));
            float imageAspectRatio = scene.width / (float)scene.height;
            int idx = j * scene.height;
            for (uint32_t i = 0; i < scene.width; ++i) {
                // generate primary ray direction
                float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                          imageAspectRatio * scale;
                float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                Vector3f dir = normalize(Vector3f(-x, y, 1));

                for (int k = 0; k < spp; k++) {
                    framebuffer[idx] +=
                        scene.castRay(Ray(eye_pos, dir), 0) / spp;
                }
                ++idx;
            }
            progress.fetch_add(1);
        });
    }
    pool.run();

    uint32_t old_progress = 0;
    while (progress < scene.height - 1) {
        if (progress != old_progress) {
            old_progress = progress;
            UpdateProgress(progress / (float)scene.height);
        }
    }
    UpdateProgress(1.f);

    pool.stop();
    pool.wait_all();

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x),
                                                  0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y),
                                                  0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z),
                                                  0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);

    std::cout << "\nfinished" << std::endl;
}
