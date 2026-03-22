#ifndef GRAPHICS_HPP
#define GRAPHICS_HPP

#include <SDL3/SDL.h>
#include <Eigen/Dense>
#include <vector>

struct Camera {
    float x = 0.0f, y = 0.0f, z = 150.0f;
    float yaw = 0.0f, pitch = 0.0f;
    float lookSpeed = 0.005f;
    float moveSpeed = 1.5f;
};

struct BackgroundStar {
    Eigen::Vector4f position;
    Eigen::Vector3f velocity;
    SDL_Color colour;
    float size_scale;
};

struct Dust {
    Eigen::Vector4f position;
    Eigen::Vector3f velocity;
    SDL_Color colour;
    float brightness;
    float size;
};

// core functions
int simulatorMain();
Eigen::Matrix4f getViewMatrix(Camera c);
Eigen::Matrix4f getProjectionMatrix(float fov, float aspect, float near, float far);
SDL_FPoint projectToPixels(Eigen::Vector4f screen_point, int width, int height);
void SDL_RenderFillCircle(SDL_Renderer* renderer, float x, float y, float radius);

// black hole functions
void generateAccretionDisk(std::vector<Dust>& disk, float rs);
SDL_FPoint calculateLensing(SDL_FPoint starPos, SDL_FPoint bhPos, float rs, float clipW, float objZ, float bhZ);

#endif