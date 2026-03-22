#include "graphics.hpp"
#include "globals.h"
#include <iostream>
#include <cmath>
#include <algorithm>

// gravitational lensing calculation
SDL_FPoint calculateLensing(SDL_FPoint starPos, SDL_FPoint bhPos, float rs, float clipW, float objZ, float bhZ) {
    float dx = starPos.x - bhPos.x;
    float dy = starPos.y - bhPos.y;
    float r = sqrtf(dx * dx + dy * dy) + 0.1f;

    float screenRs = (rs * 900.0f) / clipW;
    float warpFactor = (objZ > bhZ) ? 1.4f : 1.0f;

    float deflection = (screenRs * screenRs) / (r + screenRs * 0.2f);

    SDL_FPoint p;
    p.x = bhPos.x + dx * (warpFactor + deflection / r);
    p.y = bhPos.y + dy * (warpFactor + deflection / r);
    return p;
}

void generateAccretionDisk(std::vector<Dust>& disk, float rs) {
    disk.clear();
    for (int i = 0; i < 4000; ++i) {
        Dust d = {};
        float angle = (rand() % 628) / 100.0f;
        float dist = rs * 2.5f + ((rand() % 1000) / 1000.0f) * (rs * 5.0f);
        d.position = Eigen::Vector4f(dist * cosf(angle), ((rand() % 100) - 50) * 0.01f, dist * sinf(angle), 1.0f);
        float vMag = 8.0f / sqrtf(dist);
        d.velocity = Eigen::Vector3f(-sinf(angle) * vMag, 0, cosf(angle) * vMag);
        d.colour = { 255, (Uint8)(160 + rand() % 70), (Uint8)(50 + rand() % 80), 255 };
        d.brightness = 0.4f + ((rand() % 60) / 100.0f);
        disk.push_back(d);
    }
}

void SDL_RenderFillCircle(SDL_Renderer* renderer, float x, float y, float radius) {
    if (radius < 0.5f) return;
    for (float w = -radius; w <= radius; w += 1.0f) {
        float h = sqrtf(radius * radius - w * w);
        SDL_RenderLine(renderer, x + w, y - h, x + w, y + h);
    }
}

int simulatorMain() {
    if (!SDL_Init(SDL_INIT_VIDEO)) return -1;
    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_CreateWindowAndRenderer(WINDOW_TITLE, WIDTH, HEIGHT, 0, &window, &renderer);
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    Camera cam = {};
    cam.z = 300.0f;
    cam.lookSpeed = 0.005f;
    cam.moveSpeed = 2.5f;

    std::vector<BackgroundStar> stars;
    std::vector<Dust> disk;
    float bh_rs = 45.0f;

    generateAccretionDisk(disk, bh_rs);

    for (int i = 0; i < 800; ++i) {
        BackgroundStar s = {};
        float rad = 1000.0f + (rand() % 2000);
        float theta = (rand() % 628) / 100.0f;
        float phi = (rand() % 314) / 100.0f;
        s.position = Eigen::Vector4f(rad * sinf(phi) * cosf(theta), rad * sinf(phi) * sinf(theta), rad * cosf(phi), 1.0f);
        s.velocity = Eigen::Vector3f(0, 0, 0);
        s.colour = { (Uint8)(200 + rand() % 55), (Uint8)(200 + rand() % 55), 255, 255 };
        stars.push_back(s);
    }

    bool running = true;
    SDL_Event event;
    float deltaTime = 0.016f;

    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT) running = false;
        }

        float mX, mY;
        uint32_t mouse = SDL_GetRelativeMouseState(&mX, &mY);
        const bool* keys = SDL_GetKeyboardState(NULL);

        if ((mouse & SDL_BUTTON_RMASK) != 0) {
            cam.yaw += mX * cam.lookSpeed;
            cam.pitch += mY * cam.lookSpeed;
            cam.pitch = SDL_clamp(cam.pitch, -1.5f, 1.5f);

            float cP = cosf(cam.pitch), sP = sinf(cam.pitch);
            float cY = cosf(cam.yaw), sY = sinf(cam.yaw);

            Eigen::Vector3f fwd(sY * cP, -sP, -cY * cP);
            Eigen::Vector3f rite(cY, 0, sY);
            Eigen::Vector3f up = rite.cross(fwd);

            if (keys[SDL_SCANCODE_W]) { cam.x += fwd.x() * cam.moveSpeed; cam.y += fwd.y() * cam.moveSpeed; cam.z += fwd.z() * cam.moveSpeed; }
            if (keys[SDL_SCANCODE_S]) { cam.x -= fwd.x() * cam.moveSpeed; cam.y -= fwd.y() * cam.moveSpeed; cam.z -= fwd.z() * cam.moveSpeed; }
            if (keys[SDL_SCANCODE_A]) { cam.x -= rite.x() * cam.moveSpeed; cam.y -= rite.y() * cam.moveSpeed; cam.z -= rite.z() * cam.moveSpeed; }
            if (keys[SDL_SCANCODE_D]) { cam.x += rite.x() * cam.moveSpeed; cam.y += rite.y() * cam.moveSpeed; cam.z += rite.z() * cam.moveSpeed; }
            if (keys[SDL_SCANCODE_SPACE]) { cam.x += up.x() * cam.moveSpeed; cam.y += up.y() * cam.moveSpeed; cam.z += up.z() * cam.moveSpeed; }
            if (keys[SDL_SCANCODE_LSHIFT]) { cam.x -= up.x() * cam.moveSpeed; cam.y -= up.y() * cam.moveSpeed; cam.z -= up.z() * cam.moveSpeed; }
        }

        // disk physics
        for (auto& d : disk) {
            Eigen::Vector3f p3 = d.position.head<3>();
            float dist = p3.norm();

            if (dist < bh_rs * 1.1f) {
                float ang = (rand() % 628) / 100.0f;
                float rNew = bh_rs * 7.0f;
                d.position.head<3>() = Eigen::Vector3f(rNew * cosf(ang), 0, rNew * sinf(ang));
                dist = rNew;
            }

            p3 += d.velocity * 60.0f * deltaTime;
            p3 = p3.normalized() * dist;
            d.position.head<3>() = p3;
            d.velocity = p3.cross(Eigen::Vector3f(0, 1, 0)).normalized() * (8.0f / sqrtf(dist + 0.1f));
        }

        // star physics
        for (auto& s : stars) {
            Eigen::Vector3f p3 = s.position.head<3>();
            float d2 = p3.squaredNorm();
            float d = sqrtf(d2);

            if (d < bh_rs * 1.2f || d > 5000.0f) {
                float rad = 3000.0f;
                float th = (rand() % 628) / 100.0f;
                float ph = (rand() % 314) / 100.0f;
                s.position.head<3>() = Eigen::Vector3f(rad * sinf(ph) * cosf(th), rad * sinf(ph) * sinf(th), rad * cosf(ph));
                s.velocity = Eigen::Vector3f(0, 0, 0);
                continue;
            }

            Eigen::Vector3f grav = -p3.normalized() * (15000.0f / (d2 + 100.0f));
            s.velocity += grav * deltaTime;
            s.position.head<3>() += s.velocity * deltaTime * 10.0f;
        }

        SDL_SetRenderDrawColor(renderer, 2, 2, 10, 255);
        SDL_RenderClear(renderer);

        Eigen::Matrix4f mvp = getProjectionMatrix(1.57f, (float)WIDTH / HEIGHT, 0.1f, 6000.0f) * getViewMatrix(cam);
        Eigen::Vector4f bh_clip = mvp * Eigen::Vector4f(0, 0, 0, 1);
        SDL_FPoint bh_screen = projectToPixels(bh_clip, WIDTH, HEIGHT);

        // draw stars
        for (auto& s : stars) {
            Eigen::Vector3f p3 = s.position.head<3>();
            float d2 = p3.squaredNorm();
            float d = sqrtf(d2);

            if (d < bh_rs * 1.3f || d > 5000.0f) {
                float rad = 3000.0f + (rand() % 500);
                float th = (rand() % 628) / 100.0f;
                float ph = (rand() % 314) / 100.0f;
                s.position.head<3>() = Eigen::Vector3f(rad * sinf(ph) * cosf(th), rad * sinf(ph) * sinf(th), rad * cosf(ph));
                s.velocity = Eigen::Vector3f(0, 0, 0);
                continue;
            }

            Eigen::Vector3f gravityDir = -p3.normalized();
            float pull = 20000.0f / (d2 + 50.0f);
            s.velocity += gravityDir * pull * deltaTime;
            s.position.head<3>() += s.velocity * deltaTime * 10.0f;
            Eigen::Vector4f clip = mvp * s.position;

            if (clip.w() > 0.1f) {
                SDL_FPoint p = projectToPixels(clip, WIDTH, HEIGHT);
                if (bh_clip.w() > 0.1f) p = calculateLensing(p, bh_screen, bh_rs, clip.w(), clip.z(), bh_clip.z());
                float dim = SDL_clamp(1500.0f / (clip.w() * 4.0f + 1.0f), 0.1f, 1.0f);
                SDL_SetRenderDrawColor(renderer, (Uint8)(s.colour.r * dim), (Uint8)(s.colour.g * dim), (Uint8)(s.colour.b * dim), 255);
                SDL_RenderPoint(renderer, p.x, p.y);
            }
        }

        // painters algorithm sort
        std::sort(disk.begin(), disk.end(), [&](const Dust& a, const Dust& b) {
            return (mvp * a.position).w() > (mvp * b.position).w();
            });

        // draw disk
        Eigen::Vector3f camVec = Eigen::Vector3f(cam.x, cam.y, cam.z).normalized();
        for (auto& d : disk) {
            Eigen::Vector4f clip = mvp * d.position;
            if (clip.w() > 0.1f) {
                SDL_FPoint p = projectToPixels(clip, WIDTH, HEIGHT);
                if (bh_clip.w() > 0.1f) p = calculateLensing(p, bh_screen, bh_rs, clip.w(), clip.z(), bh_clip.z());
                float doppler = 1.0f + (d.velocity.dot(camVec) * 1.5f);
                SDL_SetRenderDrawColor(renderer,
                    (Uint8)SDL_clamp(d.colour.r * d.brightness * doppler, 0, 255),
                    (Uint8)SDL_clamp(d.colour.g * d.brightness * doppler * 0.7f, 0, 255),
                    (Uint8)SDL_clamp(d.colour.b * d.brightness * doppler * 0.4f, 0, 255), 255);
                SDL_RenderPoint(renderer, p.x, p.y);
            }
        }

        // draw event horizon
        if (bh_clip.w() > 0.1f) {
            float s_rad = (bh_rs * 2.6f * 40.0f) / bh_clip.w();
            SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
            SDL_RenderFillCircle(renderer, bh_screen.x, bh_screen.y, s_rad);
        }

        SDL_RenderPresent(renderer);
    }
    SDL_Quit();
    return 0;
}

Eigen::Matrix4f getViewMatrix(Camera c) {
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    Eigen::AngleAxisf rollAngle(0.0f, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(c.pitch, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf yawAngle(c.yaw, Eigen::Vector3f::UnitY());
    Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
    view.block<3, 3>(0, 0) = q.matrix().transpose();
    Eigen::Vector3f pos(c.x, c.y, c.z);
    view.block<3, 1>(0, 3) = -(q.matrix().transpose() * pos);
    return view;
}

Eigen::Matrix4f getProjectionMatrix(float fov, float aspect, float near_plane, float far_plane) {
    Eigen::Matrix4f projection = Eigen::Matrix4f::Zero();
    float tanHalfFov = tanf(fov / 2.0f);
    projection(0, 0) = 1.0f / (aspect * tanHalfFov);
    projection(1, 1) = 1.0f / tanHalfFov;
    projection(2, 2) = -(far_plane + near_plane) / (far_plane - near_plane);
    projection(2, 3) = -(2.0f * far_plane * near_plane) / (far_plane - near_plane);
    projection(3, 2) = -1.0f;
    return projection;
}

SDL_FPoint projectToPixels(Eigen::Vector4f screen_point, int width, int height) {
    float ndcX = screen_point.x() / screen_point.w();
    float ndcY = screen_point.y() / screen_point.w();
    SDL_FPoint p;
    p.x = (ndcX + 1.0f) * 0.5f * (float)width;
    p.y = (1.0f - ndcY) * 0.5f * (float)height;
    return p;
}