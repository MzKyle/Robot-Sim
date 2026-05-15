#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct DcCloudRenderer DcCloudRenderer;

DcCloudRenderer *dc_cloud_renderer_create(void);
void dc_cloud_renderer_destroy(DcCloudRenderer *renderer);

int dc_cloud_renderer_initialize(DcCloudRenderer *renderer);
int dc_cloud_renderer_resize(DcCloudRenderer *renderer, int width, int height);
int dc_cloud_renderer_upload_points(DcCloudRenderer *renderer, const float *xyz, size_t point_count);
int dc_cloud_renderer_upload_points_rgb(
    DcCloudRenderer *renderer,
    const float *xyz,
    const float *rgb,
    size_t point_count);
int dc_cloud_renderer_upload_points_interleaved(
    DcCloudRenderer *renderer,
    const float *xyzrgb,
    size_t point_count);
int dc_cloud_renderer_draw(
    DcCloudRenderer *renderer,
    float yaw,
    float pitch,
    float distance,
    float pan_x,
    float pan_y,
    float point_size);

const char *dc_cloud_renderer_last_error(DcCloudRenderer *renderer);

#ifdef __cplusplus
}
#endif
