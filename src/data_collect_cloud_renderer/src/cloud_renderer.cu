#include "data_collect_cloud_renderer/cloud_renderer.h"

#include <GL/glew.h>
#include <cuda_gl_interop.h>
#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <sstream>
#include <string>

namespace {

constexpr size_t kPointStride = 6;

constexpr const char *kVertexShader = R"GLSL(
#version 330 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_color;

uniform vec3 u_center;
uniform float u_radius;
uniform float u_yaw;
uniform float u_pitch;
uniform vec2 u_pan;
uniform float u_distance;
uniform float u_aspect;
uniform float u_min_z;
uniform float u_max_z;
uniform float u_point_size;

out float v_height;
out vec3 v_color;

void main()
{
    vec3 p = a_position;
    if (u_radius > 0.0) {
        p = (a_position - u_center) / max(u_radius, 0.000001);
    }

    float camera_distance = max(u_distance, 0.35);
    float cy = cos(u_yaw);
    float sy = sin(u_yaw);
    float cp = cos(u_pitch);
    float sp = sin(u_pitch);

    vec3 camera_dir = normalize(vec3(cp * cy, cp * sy, sp));
    vec3 forward = -camera_dir;
    vec3 right = cross(forward, vec3(0.0, 0.0, 1.0));
    if (dot(right, right) < 0.0001) {
        right = vec3(1.0, 0.0, 0.0);
    }
    right = normalize(right);
    vec3 up = cross(right, forward);

    vec3 from_camera = p - camera_dir * camera_distance;
    float view_x = dot(from_camera, right) + u_pan.x;
    float view_y = dot(from_camera, up) + u_pan.y;
    float view_z = dot(from_camera, forward);

    const float near_plane = 0.02;
    const float far_plane = 200.0;
    const float tan_half_fov = 0.41421356237;
    float clip_x = view_x / (max(u_aspect, 0.001) * tan_half_fov);
    float clip_y = view_y / tan_half_fov;
    float clip_z = ((far_plane + near_plane) / (far_plane - near_plane)) * view_z
        - (2.0 * far_plane * near_plane) / (far_plane - near_plane);
    gl_Position = vec4(clip_x, clip_y, clip_z, view_z);
    gl_PointSize = u_point_size;
    v_height = u_radius > 0.0
        ? clamp((a_position.z - u_min_z) / max(u_max_z - u_min_z, 0.000001), 0.0, 1.0)
        : clamp(a_position.z * 0.5 + 0.5, 0.0, 1.0);
    v_color = a_color;
}
)GLSL";

constexpr const char *kFragmentShader = R"GLSL(
#version 330 core
in float v_height;
in vec3 v_color;
out vec4 frag_color;

void main()
{
    vec3 height_low = vec3(0.12, 0.46, 0.92);
    vec3 height_mid = vec3(0.10, 0.78, 0.58);
    vec3 height_high = vec3(1.00, 0.70, 0.18);
    vec3 height_color = v_height < 0.5
        ? mix(height_low, height_mid, v_height * 2.0)
        : mix(height_mid, height_high, (v_height - 0.5) * 2.0);
    float color_strength = max(max(v_color.r, v_color.g), v_color.b);
    vec3 color = color_strength > 0.12 ? max(v_color, vec3(0.18)) : height_color;
    frag_color = vec4(color, 1.0);
}
)GLSL";

std::string gl_error_string(const std::string &prefix)
{
    const GLenum err = glGetError();
    if (err == GL_NO_ERROR) {
        return prefix;
    }
    std::ostringstream out;
    out << prefix << " (GL error 0x" << std::hex << static_cast<unsigned int>(err) << ")";
    return out.str();
}

std::string cuda_error_string(const std::string &prefix, cudaError_t err)
{
    std::ostringstream out;
    out << prefix << ": " << cudaGetErrorName(err) << " - " << cudaGetErrorString(err);
    return out.str();
}

GLuint compile_shader(GLenum type, const char *source, std::string &error)
{
    const GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint ok = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (ok == GL_TRUE) {
        return shader;
    }

    GLint length = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
    std::string log(std::max(1, length), '\0');
    glGetShaderInfoLog(shader, length, nullptr, log.data());
    glDeleteShader(shader);
    error = "shader compile failed: " + log;
    return 0;
}

GLuint link_program(std::string &error)
{
    const GLuint vertex = compile_shader(GL_VERTEX_SHADER, kVertexShader, error);
    if (!vertex) {
        return 0;
    }

    const GLuint fragment = compile_shader(GL_FRAGMENT_SHADER, kFragmentShader, error);
    if (!fragment) {
        glDeleteShader(vertex);
        return 0;
    }

    const GLuint program = glCreateProgram();
    glAttachShader(program, vertex);
    glAttachShader(program, fragment);
    glLinkProgram(program);

    glDeleteShader(vertex);
    glDeleteShader(fragment);

    GLint ok = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (ok == GL_TRUE) {
        return program;
    }

    GLint length = 0;
    glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
    std::string log(std::max(1, length), '\0');
    glGetProgramInfoLog(program, length, nullptr, log.data());
    glDeleteProgram(program);
    error = "shader link failed: " + log;
    return 0;
}

}  // namespace

struct DcCloudRenderer {
    GLuint vao = 0;
    GLuint vbo = 0;
    GLuint program = 0;
    cudaGraphicsResource *cuda_vbo = nullptr;
    size_t capacity = 0;
    size_t count = 0;
    int width = 1;
    int height = 1;
    float center[3] = {0.0f, 0.0f, 0.0f};
    float radius = 1.0f;
    float min_z = 0.0f;
    float max_z = 1.0f;
    bool initialized = false;
    bool uploaded_once = false;
    bool prefer_cuda_upload = false;
    std::string error;

    void set_error(const std::string &message)
    {
        error = message;
    }

    bool unregister_cuda_buffer()
    {
        if (!cuda_vbo) {
            return true;
        }
        const cudaError_t err = cudaGraphicsUnregisterResource(cuda_vbo);
        cuda_vbo = nullptr;
        if (err != cudaSuccess) {
            set_error(cuda_error_string("cudaGraphicsUnregisterResource failed", err));
            return false;
        }
        return true;
    }

    bool ensure_capacity(size_t requested)
    {
        requested = std::max<size_t>(requested, 1);
        if (requested <= capacity && vbo && (!prefer_cuda_upload || cuda_vbo)) {
            return true;
        }

        size_t new_capacity = std::max<size_t>(requested, capacity ? capacity * 2 : 8192);
        if (!unregister_cuda_buffer()) {
            return false;
        }
        if (vbo) {
            glDeleteBuffers(1, &vbo);
            vbo = 0;
        }

        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(
            GL_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(new_capacity * kPointStride * sizeof(float)),
            nullptr,
            GL_DYNAMIC_DRAW);
        GLenum gl_err = glGetError();
        if (gl_err != GL_NO_ERROR) {
            std::ostringstream out;
            out << "failed to allocate OpenGL VBO (GL error 0x"
                << std::hex << static_cast<unsigned int>(gl_err) << ")";
            set_error(out.str());
            return false;
        }

        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, kPointStride * sizeof(float), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(
            1,
            3,
            GL_FLOAT,
            GL_FALSE,
            kPointStride * sizeof(float),
            reinterpret_cast<void *>(3 * sizeof(float)));
        glBindVertexArray(0);

        if (!prefer_cuda_upload) {
            capacity = new_capacity;
            return true;
        }

        const cudaError_t err = cudaGraphicsGLRegisterBuffer(
            &cuda_vbo,
            vbo,
            cudaGraphicsRegisterFlagsWriteDiscard);
        if (err != cudaSuccess) {
            cuda_vbo = nullptr;
            prefer_cuda_upload = false;
            capacity = new_capacity;
            return true;
        }

        capacity = new_capacity;
        return true;
    }
};

std::vector<float> make_interleaved_points(const float *xyz, const float *rgb, size_t point_count)
{
    std::vector<float> interleaved;
    interleaved.resize(point_count * kPointStride);
    for (size_t i = 0; i < point_count; ++i) {
        interleaved[i * kPointStride + 0] = xyz[i * 3 + 0];
        interleaved[i * kPointStride + 1] = xyz[i * 3 + 1];
        interleaved[i * kPointStride + 2] = xyz[i * 3 + 2];
        if (rgb) {
            interleaved[i * kPointStride + 3] = std::clamp(rgb[i * 3 + 0], 0.0f, 1.0f);
            interleaved[i * kPointStride + 4] = std::clamp(rgb[i * 3 + 1], 0.0f, 1.0f);
            interleaved[i * kPointStride + 5] = std::clamp(rgb[i * 3 + 2], 0.0f, 1.0f);
        } else {
            interleaved[i * kPointStride + 3] = 0.0f;
            interleaved[i * kPointStride + 4] = 0.0f;
            interleaved[i * kPointStride + 5] = 0.0f;
        }
    }
    return interleaved;
}

float percentile(std::vector<float> values, float q)
{
    if (values.empty()) {
        return 0.0f;
    }
    const size_t index = std::min(
        values.size() - 1,
        static_cast<size_t>(std::floor(q * static_cast<float>(values.size() - 1))));
    std::nth_element(values.begin(), values.begin() + index, values.end());
    return values[index];
}

void update_fit(DcCloudRenderer *renderer, const float *points, size_t point_count, size_t stride)
{
    std::vector<float> xs;
    std::vector<float> ys;
    std::vector<float> zs;
    const size_t sample_count = std::min<size_t>(point_count, 50000);
    const size_t step = std::max<size_t>(1, point_count / sample_count);
    xs.reserve(sample_count);
    ys.reserve(sample_count);
    zs.reserve(sample_count);
    for (size_t i = 0; i < point_count; i += step) {
        xs.push_back(points[i * stride + 0]);
        ys.push_back(points[i * stride + 1]);
        zs.push_back(points[i * stride + 2]);
        if (xs.size() >= sample_count) {
            break;
        }
    }

    const float fit_min_x = percentile(xs, 0.01f);
    const float fit_max_x = percentile(xs, 0.99f);
    const float fit_min_y = percentile(ys, 0.01f);
    const float fit_max_y = percentile(ys, 0.99f);
    const float fit_min_z = percentile(zs, 0.01f);
    const float fit_max_z = percentile(zs, 0.99f);

    renderer->center[0] = (fit_min_x + fit_max_x) * 0.5f;
    renderer->center[1] = (fit_min_y + fit_max_y) * 0.5f;
    renderer->center[2] = (fit_min_z + fit_max_z) * 0.5f;
    renderer->min_z = fit_min_z;
    renderer->max_z = fit_max_z;

    const float dx = std::max(0.001f, fit_max_x - fit_min_x);
    const float dy = std::max(0.001f, fit_max_y - fit_min_y);
    const float dz = std::max(0.001f, fit_max_z - fit_min_z);
    const float diagonal = std::sqrt(dx * dx + dy * dy + dz * dz);
    renderer->radius = std::max(0.001f, 0.55f * diagonal);
}

bool upload_interleaved_points(
    DcCloudRenderer *renderer,
    const float *xyzrgb,
    size_t point_count,
    bool already_interleaved)
{
    if (!renderer) {
        return false;
    }
    if (!renderer->initialized) {
        renderer->set_error("renderer is not initialized");
        return false;
    }
    renderer->error.clear();

    renderer->count = point_count;
    if (point_count == 0) {
        return true;
    }
    if (!xyzrgb) {
        renderer->set_error("point buffer is null");
        return false;
    }
    if (!renderer->ensure_capacity(point_count)) {
        return false;
    }

    update_fit(renderer, xyzrgb, point_count, already_interleaved ? kPointStride : 3);

    std::vector<float> interleaved;
    const float *upload_data = xyzrgb;
    if (!already_interleaved) {
        interleaved = make_interleaved_points(xyzrgb, nullptr, point_count);
        upload_data = interleaved.data();
    }

    const size_t bytes = point_count * kPointStride * sizeof(float);
    glBindBuffer(GL_ARRAY_BUFFER, renderer->vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(bytes), upload_data);
    const GLenum cpu_upload_gl_err = glGetError();
    if (cpu_upload_gl_err != GL_NO_ERROR) {
        std::ostringstream out;
        out << "OpenGL VBO upload failed (GL error 0x"
            << std::hex << static_cast<unsigned int>(cpu_upload_gl_err) << ")";
        renderer->set_error(out.str());
        return false;
    }
    renderer->uploaded_once = true;

    return true;
}

extern "C" DcCloudRenderer *dc_cloud_renderer_create(void)
{
    return new DcCloudRenderer();
}

extern "C" void dc_cloud_renderer_destroy(DcCloudRenderer *renderer)
{
    if (!renderer) {
        return;
    }
    renderer->unregister_cuda_buffer();
    if (renderer->vbo) {
        glDeleteBuffers(1, &renderer->vbo);
    }
    if (renderer->vao) {
        glDeleteVertexArrays(1, &renderer->vao);
    }
    if (renderer->program) {
        glDeleteProgram(renderer->program);
    }
    delete renderer;
}

extern "C" int dc_cloud_renderer_initialize(DcCloudRenderer *renderer)
{
    if (!renderer) {
        return 0;
    }
    renderer->error.clear();

    glewExperimental = GL_TRUE;
    const GLenum glew_status = glewInit();
    glGetError();  // GLEW can leave GL_INVALID_ENUM on core profiles.
    if (glew_status != GLEW_OK) {
        renderer->set_error(reinterpret_cast<const char *>(glewGetErrorString(glew_status)));
        return 0;
    }

    if (!GLEW_VERSION_3_3) {
        renderer->set_error("OpenGL 3.3 is required for the point cloud renderer");
        return 0;
    }

    const char *cuda_upload = std::getenv("DATA_COLLECT_CLOUD_RENDERER_CUDA_UPLOAD");
    renderer->prefer_cuda_upload = cuda_upload && std::string(cuda_upload) != "0";
    if (renderer->prefer_cuda_upload) {
        const cudaError_t cuda_set = cudaSetDevice(0);
        if (cuda_set != cudaSuccess) {
            renderer->prefer_cuda_upload = false;
        }
    }

    renderer->program = link_program(renderer->error);
    if (!renderer->program) {
        return 0;
    }

    glGenVertexArrays(1, &renderer->vao);
    if (!renderer->vao) {
        renderer->set_error(gl_error_string("failed to create OpenGL VAO"));
        return 0;
    }

    if (!renderer->ensure_capacity(8192)) {
        return 0;
    }

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_PROGRAM_POINT_SIZE);
    renderer->initialized = true;
    return 1;
}

extern "C" int dc_cloud_renderer_resize(DcCloudRenderer *renderer, int width, int height)
{
    if (!renderer) {
        return 0;
    }
    renderer->width = std::max(1, width);
    renderer->height = std::max(1, height);
    return 1;
}

extern "C" int dc_cloud_renderer_upload_points(
    DcCloudRenderer *renderer,
    const float *xyz,
    size_t point_count)
{
    return upload_interleaved_points(renderer, xyz, point_count, false) ? 1 : 0;
}

extern "C" int dc_cloud_renderer_upload_points_rgb(
    DcCloudRenderer *renderer,
    const float *xyz,
    const float *rgb,
    size_t point_count)
{
    if (!renderer) {
        return 0;
    }
    if (!renderer->initialized) {
        renderer->set_error("renderer is not initialized");
        return 0;
    }
    if (point_count == 0) {
        renderer->count = 0;
        renderer->error.clear();
        return 1;
    }
    if (!xyz) {
        renderer->set_error("point buffer is null");
        return 0;
    }

    std::vector<float> interleaved = make_interleaved_points(xyz, rgb, point_count);
    return upload_interleaved_points(renderer, interleaved.data(), point_count, true) ? 1 : 0;
}

extern "C" int dc_cloud_renderer_upload_points_interleaved(
    DcCloudRenderer *renderer,
    const float *xyzrgb,
    size_t point_count)
{
    return upload_interleaved_points(renderer, xyzrgb, point_count, true) ? 1 : 0;
}

extern "C" int dc_cloud_renderer_draw(
    DcCloudRenderer *renderer,
    float yaw,
    float pitch,
    float distance,
    float pan_x,
    float pan_y,
    float point_size)
{
    if (!renderer) {
        return 0;
    }
    if (!renderer->initialized) {
        renderer->set_error("renderer is not initialized");
        return 0;
    }

    glDisable(GL_SCISSOR_TEST);
    glViewport(0, 0, renderer->width, renderer->height);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_TRUE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glClearColor(0.026f, 0.029f, 0.034f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (renderer->count == 0) {
        return 1;
    }

    const float aspect = static_cast<float>(renderer->width) / static_cast<float>(renderer->height);

    glUseProgram(renderer->program);
    glUniform3fv(glGetUniformLocation(renderer->program, "u_center"), 1, renderer->center);
    glUniform1f(glGetUniformLocation(renderer->program, "u_radius"), renderer->radius);
    glUniform1f(glGetUniformLocation(renderer->program, "u_yaw"), yaw);
    glUniform1f(glGetUniformLocation(renderer->program, "u_pitch"), pitch);
    glUniform2f(glGetUniformLocation(renderer->program, "u_pan"), pan_x, pan_y);
    glUniform1f(glGetUniformLocation(renderer->program, "u_distance"), std::max(0.35f, distance));
    glUniform1f(glGetUniformLocation(renderer->program, "u_aspect"), aspect);
    glUniform1f(glGetUniformLocation(renderer->program, "u_min_z"), renderer->min_z);
    glUniform1f(glGetUniformLocation(renderer->program, "u_max_z"), renderer->max_z);
    glUniform1f(glGetUniformLocation(renderer->program, "u_point_size"), std::max(1.0f, point_size));

    glBindVertexArray(renderer->vao);
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(renderer->count));
    glBindVertexArray(0);
    glUseProgram(0);

    const GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
        std::ostringstream out;
        out << "OpenGL draw failed (GL error 0x" << std::hex << static_cast<unsigned int>(err) << ")";
        renderer->set_error(out.str());
        return 0;
    }
    return 1;
}

extern "C" const char *dc_cloud_renderer_last_error(DcCloudRenderer *renderer)
{
    if (!renderer) {
        return "renderer is null";
    }
    return renderer->error.c_str();
}
