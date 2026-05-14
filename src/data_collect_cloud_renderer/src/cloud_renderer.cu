#include "data_collect_cloud_renderer/cloud_renderer.h"

#include <GL/glew.h>
#include <cuda_gl_interop.h>
#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>
#include <sstream>
#include <string>

namespace {

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
uniform int u_debug_colors;

out float v_height;
out vec3 v_color;

void main()
{
    vec3 p = a_position;
    if (u_radius > 0.0) {
        p = (a_position - u_center) / max(u_radius, 0.000001);
    }

    float cy = cos(u_yaw);
    float sy = sin(u_yaw);
    p = vec3(cy * p.x - sy * p.y, sy * p.x + cy * p.y, p.z);

    float cp = cos(u_pitch);
    float sp = sin(u_pitch);
    p = vec3(p.x, cp * p.y - sp * p.z, sp * p.y + cp * p.z);

    p.xy += u_pan;
    float view_scale = max(u_distance, 0.08);
    vec2 ndc = vec2(p.x / max(u_aspect, 0.001), p.y) / view_scale;
    float depth = clamp(p.z / 4.0 + 0.5, 0.0, 1.0);
    gl_Position = vec4(ndc, depth * 2.0 - 1.0, 1.0);
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
uniform int u_debug_colors;
out vec4 frag_color;

void main()
{
    if (u_debug_colors != 0) {
        frag_color = vec4(1.0, 1.0, 0.0, 1.0);
        return;
    }

    vec3 height_low = vec3(0.12, 0.46, 0.92);
    vec3 height_mid = vec3(0.10, 0.78, 0.58);
    vec3 height_high = vec3(1.00, 0.70, 0.18);
    vec3 height_color = v_height < 0.5
        ? mix(height_low, height_mid, v_height * 2.0)
        : mix(height_mid, height_high, (v_height - 0.5) * 2.0);
    vec3 color = length(v_color) > 0.001 ? v_color : height_color;
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

void compute_fit(DcCloudRenderer *renderer, const float *xyz, size_t point_count);

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
    bool diagnostic_mode = false;
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
        if (requested <= capacity && vbo && cuda_vbo) {
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
            static_cast<GLsizeiptr>(new_capacity * 6 * sizeof(float)),
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
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(
            1,
            3,
            GL_FLOAT,
            GL_FALSE,
            6 * sizeof(float),
            reinterpret_cast<void *>(3 * sizeof(float)));
        glBindVertexArray(0);

        const cudaError_t err = cudaGraphicsGLRegisterBuffer(
            &cuda_vbo,
            vbo,
            cudaGraphicsRegisterFlagsWriteDiscard);
        if (err != cudaSuccess) {
            cuda_vbo = nullptr;
            set_error(cuda_error_string(
                "CUDA/OpenGL interop failed while registering the VBO. "
                "If this is a hybrid GPU system, start the UI with the NVIDIA OpenGL context",
                err));
            return false;
        }

        capacity = new_capacity;
        return true;
    }
};

std::vector<float> make_debug_points()
{
    std::vector<float> points;
    points.reserve(41 * 41 * 6);
    for (int ix = -20; ix <= 20; ++ix) {
        for (int iy = -20; iy <= 20; ++iy) {
            const float x = static_cast<float>(ix) / 20.0f;
            const float y = static_cast<float>(iy) / 20.0f;
            const float z = 0.18f * std::sin(x * 5.0f) * std::cos(y * 5.0f);
            points.push_back(x);
            points.push_back(y);
            points.push_back(z);
            points.push_back(1.0f);
            points.push_back(1.0f);
            points.push_back(0.0f);
        }
    }
    return points;
}

std::vector<float> make_interleaved_points(const float *xyz, const float *rgb, size_t point_count)
{
    std::vector<float> interleaved;
    interleaved.resize(point_count * 6);
    for (size_t i = 0; i < point_count; ++i) {
        interleaved[i * 6 + 0] = xyz[i * 3 + 0];
        interleaved[i * 6 + 1] = xyz[i * 3 + 1];
        interleaved[i * 6 + 2] = xyz[i * 3 + 2];
        if (rgb) {
            interleaved[i * 6 + 3] = rgb[i * 3 + 0];
            interleaved[i * 6 + 4] = rgb[i * 3 + 1];
            interleaved[i * 6 + 5] = rgb[i * 3 + 2];
        } else {
            interleaved[i * 6 + 3] = 0.0f;
            interleaved[i * 6 + 4] = 0.0f;
            interleaved[i * 6 + 5] = 0.0f;
        }
    }
    return interleaved;
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

    const cudaError_t cuda_set = cudaSetDevice(0);
    if (cuda_set != cudaSuccess) {
        renderer->set_error(cuda_error_string("cudaSetDevice(0) failed", cuda_set));
        return 0;
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

    glDisable(GL_DEPTH_TEST);
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
    if (!renderer) {
        return 0;
    }
    if (!renderer->initialized) {
        renderer->set_error("renderer is not initialized");
        return 0;
    }
    renderer->error.clear();

    renderer->count = point_count;
    if (point_count == 0) {
        return 1;
    }
    if (!xyz) {
        renderer->set_error("point buffer is null");
        return 0;
    }
    if (!renderer->ensure_capacity(point_count)) {
        return 0;
    }

    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();
    float max_z = std::numeric_limits<float>::lowest();
    for (size_t i = 0; i < point_count; ++i) {
        const float x = xyz[i * 3 + 0];
        const float y = xyz[i * 3 + 1];
        const float z = xyz[i * 3 + 2];
        min_x = std::min(min_x, x);
        min_y = std::min(min_y, y);
        min_z = std::min(min_z, z);
        max_x = std::max(max_x, x);
        max_y = std::max(max_y, y);
        max_z = std::max(max_z, z);
    }

    renderer->min_z = min_z;
    renderer->max_z = max_z;

    std::vector<float> xs;
    std::vector<float> ys;
    std::vector<float> zs;
    const size_t sample_count = std::min<size_t>(point_count, 20000);
    const size_t stride = std::max<size_t>(1, point_count / sample_count);
    xs.reserve(sample_count);
    ys.reserve(sample_count);
    zs.reserve(sample_count);
    for (size_t i = 0; i < point_count; i += stride) {
        xs.push_back(xyz[i * 3 + 0]);
        ys.push_back(xyz[i * 3 + 1]);
        zs.push_back(xyz[i * 3 + 2]);
        if (xs.size() >= sample_count) {
            break;
        }
    }
    auto percentile = [](std::vector<float> values, float q) {
        if (values.empty()) {
            return 0.0f;
        }
        const size_t index = std::min(
            values.size() - 1,
            static_cast<size_t>(std::floor(q * static_cast<float>(values.size() - 1))));
        std::nth_element(values.begin(), values.begin() + index, values.end());
        return values[index];
    };
    const float fit_min_x = percentile(xs, 0.08f);
    const float fit_max_x = percentile(xs, 0.92f);
    const float fit_min_y = percentile(ys, 0.08f);
    const float fit_max_y = percentile(ys, 0.92f);
    const float fit_min_z = percentile(zs, 0.08f);
    const float fit_max_z = percentile(zs, 0.92f);
    renderer->center[0] = (fit_min_x + fit_max_x) * 0.5f;
    renderer->center[1] = (fit_min_y + fit_max_y) * 0.5f;
    renderer->center[2] = (fit_min_z + fit_max_z) * 0.5f;
    const float dx = std::max(0.001f, fit_max_x - fit_min_x);
    const float dy = std::max(0.001f, fit_max_y - fit_min_y);
    const float dz = std::max(0.001f, fit_max_z - fit_min_z);
    renderer->radius = std::max(0.001f, 0.36f * std::sqrt(dx * dx + dy * dy + dz * dz));

    glBindBuffer(GL_ARRAY_BUFFER, renderer->vbo);
    glBufferSubData(
        GL_ARRAY_BUFFER,
        0,
        static_cast<GLsizeiptr>(point_count * 3 * sizeof(float)),
        xyz);
    const GLenum cpu_upload_gl_err = glGetError();
    if (cpu_upload_gl_err != GL_NO_ERROR) {
        std::ostringstream out;
        out << "OpenGL VBO upload failed (GL error 0x"
            << std::hex << static_cast<unsigned int>(cpu_upload_gl_err) << ")";
        renderer->set_error(out.str());
        return 0;
    }
    renderer->uploaded_once = true;

    if (!renderer->prefer_cuda_upload) {
        return 1;
    }

    cudaError_t err = cudaGraphicsMapResources(1, &renderer->cuda_vbo, 0);
    if (err != cudaSuccess) {
        renderer->set_error(cuda_error_string("cudaGraphicsMapResources failed", err));
        return 0;
    }

    void *device_ptr = nullptr;
    size_t mapped_size = 0;
    err = cudaGraphicsResourceGetMappedPointer(&device_ptr, &mapped_size, renderer->cuda_vbo);
    if (err == cudaSuccess) {
        const size_t bytes = point_count * 3 * sizeof(float);
        if (mapped_size < bytes) {
            err = cudaErrorInvalidValue;
        } else {
            err = cudaMemcpy(device_ptr, xyz, bytes, cudaMemcpyHostToDevice);
        }
    }

    const cudaError_t unmap_err = cudaGraphicsUnmapResources(1, &renderer->cuda_vbo, 0);
    if (err != cudaSuccess) {
        renderer->set_error(cuda_error_string("CUDA upload to OpenGL VBO failed after OpenGL fallback upload", err));
        return 0;
    }
    if (unmap_err != cudaSuccess) {
        renderer->set_error(cuda_error_string("cudaGraphicsUnmapResources failed", unmap_err));
        return 0;
    }
    return 1;
}

extern "C" int dc_cloud_renderer_show_debug_points(DcCloudRenderer *renderer)
{
    if (!renderer) {
        return 0;
    }
    if (!renderer->initialized) {
        renderer->set_error("renderer is not initialized");
        return 0;
    }
    static const std::vector<float> debug_points = make_debug_points();
    if (!renderer->ensure_capacity(debug_points.size() / 3)) {
        return 0;
    }
    glBindBuffer(GL_ARRAY_BUFFER, renderer->vbo);
    glBufferSubData(
        GL_ARRAY_BUFFER,
        0,
        static_cast<GLsizeiptr>(debug_points.size() * sizeof(float)),
        debug_points.data());
    const GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
        std::ostringstream out;
        out << "OpenGL debug VBO upload failed (GL error 0x"
            << std::hex << static_cast<unsigned int>(err) << ")";
        renderer->set_error(out.str());
        return 0;
    }
    renderer->center[0] = 0.0f;
    renderer->center[1] = 0.0f;
    renderer->center[2] = 0.0f;
    renderer->min_z = -0.2f;
    renderer->max_z = 0.2f;
    renderer->radius = -1.0f;
    renderer->count = debug_points.size() / 3;
    renderer->uploaded_once = true;
    return 1;
}

extern "C" void dc_cloud_renderer_set_diagnostic_mode(DcCloudRenderer *renderer, int enabled)
{
    if (!renderer) {
        return;
    }
    renderer->diagnostic_mode = enabled != 0;
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

    glViewport(0, 0, renderer->width, renderer->height);
    glClearColor(0.015f, 0.020f, 0.025f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (renderer->diagnostic_mode) {
        const int box_w = std::max(24, renderer->width / 3);
        const int box_h = std::max(24, renderer->height / 3);
        const int box_x = std::max(0, (renderer->width - box_w) / 2);
        const int box_y = std::max(0, (renderer->height - box_h) / 2);
        glEnable(GL_SCISSOR_TEST);
        glScissor(box_x, box_y, box_w, box_h);
        glClearColor(1.0f, 0.12f, 0.02f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glDisable(GL_SCISSOR_TEST);
        glClearColor(0.015f, 0.020f, 0.025f, 1.0f);
        return 1;
    }

    bool drawing_debug_points = false;
    if (renderer->count == 0 && !renderer->uploaded_once) {
        static const std::vector<float> debug_points = make_debug_points();
        if (!renderer->ensure_capacity(debug_points.size() / 3)) {
            return 0;
        }
        glBindBuffer(GL_ARRAY_BUFFER, renderer->vbo);
        glBufferSubData(
            GL_ARRAY_BUFFER,
            0,
            static_cast<GLsizeiptr>(debug_points.size() * sizeof(float)),
            debug_points.data());
        renderer->count = debug_points.size() / 3;
        drawing_debug_points = true;
    } else if (renderer->count == 0) {
        return 1;
    }

    const float aspect = static_cast<float>(renderer->width) / static_cast<float>(renderer->height);

    glUseProgram(renderer->program);
    const float debug_center[3] = {0.0f, 0.0f, 0.0f};
    glUniform3fv(
        glGetUniformLocation(renderer->program, "u_center"),
        1,
        drawing_debug_points ? debug_center : renderer->center);
    glUniform1f(
        glGetUniformLocation(renderer->program, "u_radius"),
        drawing_debug_points ? -1.0f : renderer->radius);
    glUniform1f(glGetUniformLocation(renderer->program, "u_yaw"), yaw);
    glUniform1f(glGetUniformLocation(renderer->program, "u_pitch"), pitch);
    glUniform2f(glGetUniformLocation(renderer->program, "u_pan"), pan_x, pan_y);
    glUniform1f(glGetUniformLocation(renderer->program, "u_distance"), std::max(0.15f, distance));
    glUniform1f(glGetUniformLocation(renderer->program, "u_aspect"), aspect);
    glUniform1f(glGetUniformLocation(renderer->program, "u_min_z"), renderer->min_z);
    glUniform1f(glGetUniformLocation(renderer->program, "u_max_z"), renderer->max_z);
    glUniform1f(glGetUniformLocation(renderer->program, "u_point_size"), std::max(2.0f, point_size));
    glUniform1i(glGetUniformLocation(renderer->program, "u_debug_colors"), drawing_debug_points ? 1 : 0);

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
