#include "data_collect_cloud_renderer/cloud_renderer.h"

#include <GL/glew.h>
#include <cuda_gl_interop.h>
#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>

namespace {

constexpr const char *kVertexShader = R"GLSL(
#version 330 core
layout(location = 0) in vec3 a_position;

uniform vec3 u_center;
uniform float u_radius;
uniform float u_yaw;
uniform float u_pitch;
uniform vec2 u_pan;
uniform float u_distance;
uniform mat4 u_projection;
uniform float u_min_z;
uniform float u_max_z;
uniform float u_point_size;

out float v_height;

void main()
{
    vec3 p = (a_position - u_center) / max(u_radius, 0.000001);

    float cy = cos(u_yaw);
    float sy = sin(u_yaw);
    p = vec3(cy * p.x - sy * p.y, sy * p.x + cy * p.y, p.z);

    float cp = cos(u_pitch);
    float sp = sin(u_pitch);
    p = vec3(p.x, cp * p.y - sp * p.z, sp * p.y + cp * p.z);

    p.xy += u_pan;
    gl_Position = u_projection * vec4(p.x, p.y, p.z - u_distance, 1.0);
    gl_PointSize = u_point_size;
    v_height = clamp((a_position.z - u_min_z) / max(u_max_z - u_min_z, 0.000001), 0.0, 1.0);
}
)GLSL";

constexpr const char *kFragmentShader = R"GLSL(
#version 330 core
in float v_height;
out vec4 frag_color;

void main()
{
    vec2 p = gl_PointCoord * 2.0 - 1.0;
    if (dot(p, p) > 1.0) {
        discard;
    }

    vec3 low = vec3(0.12, 0.46, 0.92);
    vec3 mid = vec3(0.10, 0.78, 0.58);
    vec3 high = vec3(1.00, 0.70, 0.18);
    vec3 color = v_height < 0.5
        ? mix(low, mid, v_height * 2.0)
        : mix(mid, high, (v_height - 0.5) * 2.0);
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

void perspective(float fovy_radians, float aspect, float near_z, float far_z, float out[16])
{
    std::fill(out, out + 16, 0.0f);
    const float f = 1.0f / std::tan(fovy_radians * 0.5f);
    out[0] = f / std::max(aspect, 0.001f);
    out[5] = f;
    out[10] = (far_z + near_z) / (near_z - far_z);
    out[11] = -1.0f;
    out[14] = (2.0f * far_z * near_z) / (near_z - far_z);
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
            static_cast<GLsizeiptr>(new_capacity * 3 * sizeof(float)),
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
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
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

    glEnable(GL_DEPTH_TEST);
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

    renderer->center[0] = (min_x + max_x) * 0.5f;
    renderer->center[1] = (min_y + max_y) * 0.5f;
    renderer->center[2] = (min_z + max_z) * 0.5f;
    renderer->min_z = min_z;
    renderer->max_z = max_z;
    const float dx = max_x - min_x;
    const float dy = max_y - min_y;
    const float dz = max_z - min_z;
    renderer->radius = std::max(0.001f, 0.5f * std::sqrt(dx * dx + dy * dy + dz * dz));

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
        renderer->set_error(cuda_error_string("CUDA upload to OpenGL VBO failed", err));
        return 0;
    }
    if (unmap_err != cudaSuccess) {
        renderer->set_error(cuda_error_string("cudaGraphicsUnmapResources failed", unmap_err));
        return 0;
    }
    return 1;
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
    glClearColor(0.04f, 0.06f, 0.08f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (renderer->count == 0) {
        return 1;
    }

    float projection[16];
    const float aspect = static_cast<float>(renderer->width) / static_cast<float>(renderer->height);
    perspective(45.0f * 3.1415926535f / 180.0f, aspect, 0.05f, 100.0f, projection);

    glUseProgram(renderer->program);
    glUniform3fv(glGetUniformLocation(renderer->program, "u_center"), 1, renderer->center);
    glUniform1f(glGetUniformLocation(renderer->program, "u_radius"), renderer->radius);
    glUniform1f(glGetUniformLocation(renderer->program, "u_yaw"), yaw);
    glUniform1f(glGetUniformLocation(renderer->program, "u_pitch"), pitch);
    glUniform2f(glGetUniformLocation(renderer->program, "u_pan"), pan_x, pan_y);
    glUniform1f(glGetUniformLocation(renderer->program, "u_distance"), std::max(0.2f, distance));
    glUniformMatrix4fv(glGetUniformLocation(renderer->program, "u_projection"), 1, GL_FALSE, projection);
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
