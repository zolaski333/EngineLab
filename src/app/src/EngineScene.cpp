#include <enginelab/app/EngineScene.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>

namespace enginelab::ui {
using namespace juce::gl;
using render::Mat4;
using render::SceneMaterial;
using render::Vec3;

namespace {
// ------------------------------------------------------------------ shaders
// Every part's transform and material live in a buffer texture, six texels
// per part (4 matrix columns, colour + metalness, roughness / opacity / rim /
// glow), so a whole material group is one draw call.
constexpr const char* meshVertexSource = R"(#version 150
in vec3 aPosition;
in vec3 aNormal;
in float aPart;
uniform samplerBuffer uParts;
uniform mat4 uViewProjection;
uniform int uSkipDark;
out vec3 vWorld;
out vec3 vNormal;
flat out int vPart;
void main() {
    int base = int(aPart + 0.5) * 6;
    mat4 model = mat4(texelFetch(uParts, base), texelFetch(uParts, base + 1),
                      texelFetch(uParts, base + 2), texelFetch(uParts, base + 3));
    vec4 world = model * vec4(aPosition, 1.0);
    vWorld = world.xyz;
    vNormal = mat3(model) * aNormal;
    vPart = base;
    gl_Position = uViewProjection * world;
    // An unlit flame is moved outside the clip volume instead of drawn black.
    if (uSkipDark == 1 && texelFetch(uParts, base + 5).w < 0.003) gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
}
)";

// Metals, pistons and shells. Two directional lights as in the prototype
// (warm key, cool rim), a hemisphere fill, a procedural studio environment for
// reflections, the burning cylinders as point lights, then ACES tone mapping.
// The output is premultiplied by its opacity.
constexpr const char* surfaceFragmentSource = R"(#version 150
in vec3 vWorld;
in vec3 vNormal;
flat in int vPart;
uniform samplerBuffer uParts;
uniform vec3 uEye;
uniform int uLightCount;
uniform vec4 uLightPosition[8];
uniform vec3 uLightColour[8];
out vec4 fragColour;

const float pi = 3.14159265;
const vec3 keyDirection = vec3(0.5657, 0.7071, 0.4243);
const vec3 rimDirection = vec3(-0.5571, 0.3714, -0.7428);

float distributionGgx(float nh, float a) {
    float a2 = a * a;
    float d = nh * nh * (a2 - 1.0) + 1.0;
    return a2 / (pi * d * d);
}

float visibilitySmith(float nv, float nl, float a) {
    float k = a * 0.5;
    float gv = nv / (nv * (1.0 - k) + k);
    float gl = nl / (nl * (1.0 - k) + k);
    return gv * gl / max(4.0 * nv * nl, 1e-4);
}

vec3 fresnelSchlick(float c, vec3 f0) {
    return f0 + (vec3(1.0) - f0) * pow(1.0 - c, 5.0);
}

vec3 shade(vec3 n, vec3 v, vec3 l, vec3 radiance, vec3 base, vec3 f0, float a, float metal) {
    float nl = dot(n, l);
    if (nl <= 0.0) return vec3(0.0);
    vec3 h = normalize(l + v);
    float nv = max(dot(n, v), 1e-3);
    vec3 f = fresnelSchlick(max(dot(h, v), 0.0), f0);
    vec3 specular = f * distributionGgx(max(dot(n, h), 0.0), a) * visibilitySmith(nv, nl, a);
    vec3 diffuse = (vec3(1.0) - f) * (1.0 - metal) * base / pi;
    return (diffuse + specular) * radiance * nl * pi;
}

vec3 studio(vec3 r, float rough) {
    vec3 c = mix(vec3(0.010, 0.013, 0.012), vec3(0.26, 0.31, 0.30), smoothstep(-0.25, 0.85, r.y));
    float sharpness = mix(40.0, 2.5, rough);
    float gain = mix(1.0, 0.35, rough);
    c += vec3(1.0, 0.95, 0.88) * pow(max(dot(r, vec3(0.57, 0.70, 0.43)), 0.0), sharpness) * 2.4 * gain;
    c += vec3(0.40, 0.72, 0.88) * pow(max(dot(r, vec3(-0.62, 0.38, -0.69)), 0.0), sharpness) * 0.9 * gain;
    c += vec3(0.85) * pow(max(dot(r, vec3(0.10, 0.25, 0.96)), 0.0), sharpness) * 1.1 * gain;
    return c;
}

vec3 environmentBrdf(vec3 f0, float rough, float nv) {
    const vec4 c0 = vec4(-1.0, -0.0275, -0.572, 0.022);
    const vec4 c1 = vec4(1.0, 0.0425, 1.04, -0.04);
    vec4 r = rough * c0 + c1;
    float a004 = min(r.x * r.x, exp2(-9.28 * nv)) * r.x + r.y;
    vec2 ab = vec2(-1.04, 1.04) * a004 + r.zw;
    return f0 * ab.x + ab.y;
}

vec3 aces(vec3 x) {
    return clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0);
}

void main() {
    vec4 look = texelFetch(uParts, vPart + 4);
    vec4 finish = texelFetch(uParts, vPart + 5);
    vec3 uBaseColour = look.rgb;
    float uMetalness = look.a;
    float uRoughness = finish.x;
    float uOpacity = finish.y;
    float uRim = finish.z;
    vec3 uEmissive = look.rgb * finish.w;
    vec3 n = normalize(vNormal);
    vec3 v = normalize(uEye - vWorld);
    float facing = dot(n, v);
    if (facing < 0.0) { n = -n; facing = -facing; }
    float rough = clamp(uRoughness, 0.05, 1.0);
    float a = rough * rough;
    vec3 f0 = mix(vec3(0.04), uBaseColour, uMetalness);

    vec3 colour = shade(n, v, keyDirection, vec3(1.0, 0.945, 0.88) * 1.6, uBaseColour, f0, a, uMetalness);
    colour += shade(n, v, rimDirection, vec3(0.435, 0.78, 0.90) * 0.7, uBaseColour, f0, a, uMetalness);
    for (int i = 0; i < uLightCount; ++i) {
        vec3 toLight = uLightPosition[i].xyz - vWorld;
        float d = length(toLight);
        float fall = clamp(1.0 - d / uLightPosition[i].w, 0.0, 1.0);
        colour += shade(n, v, toLight / max(d, 1e-3), uLightColour[i] * fall * fall, uBaseColour, f0, a, uMetalness);
    }
    vec3 hemisphere = mix(vec3(0.004, 0.005, 0.005), vec3(0.35, 0.46, 0.43), 0.5 + 0.5 * n.y) * 0.35;
    colour += hemisphere * uBaseColour * (1.0 - uMetalness);
    colour += studio(reflect(-v, n), rough) * environmentBrdf(f0, rough, facing);
    colour += studio(n, 1.0) * 0.25 * uBaseColour * (1.0 - uMetalness);
    colour += uEmissive;

    vec3 display = pow(aces(colour * 1.05), vec3(1.0 / 2.2));
    float alpha = clamp(uOpacity + uRim * pow(1.0 - facing, 3.0), 0.0, 1.0);
    fragColour = vec4(display * alpha, alpha);
}
)";

// Burning charge: additive, brighter where the surface faces the viewer.
constexpr const char* flameFragmentSource = R"(#version 150
in vec3 vWorld;
in vec3 vNormal;
flat in int vPart;
uniform samplerBuffer uParts;
uniform vec3 uEye;
uniform vec3 uColour;
out vec4 fragColour;
void main() {
    float intensity = texelFetch(uParts, vPart + 5).w;
    float facing = abs(dot(normalize(vNormal), normalize(uEye - vWorld)));
    fragColour = vec4(uColour * intensity * (0.25 + 0.85 * pow(facing, 1.5)), 0.0);
}
)";

// A triangle covering the viewport, without any vertex buffer.
constexpr const char* screenVertexSource = R"(#version 150
void main() {
    vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)";

// The viewport's radial gradient (CSS "ellipse at 50% 40%, #18211f 0%,
// #0b0f0e 70%"), dithered so the dark ramp does not band.
constexpr const char* backgroundFragmentSource = R"(#version 150
uniform vec2 uSize;
out vec4 fragColour;
void main() {
    vec2 uv = gl_FragCoord.xy / uSize;
    vec2 q = (uv - vec2(0.5, 0.6)) / vec2(0.901, 0.721);
    float t = clamp(length(q) / 0.7, 0.0, 1.0);
    vec3 c = mix(vec3(0.0941, 0.1294, 0.1216), vec3(0.0431, 0.0588, 0.0549), t);
    float noise = fract(sin(dot(gl_FragCoord.xy, vec2(12.9898, 78.233))) * 43758.5453);
    fragColour = vec4(c + (noise - 0.5) / 255.0, 1.0);
}
)";

constexpr const char* copyFragmentSource = R"(#version 150
uniform sampler2D uTexture;
uniform float uWeight;
out vec4 fragColour;
void main() {
    fragColour = texelFetch(uTexture, ivec2(gl_FragCoord.xy), 0) * uWeight;
}
)";

constexpr const char* floorVertexSource = R"(#version 150
in vec3 aPosition;
uniform mat4 uViewProjection;
out vec3 vWorld;
void main() {
    vWorld = aPosition;
    gl_Position = uViewProjection * vec4(aPosition, 1.0);
}
)";

// Mode 0: polar grid lines fading towards the rim. Mode 1: soft contact shadow.
constexpr const char* floorFragmentSource = R"(#version 150
in vec3 vWorld;
uniform int uMode;
uniform vec4 uColour;
uniform vec3 uCentre;
uniform vec2 uExtent;
uniform float uRadius;
out vec4 fragColour;
void main() {
    vec2 offset = vWorld.xz - uCentre.xz;
    if (uMode == 0) {
        fragColour = vec4(uColour.rgb, uColour.a * (1.0 - smoothstep(0.45 * uRadius, uRadius, length(offset))));
    } else {
        float r = length(offset / uExtent);
        fragColour = vec4(uColour.rgb, uColour.a * (1.0 - smoothstep(0.1, 1.0, r)));
    }
}
)";

// ---------------------------------------------------------------- materials
struct MaterialLook final {
    std::uint32_t srgb;
    float metalness;
    float roughness;
    /** Opacity of a shell in X-ray mode, or of a flow duct. */
    float opacity;
};

// Colours, metalness and roughness of the prototype's three.js materials.
constexpr std::array<MaterialLook, render::sceneMaterialCount> looks { {
    { 0xc9d1ce, 0.95F, 0.28F, 1.0F },   // steel
    { 0x8e9895, 0.90F, 0.40F, 1.0F },   // forged
    { 0xdfe4e2, 0.80F, 0.30F, 1.0F },   // piston
    { 0x9fdcef, 0.85F, 0.30F, 1.0F },   // intake valve
    { 0xe8b9a2, 0.85F, 0.35F, 1.0F },   // exhaust valve
    { 0x6f8a84, 0.20F, 0.55F, 0.15F },  // block shell
    { 0x5d736e, 0.20F, 0.55F, 0.13F },  // head shell
    { 0x41b6d7, 0.40F, 0.40F, 0.55F },  // intake flow
    { 0xa3958b, 0.55F, 0.38F, 1.0F },   // exhaust pipe
    { 0xff7a3d, 0.00F, 1.00F, 0.85F },  // flame
} };

[[nodiscard]] Vec3 linearColour(std::uint32_t srgb) noexcept {
    const auto channel = [](std::uint32_t value) {
        return std::pow(static_cast<float>(value & 0xffU) / 255.0F, 2.2F);
    };
    return { channel(srgb >> 16U), channel(srgb >> 8U), channel(srgb) };
}

[[nodiscard]] Vec3 displayColour(std::uint32_t srgb) noexcept {
    return { static_cast<float>((srgb >> 16U) & 0xffU) / 255.0F, static_cast<float>((srgb >> 8U) & 0xffU) / 255.0F,
             static_cast<float>(srgb & 0xffU) / 255.0F };
}

[[nodiscard]] bool isShell(SceneMaterial material) noexcept {
    return material == SceneMaterial::blockShell || material == SceneMaterial::headShell;
}

/** Draw pass of a part: 0 hidden, 1 opaque, 2..4 translucent (back to front
    by family), 5 additive flame. */
[[nodiscard]] int passOf(SceneMaterial material, std::uint8_t layers, const SceneFrame& frame) noexcept {
    switch (frame.layer) {
    case SceneLayerMode::all: break;
    case SceneLayerMode::combustion:
        if ((layers & render::layerIntake) != 0U) return 0;
        break;
    case SceneLayerMode::mechanical:
        if ((layers & (render::layerIntake | render::layerCombustion)) != 0U) return 0;
        break;
    case SceneLayerMode::gasFlow:
        if ((layers & (render::layerValves | render::layerCombustion)) != 0U) return 0;
        break;
    }
    if (material == SceneMaterial::flame) return 5;
    if (isShell(material)) return frame.xray ? 4 : 1;
    if (material == SceneMaterial::intakeFlow) return 3;
    if (material == SceneMaterial::exhaustPipe) return 1;
    // The gas-flow layer fades the metal so the ducts read through it.
    return frame.layer == SceneLayerMode::gasFlow ? 2 : 1;
}

// ------------------------------------------------------------------ helpers
[[nodiscard]] GLuint compileShader(GLenum type, const char* source, juce::String& error) {
    const auto shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    GLint status = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    if (status == GL_TRUE) return shader;
    std::array<GLchar, 2048> log {};
    GLsizei length = 0;
    glGetShaderInfoLog(shader, static_cast<GLsizei>(log.size()), &length, log.data());
    error = juce::String::fromUTF8(log.data(), length);
    glDeleteShader(shader);
    return 0;
}

[[nodiscard]] GLuint linkProgram(const char* vertexSource, const char* fragmentSource, juce::String& error) {
    const auto vertex = compileShader(GL_VERTEX_SHADER, vertexSource, error);
    if (vertex == 0) return 0;
    const auto fragment = compileShader(GL_FRAGMENT_SHADER, fragmentSource, error);
    if (fragment == 0) {
        glDeleteShader(vertex);
        return 0;
    }
    const auto program = glCreateProgram();
    glAttachShader(program, vertex);
    glAttachShader(program, fragment);
    glBindAttribLocation(program, 0, "aPosition");
    glBindAttribLocation(program, 1, "aNormal");
    glBindAttribLocation(program, 2, "aPart");
    glBindFragDataLocation(program, 0, "fragColour");
    glLinkProgram(program);
    glDeleteShader(vertex);
    glDeleteShader(fragment);
    GLint status = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &status);
    if (status == GL_TRUE) return program;
    std::array<GLchar, 2048> log {};
    GLsizei length = 0;
    glGetProgramInfoLog(program, static_cast<GLsizei>(log.size()), &length, log.data());
    error = juce::String::fromUTF8(log.data(), length);
    glDeleteProgram(program);
    return 0;
}

void setMatrix(GLint location, const Mat4& matrix) {
    glUniformMatrix4fv(location, 1, GL_FALSE, matrix.m.data());
}

void setVec3(GLint location, Vec3 value) {
    glUniform3f(location, value.x, value.y, value.z);
}
} // namespace

struct EngineSceneRenderer::Impl final {
    /** Parts sharing a material and layer set, drawn with one call. */
    struct Group final {
        SceneMaterial material {};
        std::uint8_t layers {};
        GLsizei count {};
        std::uintptr_t offset {};
    };
    struct GpuVertex final {
        Vec3 position;
        Vec3 normal;
        float part {};
    };
    static constexpr std::size_t texelsPerPart = 6;

    GLuint surface {}, flame {}, background {}, copyProgram {}, floorProgram {};
    GLint surfaceViewProjection {}, surfaceEye {}, surfaceSkipDark {}, lightCount {}, lightPosition {}, lightColour {};
    GLint flameViewProjection {}, flameEye {}, flameColour {}, flameSkipDark {};
    GLint backgroundSize {};
    GLint copyTexture {}, copyWeight {};
    GLint floorViewProjection {}, floorMode {}, floorColour {}, floorCentreUniform {}, floorExtent {}, floorRadiusUniform {};

    GLuint emptyVao {}, meshVao {}, meshVbo {}, meshEbo {}, floorVao {}, floorVbo {};
    GLuint partBuffer {}, partTexture {};
    std::vector<Group> groups;
    std::vector<float> partData;
    GLsizei radialLineVertices {}, circleLineVertices {}, shadowVertices {};
    Vec3 floorCentre {};
    float floorRadius {};
    float shadowExtentX {}, shadowExtentZ {};
    float flameLightRange {};

    GLuint msaaFbo {}, msaaColour {}, msaaDepth {}, resolveFbo {}, resolveTexture {}, accumFbo {}, accumTexture {};
    int targetWidth {}, targetHeight {}, targetSamples { -1 };
    int maximumSamples {};

    std::vector<render::SceneInstance> instances;
    std::array<float, 32> lightPositions {};
    std::array<float, 24> lightColours {};

    void destroyTargets() {
        const std::array<GLuint, 3> framebuffers { msaaFbo, resolveFbo, accumFbo };
        glDeleteFramebuffers(3, framebuffers.data());
        const std::array<GLuint, 2> renderbuffers { msaaColour, msaaDepth };
        glDeleteRenderbuffers(2, renderbuffers.data());
        const std::array<GLuint, 2> textures { resolveTexture, accumTexture };
        glDeleteTextures(2, textures.data());
        msaaFbo = msaaColour = msaaDepth = resolveFbo = resolveTexture = accumFbo = accumTexture = 0;
        targetWidth = targetHeight = 0;
        targetSamples = -1;
    }

    static GLuint makeTexture(GLint internalFormat, int width, int height, GLenum type) {
        GLuint texture = 0;
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, width, height, 0, GL_RGBA, type, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);
        return texture;
    }

    void ensureTargets(int width, int height, int samples) {
        if (width == targetWidth && height == targetHeight && samples == targetSamples) return;
        destroyTargets();
        targetWidth = width;
        targetHeight = height;
        targetSamples = samples;

        glGenFramebuffers(1, &msaaFbo);
        glBindFramebuffer(GL_FRAMEBUFFER, msaaFbo);
        glGenRenderbuffers(1, &msaaColour);
        glBindRenderbuffer(GL_RENDERBUFFER, msaaColour);
        glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_RGBA8, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, msaaColour);
        glGenRenderbuffers(1, &msaaDepth);
        glBindRenderbuffer(GL_RENDERBUFFER, msaaDepth);
        glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_DEPTH_COMPONENT24, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, msaaDepth);
        glBindRenderbuffer(GL_RENDERBUFFER, 0);

        resolveTexture = makeTexture(GL_RGBA8, width, height, GL_UNSIGNED_BYTE);
        glGenFramebuffers(1, &resolveFbo);
        glBindFramebuffer(GL_FRAMEBUFFER, resolveFbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, resolveTexture, 0);

        accumTexture = makeTexture(GL_RGBA16F, width, height, GL_FLOAT);
        glGenFramebuffers(1, &accumFbo);
        glBindFramebuffer(GL_FRAMEBUFFER, accumFbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, accumTexture, 0);
    }

    void destroyGeometry() {
        const std::array<GLuint, 4> buffers { meshVbo, meshEbo, floorVbo, partBuffer };
        glDeleteBuffers(4, buffers.data());
        const std::array<GLuint, 2> arrays { meshVao, floorVao };
        glDeleteVertexArrays(2, arrays.data());
        if (partTexture != 0) glDeleteTextures(1, &partTexture);
        meshVbo = meshEbo = floorVbo = partBuffer = meshVao = floorVao = partTexture = 0;
        groups.clear();
    }

    void upload(const render::EngineModel3D& model) {
        destroyGeometry();
        const auto& parts = model.parts();
        // Index buffer ordered by group, so each group is one contiguous range.
        std::vector<std::size_t> order(parts.size());
        for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
        std::stable_sort(order.begin(), order.end(), [&parts](std::size_t a, std::size_t b) {
            return std::pair(parts[a].material, parts[a].layers) < std::pair(parts[b].material, parts[b].layers);
        });
        std::vector<GpuVertex> vertices;
        std::vector<std::uint32_t> indices;
        for (const auto index : order) {
            const auto& part = parts[index];
            if (groups.empty() || groups.back().material != part.material || groups.back().layers != part.layers)
                groups.push_back({ part.material, part.layers, 0,
                                   static_cast<std::uintptr_t>(indices.size() * sizeof(std::uint32_t)) });
            const auto base = static_cast<std::uint32_t>(vertices.size());
            for (const auto& vertex : part.mesh.vertices)
                vertices.push_back({ vertex.position, vertex.normal, static_cast<float>(index) });
            for (const auto i : part.mesh.indices) indices.push_back(base + i);
            groups.back().count += static_cast<GLsizei>(part.mesh.indices.size());
        }
        glGenVertexArrays(1, &meshVao);
        glBindVertexArray(meshVao);
        glGenBuffers(1, &meshVbo);
        glBindBuffer(GL_ARRAY_BUFFER, meshVbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(GpuVertex)), vertices.data(),
                     GL_STATIC_DRAW);
        glGenBuffers(1, &meshEbo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, meshEbo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(indices.size() * sizeof(std::uint32_t)),
                     indices.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GpuVertex), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GpuVertex),
                              reinterpret_cast<const void*>(offsetof(GpuVertex, normal)));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(GpuVertex),
                              reinterpret_cast<const void*>(offsetof(GpuVertex, part)));
        glBindVertexArray(0);

        partData.assign(parts.size() * texelsPerPart * 4, 0.0F);
        glGenBuffers(1, &partBuffer);
        glBindBuffer(GL_TEXTURE_BUFFER, partBuffer);
        glBufferData(GL_TEXTURE_BUFFER, static_cast<GLsizeiptr>(partData.size() * sizeof(float)), nullptr,
                     GL_STREAM_DRAW);
        glGenTextures(1, &partTexture);
        glBindTexture(GL_TEXTURE_BUFFER, partTexture);
        glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, partBuffer);
        glBindTexture(GL_TEXTURE_BUFFER, 0);
        glBindBuffer(GL_TEXTURE_BUFFER, 0);

        // Floor: polar grid (16 spokes, 12 rings) and the contact-shadow
        // quad, under the whole system so the exhaust never sinks into it.
        const auto& bounds = model.systemBounds();
        const auto diagonal = bounds.diagonal();
        const auto centre = bounds.centre();
        const auto y = bounds.minimum.y - 0.03F * diagonal;
        floorCentre = { centre.x, y, centre.z };
        floorRadius = 0.95F * diagonal;
        shadowExtentX = 0.75F * (bounds.maximum.x - bounds.minimum.x) + 0.05F * diagonal;
        shadowExtentZ = 0.62F * (bounds.maximum.z - bounds.minimum.z) + 0.05F * diagonal;
        float bore = 0.0F;
        for (const auto& cylinder : model.config().cylinders) bore = std::max(bore, static_cast<float>(cylinder.boreMm));
        flameLightRange = 6.0F * bore;

        std::vector<Vec3> lines;
        constexpr auto twoPi = 2.0F * std::numbers::pi_v<float>;
        for (int spoke = 0; spoke < 16; ++spoke) {
            const auto a = twoPi * static_cast<float>(spoke) / 16.0F;
            lines.push_back(floorCentre);
            lines.push_back(floorCentre + Vec3 { std::cos(a), 0.0F, std::sin(a) } * floorRadius);
        }
        radialLineVertices = static_cast<GLsizei>(lines.size());
        for (int ring = 1; ring <= 12; ++ring) {
            const auto r = floorRadius * static_cast<float>(ring) / 12.0F;
            for (int segment = 0; segment < 96; ++segment) {
                const auto a0 = twoPi * static_cast<float>(segment) / 96.0F;
                const auto a1 = twoPi * static_cast<float>(segment + 1) / 96.0F;
                lines.push_back(floorCentre + Vec3 { std::cos(a0), 0.0F, std::sin(a0) } * r);
                lines.push_back(floorCentre + Vec3 { std::cos(a1), 0.0F, std::sin(a1) } * r);
            }
        }
        circleLineVertices = static_cast<GLsizei>(lines.size()) - radialLineVertices;
        const auto lift = Vec3 { 0.0F, 0.002F * diagonal, 0.0F };
        const Vec3 c0 = floorCentre + lift + Vec3 { -shadowExtentX, 0.0F, -shadowExtentZ };
        const Vec3 c1 = floorCentre + lift + Vec3 { shadowExtentX, 0.0F, -shadowExtentZ };
        const Vec3 c2 = floorCentre + lift + Vec3 { shadowExtentX, 0.0F, shadowExtentZ };
        const Vec3 c3 = floorCentre + lift + Vec3 { -shadowExtentX, 0.0F, shadowExtentZ };
        for (const auto& corner : { c0, c1, c2, c0, c2, c3 }) lines.push_back(corner);
        shadowVertices = 6;

        glGenVertexArrays(1, &floorVao);
        glBindVertexArray(floorVao);
        glGenBuffers(1, &floorVbo);
        glBindBuffer(GL_ARRAY_BUFFER, floorVbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(lines.size() * sizeof(Vec3)), lines.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), nullptr);
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    /** Writes every part's transform and material for this sub-frame. */
    void writePartData(const render::EngineModel3D& model, const SceneFrame& frame) {
        for (const auto& instance : instances) {
            const auto& part = model.parts()[instance.part];
            const auto pass = passOf(part.material, part.layers, frame);
            const auto& look = looks[static_cast<std::size_t>(part.material)];
            const auto base = linearColour(look.srgb);
            auto opacity = 1.0F;
            auto rim = 0.0F;
            auto glow = 0.0F;
            if (pass == 2) opacity = 0.22F;
            if (pass == 3) {
                opacity = look.opacity;
                glow = 0.08F + 1.4F * instance.intensity;
            }
            if (pass == 4) {
                opacity = look.opacity;
                rim = 0.22F;
            }
            if (pass == 5) glow = instance.intensity;
            auto* out = partData.data() + static_cast<std::size_t>(instance.part) * texelsPerPart * 4;
            std::copy(instance.transform.m.begin(), instance.transform.m.end(), out);
            out[16] = base.x;
            out[17] = base.y;
            out[18] = base.z;
            out[19] = look.metalness;
            out[20] = look.roughness;
            out[21] = opacity;
            out[22] = rim;
            out[23] = glow;
        }
        glBindBuffer(GL_TEXTURE_BUFFER, partBuffer);
        const auto bytes = static_cast<GLsizeiptr>(partData.size() * sizeof(float));
        glBufferData(GL_TEXTURE_BUFFER, bytes, nullptr, GL_STREAM_DRAW);
        glBufferSubData(GL_TEXTURE_BUFFER, 0, bytes, partData.data());
        glBindBuffer(GL_TEXTURE_BUFFER, 0);
    }

    void drawScene(const render::EngineModel3D& model, const SceneFrame& frame, int width, int height) {
        writePartData(model, frame);
        const auto viewProjection = frame.projection * frame.view;
        glViewport(0, 0, width, height);
        glClearColor(0.0F, 0.0F, 0.0F, 1.0F);
        glClearDepth(1.0);
        glDepthMask(GL_TRUE);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glDisable(GL_CULL_FACE);

        glDisable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);
        glUseProgram(background);
        glUniform2f(backgroundSize, static_cast<float>(width), static_cast<float>(height));
        glBindVertexArray(emptyVao);
        glDrawArrays(GL_TRIANGLES, 0, 3);

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glUseProgram(floorProgram);
        setMatrix(floorViewProjection, viewProjection);
        setVec3(floorCentreUniform, floorCentre);
        glUniform1f(floorRadiusUniform, floorRadius);
        glUniform2f(floorExtent, shadowExtentX, shadowExtentZ);
        glBindVertexArray(floorVao);
        glUniform1i(floorMode, 1);
        glUniform4f(floorColour, 0.0F, 0.0F, 0.0F, 0.45F);
        glDrawArrays(GL_TRIANGLES, radialLineVertices + circleLineVertices, shadowVertices);
        glUniform1i(floorMode, 0);
        glUniform4f(floorColour, 0.122F, 0.165F, 0.153F, 1.0F);
        glDrawArrays(GL_LINES, 0, radialLineVertices);
        glUniform4f(floorColour, 0.106F, 0.141F, 0.133F, 1.0F);
        glDrawArrays(GL_LINES, radialLineVertices, circleLineVertices);

        // Each burning cylinder lights its surroundings.
        int lights = 0;
        const auto showFlames = frame.layer == SceneLayerMode::all || frame.layer == SceneLayerMode::combustion;
        if (showFlames) {
            for (const auto& instance : instances) {
                if (lights >= 8) break;
                if (model.parts()[instance.part].material != SceneMaterial::flame || instance.intensity < 0.01F) continue;
                const auto slot = static_cast<std::size_t>(lights);
                lightPositions[slot * 4 + 0] = instance.transform.at(0, 3);
                lightPositions[slot * 4 + 1] = instance.transform.at(1, 3);
                lightPositions[slot * 4 + 2] = instance.transform.at(2, 3);
                lightPositions[slot * 4 + 3] = flameLightRange;
                lightColours[slot * 3 + 0] = 2.6F * instance.intensity;
                lightColours[slot * 3 + 1] = 1.4F * instance.intensity;
                lightColours[slot * 3 + 2] = 0.75F * instance.intensity;
                ++lights;
            }
        }

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_BUFFER, partTexture);
        glUseProgram(surface);
        setMatrix(surfaceViewProjection, viewProjection);
        setVec3(surfaceEye, frame.eye);
        glUniform1i(surfaceSkipDark, 0);
        glUniform1i(lightCount, lights);
        glUniform4fv(lightPosition, 8, lightPositions.data());
        glUniform3fv(lightColour, 8, lightColours.data());
        glBindVertexArray(meshVao);

        const auto drawPass = [this, &frame](int pass) {
            for (const auto& group : groups)
                if (passOf(group.material, group.layers, frame) == pass)
                    glDrawElements(GL_TRIANGLES, group.count, GL_UNSIGNED_INT, reinterpret_cast<const void*>(group.offset));
        };

        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        drawPass(1);

        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
        drawPass(2);
        drawPass(3);
        drawPass(4);

        glBlendFunc(GL_ONE, GL_ONE);
        glUseProgram(flame);
        setMatrix(flameViewProjection, viewProjection);
        setVec3(flameEye, frame.eye);
        glUniform1i(flameSkipDark, 1);
        const auto& flameLook = looks[static_cast<std::size_t>(SceneMaterial::flame)];
        setVec3(flameColour, displayColour(flameLook.srgb) * flameLook.opacity);
        drawPass(5);
        glDepthMask(GL_TRUE);
        glBindTexture(GL_TEXTURE_BUFFER, 0);
        glActiveTexture(GL_TEXTURE0);
    }

    void drawTexture(GLuint texture, float weight) const {
        glUseProgram(copyProgram);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture);
        glUniform1i(copyTexture, 0);
        glUniform1f(copyWeight, weight);
        glBindVertexArray(emptyVao);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindTexture(GL_TEXTURE_2D, 0);
    }
};

EngineSceneRenderer::EngineSceneRenderer() : impl_(std::make_unique<Impl>()) {}

EngineSceneRenderer::~EngineSceneRenderer() = default;

bool EngineSceneRenderer::initialise(juce::String& error) {
    auto& s = *impl_;
    s.surface = linkProgram(meshVertexSource, surfaceFragmentSource, error);
    if (s.surface != 0) s.flame = linkProgram(meshVertexSource, flameFragmentSource, error);
    if (s.flame != 0) s.background = linkProgram(screenVertexSource, backgroundFragmentSource, error);
    if (s.background != 0) s.copyProgram = linkProgram(screenVertexSource, copyFragmentSource, error);
    if (s.copyProgram != 0) s.floorProgram = linkProgram(floorVertexSource, floorFragmentSource, error);
    if (s.floorProgram == 0) {
        release();
        return false;
    }
    const auto uniform = [](GLuint program, const char* name) { return glGetUniformLocation(program, name); };
    s.surfaceViewProjection = uniform(s.surface, "uViewProjection");
    s.surfaceEye = uniform(s.surface, "uEye");
    s.surfaceSkipDark = uniform(s.surface, "uSkipDark");
    s.lightCount = uniform(s.surface, "uLightCount");
    s.lightPosition = uniform(s.surface, "uLightPosition");
    s.lightColour = uniform(s.surface, "uLightColour");
    s.flameViewProjection = uniform(s.flame, "uViewProjection");
    s.flameEye = uniform(s.flame, "uEye");
    s.flameColour = uniform(s.flame, "uColour");
    s.flameSkipDark = uniform(s.flame, "uSkipDark");
    s.backgroundSize = uniform(s.background, "uSize");
    s.copyTexture = uniform(s.copyProgram, "uTexture");
    s.copyWeight = uniform(s.copyProgram, "uWeight");
    s.floorViewProjection = uniform(s.floorProgram, "uViewProjection");
    s.floorMode = uniform(s.floorProgram, "uMode");
    s.floorColour = uniform(s.floorProgram, "uColour");
    s.floorCentreUniform = uniform(s.floorProgram, "uCentre");
    s.floorExtent = uniform(s.floorProgram, "uExtent");
    s.floorRadiusUniform = uniform(s.floorProgram, "uRadius");
    // The part buffer texture always sits on unit 1.
    for (const auto program : { s.surface, s.flame }) {
        glUseProgram(program);
        glUniform1i(uniform(program, "uParts"), 1);
    }
    glUseProgram(0);
    glGenVertexArrays(1, &s.emptyVao);
    glGetIntegerv(GL_MAX_SAMPLES, &s.maximumSamples);
    if (model_) s.upload(*model_);
    return true;
}

void EngineSceneRenderer::release() {
    auto& s = *impl_;
    s.destroyTargets();
    s.destroyGeometry();
    for (auto* program : { &s.surface, &s.flame, &s.background, &s.copyProgram, &s.floorProgram }) {
        if (*program != 0) glDeleteProgram(*program);
        *program = 0;
    }
    if (s.emptyVao != 0) glDeleteVertexArrays(1, &s.emptyVao);
    s.emptyVao = 0;
}

void EngineSceneRenderer::setModel(std::shared_ptr<const render::EngineModel3D> model) {
    model_ = std::move(model);
    if (model_ && impl_->surface != 0) impl_->upload(*model_);
}

void EngineSceneRenderer::render(const SceneFrame& frame, int width, int height) {
    auto& s = *impl_;
    if (!model_ || s.surface == 0 || width <= 0 || height <= 0 || frame.angles.empty()) return;
    GLint entryFramebuffer = 0;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &entryFramebuffer);
    s.ensureTargets(width, height, frame.antiAliasing ? std::min(4, s.maximumSamples) : 0);

    auto pose = frame.pose;
    const auto subframes = frame.angles.size();
    for (std::size_t k = 0; k < subframes; ++k) {
        pose.crankAngleDegrees = frame.angles[k];
        model_->pose(pose, s.instances);
        glBindFramebuffer(GL_FRAMEBUFFER, s.msaaFbo);
        s.drawScene(*model_, frame, width, height);

        glBindFramebuffer(GL_READ_FRAMEBUFFER, s.msaaFbo);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, s.resolveFbo);
        glBlitFramebuffer(0, 0, width, height, 0, 0, width, height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        if (subframes == 1) break;

        // Average the sub-frames: each one adds its share.
        glBindFramebuffer(GL_FRAMEBUFFER, s.accumFbo);
        glViewport(0, 0, width, height);
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        if (k == 0) {
            glClearColor(0.0F, 0.0F, 0.0F, 0.0F);
            glClear(GL_COLOR_BUFFER_BIT);
        }
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);
        s.drawTexture(s.resolveTexture, 1.0F / static_cast<float>(subframes));
    }

    glBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(entryFramebuffer));
    glViewport(0, 0, width, height);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDepthMask(GL_FALSE);
    s.drawTexture(subframes == 1 ? s.resolveTexture : s.accumTexture, 1.0F);

    // Leave the state the way JUCE's component compositing expects it.
    glDepthMask(GL_TRUE);
    glUseProgram(0);
    glBindVertexArray(0);
    glActiveTexture(GL_TEXTURE0);
}

} // namespace enginelab::ui
