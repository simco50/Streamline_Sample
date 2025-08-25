//----------------------------------------------------------------------------------
// File:        StreamlineSample.hlsl
// SDK Version: 2.0
// Email:       StreamlineSupport@nvidia.com
// Site:        http://developer.nvidia.com/
//
// Copyright (c) 2024-2025, NVIDIA CORPORATION. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//----------------------------------------------------------------------------------

#pragma pack_matrix(row_major)

#if REFLECT_MATERIALS
#define MATERIAL_CB_SLOT        0
#define MATERIAL_DIFFUSE_SLOT   3
#define MATERIAL_SPECULAR_SLOT  4
#define MATERIAL_NORMALS_SLOT   5
#define MATERIAL_EMISSIVE_SLOT  6
#define MATERIAL_OCCLUSION_SLOT 7
#define MATERIAL_TRANSMISSION_SLOT 8
#define MATERIAL_SAMPLER_SLOT   0
#define MATERIAL_REGISTER_SPACE 1
#endif

#include <donut/shaders/gbuffer.hlsli>
#include <donut/shaders/scene_material.hlsli>
#include <donut/shaders/material_bindings.hlsli>
#include <donut/shaders/lighting.hlsli>
#include "lighting_cb.h"

// ---[ Structures ]---

struct ShadowHitInfo
{
    bool missed;
};

struct ReflectionHitInfo
{
    float3 color;
    float2 randSample;
    float hitDist;
};

struct Attributes 
{
    float2 uv;
};

// ---[ Resources ]---

ConstantBuffer<LightingConstants> g_Lighting : register(b0);

RWTexture2D<float4> u_Output : register(u0);

RaytracingAccelerationStructure SceneBVH : register(t0);
Texture2D t_GBufferDepth : register(t1);
Texture2D t_GBuffer0 : register(t2);
Texture2D t_GBuffer1 : register(t3);
Texture2D t_GBuffer2 : register(t4);
Texture2D t_GBuffer3 : register(t5);

// ---[ Utilities ]---
inline float Halton(int index, int base)
{
    float result = 0.0f;
    float invBase = 1.0f / base;
    float fraction = invBase;
    while (index > 0)
    {
        result += (index % base) * fraction;
        index /= base;
        fraction *= invBase;
    }
    return result;
}

uint MortonEncode2(uint2 pixel)
{
    uint morton = pixel.x | pixel.y << 16;
    morton = (morton | (morton << 4)) & 0x0f0f0f0f;
    morton = (morton | (morton << 2)) & 0x33333333;
    morton = (morton | (morton << 1)) & 0x55555555;
    return (morton | morton >> 15) & 0xffff;
}

float3 SampleCone(float2 randSample, float cosThetaMax)
{
    float PI = 3.14159;
    float phi = 2 * PI * randSample.x;
    float cosTheta = lerp(cosThetaMax, 1, randSample.y);
    float sinTheta = sqrt(1 - cosTheta * cosTheta);

    return float3(sinTheta * cos(phi), sinTheta * sin(phi), cosTheta);
}

float3 TangentToWorld(float3 vec, float3 tangentZ)
{
    const float sign = tangentZ.z >= 0 ? 1 : -1;
    const float a = -rcp(sign + tangentZ.z);
    const float b = tangentZ.x * tangentZ.y * a;
    
    float3 tangentX = { 1 + sign * a * pow(2, tangentZ.x), sign * b, -sign * tangentZ.x };
    float3 tangentY = { b,  sign + a * pow(2, tangentZ.y), -tangentZ.y };

    return mul(vec, float3x3(tangentX, tangentY, tangentZ));
}

float3 EnvBRDFApprox2(float3 SpecularColor, float alpha, float NoV)
{
    NoV = abs(NoV);
    // [Ray Tracing Gems, Chapter 32]
    float4 X;
    X.x = 1.f;
    X.y = NoV;
    X.z = NoV * NoV;
    X.w = NoV * X.z;

    float4 Y;
    Y.x = 1.f;
    Y.y = alpha;
    Y.z = alpha * alpha;
    Y.w = alpha * Y.z;

    float2x2 M1 = float2x2(0.99044f, -1.28514f, 1.29678f, -0.755907f);
    float3x3 M2 = float3x3(1.f, 2.92338f, 59.4188f, 20.3225f, -27.0302f, 222.592f, 121.563f, 626.13f, 316.627f);

    float2x2 M3 = float2x2(0.0365463f, 3.32707, 9.0632f, -9.04756);
    float3x3 M4 = float3x3(1.f, 3.59685f, -1.36772f, 9.04401f, -16.3174f, 9.22949f, 5.56589f, 19.7886f, -20.2123f);

    float bias = dot(mul(M1, X.xy), Y.xy) * rcp(dot(mul(M2, X.xyw), Y.xyw));
    float scale = dot(mul(M3, X.xy), Y.xy) * rcp(dot(mul(M4, X.xzw), Y.xyw));

    // This is a hack for specular reflectance of 0
    bias *= saturate(SpecularColor.g * 50);

    return mad(SpecularColor, max(0, scale), max(0, bias));
}

// ---[ Ray Generation Shader ]---

float GetShadow(float3 worldPos, float3 lightDirection, float angularSize, float2 randSample)
{
    // Sample uniformly about a cone aligned with the z-axis
    float sinTheta = sin(angularSize * 0.5f);
    float3 dir = SampleCone(randSample, sqrt(1.0f - sinTheta * sinTheta));

    // Setup the ray
    RayDesc ray;
    ray.Origin = worldPos;
    ray.Direction = -normalize(TangentToWorld(dir, lightDirection));
    ray.TMin = 0.01f;
    ray.TMax = 100.f;

    // Trace the ray
    ShadowHitInfo shadowPayload;
    shadowPayload.missed = false;

    TraceRay(
        SceneBVH,
        RAY_FLAG_CULL_BACK_FACING_TRIANGLES,
        0xFF, // InstanceInclusionMask
        0, // RayContributionToHitGroupIndex 
        2, // MultiplierForGeometryContributionToHitGroupIndex
        0, // MissShaderIndex
        ray,
        shadowPayload);

    return (shadowPayload.missed) ? 1 : 0;
}

float4 GetReflection(float3 worldPos, float3 reflectedVector, float roughness, float2 randSample)
{
    // Sample uniformly about a cone aligned with the z-axis
    float3 dir = ImportanceSampleGGX(randSample, saturate(roughness));

    // Setup the ray
    RayDesc ray;
    ray.Origin = worldPos;
    ray.Direction = normalize(TangentToWorld(dir, reflectedVector));
    ray.TMin = 0.01f;
    ray.TMax = 100.f;

    // Trace the ray
    ReflectionHitInfo reflectionPayload;
    reflectionPayload.color = 0;
    reflectionPayload.randSample = randSample;
    reflectionPayload.hitDist = ray.TMax;

    TraceRay(
        SceneBVH,
        RAY_FLAG_CULL_BACK_FACING_TRIANGLES,
        0xFF, // InstanceInclusionMask
        1, // RayContributionToHitGroupIndex 
        2, // MultiplierForGeometryContributionToHitGroupIndex
        1, // MissShaderIndex
        ray,
        reflectionPayload);

    return float4(reflectionPayload.color, reflectionPayload.hitDist);
}

[shader("raygeneration")]
void RayGen()
{
    uint2 globalIdx = DispatchRaysIndex().xy;
    float2 pixelPosition = float2(globalIdx) + 0.5;

    MaterialSample surfaceMaterial = DecodeGBuffer(globalIdx, t_GBuffer0, t_GBuffer1, t_GBuffer2, t_GBuffer3);

    float3 surfaceWorldPos = ReconstructWorldPosition(g_Lighting.view, pixelPosition.xy, t_GBufferDepth[pixelPosition.xy].x);

    float3 viewIncident = GetIncidentVector(g_Lighting.view.cameraDirectionOrPosition, surfaceWorldPos);

    float3 diffuseTerm = 0;
    float3 specularTerm = 0;
    float hitDist = 100.0f;

    float3 specAlbedo = 0;

    if (any(surfaceMaterial.shadingNormal != 0))
    {
        int seed = MortonEncode2(globalIdx) + g_Lighting.frameIndex * 127;
        // Use a 5,7 Halton sequence for sampling, since 2,3 is already being used for jitter
        // and using the same pattern leads to striations or checkerboard patterns
        float2 randSample = float2(Halton(seed, 5), Halton(seed, 7));

        float shadow = GetShadow(surfaceWorldPos, g_Lighting.light.direction, g_Lighting.light.angularSizeOrInvRange, randSample);

        if (shadow > 0)
        {
            float3 diffuseRadiance, specularRadiance;
            ShadeSurface(g_Lighting.light, surfaceMaterial, surfaceWorldPos, viewIncident, diffuseRadiance, specularRadiance);

            diffuseTerm += (shadow * diffuseRadiance) * g_Lighting.light.color;
            specularTerm += (shadow * specularRadiance) * g_Lighting.light.color;
        }

        diffuseTerm += g_Lighting.ambientColor.rgb * surfaceMaterial.diffuseAlbedo;
        
        float4 reflection = GetReflection(surfaceWorldPos, reflect(viewIncident, surfaceMaterial.shadingNormal), surfaceMaterial.roughness, randSample);
        specAlbedo = EnvBRDFApprox2(surfaceMaterial.specularF0, square(surfaceMaterial.roughness), saturate(-dot(viewIncident, surfaceMaterial.shadingNormal)));
        specularTerm += reflection.xyz * specAlbedo;

        hitDist = reflection.w;
    }

    float3 outputColor = diffuseTerm
        + specularTerm
        + surfaceMaterial.emissiveColor;

    u_Output[globalIdx] = float4(outputColor, 1);
}

// ---[ Shadow Miss Shader ]---

[shader("miss")]
void ShadowMiss(inout ShadowHitInfo shadowPayload : SV_RayPayload)
{
    shadowPayload.missed = true;
}

// ---[ Reflection Shaders ]---

#if REFLECT_MATERIALS
Buffer<uint> t_MeshIndexBuffer : register(t0, space1);
Buffer<float2> t_MeshTexCoordBuffer : register(t1, space1);
Buffer<float4> t_MeshNormalsBuffer : register(t2, space1);
#endif

[shader("miss")]
void ReflectionMiss(inout ReflectionHitInfo reflectionPayload : SV_RayPayload)
{
}

[shader("closesthit")]
void ReflectionClosestHit(inout ReflectionHitInfo reflectionPayload : SV_RayPayload, in Attributes attrib : SV_IntersectionAttributes)
{
#if REFLECT_MATERIALS
    uint triangleIndex = PrimitiveIndex();
    float3 barycentrics = float3((1.0f - attrib.uv.x - attrib.uv.y), attrib.uv.x, attrib.uv.y);

    uint3 indices;
    indices.x = t_MeshIndexBuffer[triangleIndex * 3 + 0];
    indices.y = t_MeshIndexBuffer[triangleIndex * 3 + 1];
    indices.z = t_MeshIndexBuffer[triangleIndex * 3 + 2];

    float2 vertexUVs[3];
    vertexUVs[0] = t_MeshTexCoordBuffer[indices.x];
    vertexUVs[1] = t_MeshTexCoordBuffer[indices.y];
    vertexUVs[2] = t_MeshTexCoordBuffer[indices.z];

    float3 vertexNormals[3];
    vertexNormals[0] = t_MeshNormalsBuffer[indices.x].xyz;
    vertexNormals[1] = t_MeshNormalsBuffer[indices.y].xyz;
    vertexNormals[2] = t_MeshNormalsBuffer[indices.z].xyz;

    float2 uv =
        vertexUVs[0] * barycentrics.x +
        vertexUVs[1] * barycentrics.y +
        vertexUVs[2] * barycentrics.z;

    float3 normal = normalize(
        vertexNormals[0] * barycentrics.x +
        vertexNormals[1] * barycentrics.y +
        vertexNormals[2] * barycentrics.z);
    
    MaterialTextureSample textures = SampleMaterialTexturesLevel(uv, 3);

    MaterialSample surfaceMaterial = EvaluateSceneMaterial(normal, /* tangent = */ 0, g_Material, textures);

    float3 surfaceWorldPos = WorldRayOrigin() + WorldRayDirection() * RayTCurrent();

    float3 diffuseRadiance, specularRadiance;
    float3 viewIncident = WorldRayDirection();
    ShadeSurface(g_Lighting.light, surfaceMaterial, surfaceWorldPos, viewIncident, diffuseRadiance, specularRadiance);
#else
    float3 surfaceWorldPos = WorldRayOrigin() + WorldRayDirection() * RayTCurrent();

    float3 diffuseRadiance = 0.05f;
    float3 specularRadiance = 0;
#endif

    float3 diffuseTerm = 0;
    float3 specularTerm = 0;

    float shadow = GetShadow(surfaceWorldPos, g_Lighting.light.direction, g_Lighting.light.angularSizeOrInvRange, reflectionPayload.randSample);
    diffuseTerm += (shadow * diffuseRadiance) * g_Lighting.light.color;
    specularTerm += (shadow * specularRadiance) * g_Lighting.light.color;

#if REFLECT_MATERIALS
    diffuseTerm += g_Lighting.ambientColor.rgb * surfaceMaterial.diffuseAlbedo;
#endif

    reflectionPayload.color = diffuseTerm + specularTerm;
    reflectionPayload.hitDist = RayTCurrent();
}