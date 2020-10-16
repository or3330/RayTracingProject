/*
 * Adapted from Open Shading Language with this license:
 *
 * Copyright (c) 2009-2010 Sony Pictures Imageworks Inc., et al.
 * All Rights Reserved.
 *
 * Modifications Copyright 2012, Blender Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the name of Sony Pictures Imageworks nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __BSDF_COOK_TORRANCE_H__
#define __BSDF_COOK_TORRANCE_H__

CCL_NAMESPACE_BEGIN

#ifdef __OSL__

typedef ccl_addr_space struct CookTorranceBsdf {
  SHADER_CLOSURE_BASE;
  float roughness;
  float ior;
  float metallic;
  float reflectance;
  float3 base_color;
} CookTorranceBsdf;


//ccl_device_forceinline float3 reflection_color(const CookTorranceBsdf *bsdf, float3 L, float3 H)
//{
//  float3 F = make_float3(1.0f, 1.0f, 1.0f);
//  //cspec0 need to understand this variable, now its a defualt value like at  intern/cycles/kernel/osl/osl_closures.cpp:304
//  float3 cspec0 = make_float3(0.04f, 0.04f, 0.04f);
//  float F0 = fresnel_dielectric_cos(1.0f, bsdf->ior);
//  F = interpolate_fresnel_color(L, H, bsdf->ior, F0, cspec0);
//  return F;
//}

static_assert(sizeof(ShaderClosure) >= sizeof(CookTorranceBsdf), "CookTorranceBsdf is too large!");

ccl_device int bsdf_cook_torrance_setup(CookTorranceBsdf *bsdf)
{
  bsdf->type = CLOSURE_BSDF_COOK_TORRANCE_ID;
  bsdf->roughness = max(bsdf->roughness, 0.0f);
  bsdf->ior = max(bsdf->ior, 0.0f);
  bsdf->metallic = max(bsdf->metallic, 0.0f);
  bsdf->reflectance = max(bsdf->reflectance, 0.0f);
  printf("reflectance %f\n", double(bsdf->reflectance));
  //maybe change return value
  return SD_BSDF | SD_BSDF_HAS_EVAL;
}

ccl_device float3 bsdf_cook_torrance_eval_reflect(const ShaderClosure *sc,
                                               const float3 I,
                                               const float3 omega_in,
                                               float *pdf)
{
  // i use this method for the cook torrance https://google.github.io/filament/Filament.md.html

  const CookTorranceBsdf *bsdf = (const CookTorranceBsdf *)sc;
  //get the param from the sc
  float roughness = bsdf->roughness;
  
  //CHECK IF NEED REPLACE EYE VECTOR AND LIGHT VECTOR;
  float3 viewVector = I;
  float3 lightVector = omega_in;

  // intermediary values
  float3 halfvector = normalize(omega_in + I);
  float alpha2 = roughness*roughness;
  float NdotH = max(dot(bsdf->N,halfvector),0.0f); 
  float NdotV = max(dot(bsdf->N,viewVector),0.0f);
  float NdotL = max(dot(bsdf->N,lightVector),0.0f);
  float VdotH = max(dot(viewVector, halfvector), 0.0f);
  
  // first we implemet the D function, we use ggx method
  
  	
  float  ggx_1_temp= roughness / (1.0f - (NdotH * NdotH) + alpha2);
  float  D =  ggx_1_temp * ggx_1_temp * M_1_PI_F ; // M_1_PI_F  = 1/Pi 

  // now we calculate the G function
  float g1 = (2.0f * NdotV) / (NdotV + safe_sqrtf(alpha2 + (1.0f - alpha2) * NdotV * NdotV));
  float g2 = (2.0f * NdotL) / (NdotL + sqrt(alpha2 + (1.0f - alpha2) * NdotL * NdotL));
  float G = g1*g2;


  //now we calculate fresnel factor;
  float m = bsdf->metallic;
  float3 base_color = bsdf->base_color;
  float3 f90 = (1.0f - m) * bsdf->base_color;
  float reflect_part = 0.16f * bsdf->reflectance * bsdf->reflectance * (1.0f - m);
  float3 f0 = make_float3(reflect_part, reflect_part, reflect_part) + base_color * m;
  float3 F = f0 + (f90 - f0) * powf(1.0f - VdotH, 5);
  
  if (NdotV > 0 && NdotL > 0) {
    // reflect the view vector
      float common = D * 0.25f / NdotV;
      float3 out = F*G*common;

      //TODO: itay explain what is this!! https://github.com/rorydriscoll/RayTracer/blob/master/Source/RayTracer/Brdf/MicrofacetBrdf.cpp
      *pdf = g1 * common;

      return out;
    }
  return make_float3(0.0f, 0.0f, 0.0f);
}

ccl_device float3 bsdf_cook_torrance_eval_transmit(const ShaderClosure *sc,
                                                const float3 I,
                                                const float3 omega_in,
                                                float *pdf)
{
  return make_float3(0.0f, 0.0f, 0.0f);
}

ccl_device int bsdf_cook_torrance_sample(const ShaderClosure *sc,
                                      float3 Ng,
                                      float3 I,
                                      float3 dIdx,
                                      float3 dIdy,
                                      float randu,
                                      float randv,
                                      float3 *eval,
                                      float3 *omega_in,
                                      float3 *domega_in_dx,
                                      float3 *domega_in_dy,
                                      float *pdf)
{
  const CookTorranceBsdf *bsdf = (const CookTorranceBsdf *)sc;
  float cosNO = dot(bsdf->N, I);
  float m_exponent = 2.0f;

  if (cosNO > 0) {
    // reflect the view vector
    float3 R = (2 * cosNO) * bsdf->N - I;

#  ifdef __RAY_DIFFERENTIALS__
    *domega_in_dx = (2 * dot(bsdf->N, dIdx)) * bsdf->N - dIdx;
    *domega_in_dy = (2 * dot(bsdf->N, dIdy)) * bsdf->N - dIdy;
#  endif

    float3 T, B;
    make_orthonormals(R, &T, &B);
    float phi = M_2PI_F * randu;
    float cosTheta = powf(randv, 1 / (m_exponent + 1));
    float sinTheta2 = 1 - cosTheta * cosTheta;
    float sinTheta = sinTheta2 > 0 ? sqrtf(sinTheta2) : 0;
    *omega_in = (cosf(phi) * sinTheta) * T + (sinf(phi) * sinTheta) * B + (cosTheta)*R;
    if (dot(Ng, *omega_in) > 0.0f) {
      // common terms for pdf and eval
      float cosNI = dot(bsdf->N, *omega_in);
      // make sure the direction we chose is still in the right hemisphere
      if (cosNI > 0) {
        float cosp = powf(cosTheta, m_exponent);
        float common = 0.5f * M_1_PI_F * cosp;
        *pdf = (m_exponent + 1) * common;
        float out = cosNI * (m_exponent + 2) * common;
        *eval = make_float3(1.0f, 1.0f, 1.0f)* out;
      }
    }
  }
  return LABEL_REFLECT | LABEL_GLOSSY;
}

#endif /* __OSL__ */

CCL_NAMESPACE_END

#endif /* __BSDF_COOK_TORRANCE_H__ */
