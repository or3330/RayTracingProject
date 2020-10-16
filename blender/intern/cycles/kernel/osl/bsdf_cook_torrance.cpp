/*
 * Adapted from Open Shading Language with this license:
 *
 * Copyright (c) 2009-2010 Sony Pictures Imageworks Inc., et al.
 * All Rights Reserved.
 *
 * Modifications Copyright 2011, Blender Foundation.
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

#include <OpenImageIO/fmath.h>

#include <OSL/genclosure.h>

#include "kernel/kernel_compat_cpu.h"
#include "kernel/osl/osl_closures.h"

// clang-format off
#include "kernel/kernel_types.h"
#include "kernel/closure/alloc.h"
#include "kernel/closure/bsdf_util.h"
#include "kernel/closure/bsdf_cook_torrance.h"
// clang-format on

CCL_NAMESPACE_BEGIN

using namespace OSL;

class CookTorranceClosure : public CBSDFClosure {
 public:
  CookTorranceBsdf params;

  void setup(ShaderData *sd, int /* path_flag */, float3 weight)
  {
    CookTorranceBsdf *bsdf = (CookTorranceBsdf *)bsdf_alloc_osl(
        sd, sizeof(CookTorranceBsdf), weight, &params);
  
        sd->flag |= bsdf_cook_torrance_setup(bsdf);
      
  }
};

ClosureParam *closure_bsdf_cook_torrance_params()
{
  static ClosureParam params[] = {CLOSURE_FLOAT3_PARAM(CookTorranceClosure, params.N),
                                  CLOSURE_FLOAT_PARAM(CookTorranceClosure, params.roughness),
                                  CLOSURE_FLOAT_PARAM(CookTorranceClosure, params.metallic),
                                  CLOSURE_FLOAT_PARAM(CookTorranceClosure, params.ior),
                                  CLOSURE_FLOAT3_PARAM(CookTorranceClosure, params.fresnel_color),
                                  CLOSURE_STRING_KEYPARAM(CookTorranceClosure, label, "label"),
                                  CLOSURE_FINISH_PARAM(CookTorranceClosure)};
  return params;
}

CCLOSURE_PREPARE(closure_bsdf_cook_torrance_prepare, CookTorranceClosure)

CCL_NAMESPACE_END
