/***************************************************************************
 # Copyright (c) 2015-23, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/
#pragma once
#include "Falcor.h"
#include "Core/SampleApp.h"
#include "Core/Pass/FullScreenPass.h"
#include "Eigen/Dense"

using namespace Falcor;

class Bezier : public SampleApp
{
public:
    Bezier(const SampleAppConfig& config);
    ~Bezier();

    void onLoad(RenderContext* pRenderContext) override;
    void onResize(uint32_t width, uint32_t height) override;
    void onFrameRender(RenderContext* pRenderContext, const ref<Fbo>& pTargetFbo) override;

    void nk_precalc(int n);
    void cheb_precalc(int n);
    std::vector<float> bernsteinmatrix();

private:
    float mAspectRatio = 0;

    ref<Program> program;
    ref<Vao> mpVao;
    ref<Buffer> mpVbo;
    ref<GraphicsState> state;
    ref<ProgramVars> vars;

    ref<Camera> camera;
    ref<CameraController> cameracontrol;

    int nChoosek[1000];
    float chebisev[1000];

    int bezierdegree = 5;
    std::vector<float> base;
    ref<Buffer> b;
    ref<Buffer> nk;

    int bez_alg = 1;
    //0 - de casteljau, 1- chudy, 2- mienk, 3- mienk + nk, 4- estrin, 5- estrin + nk
    int nk_input = 1; //0 - structured buffer, 1 - const buffer, 2 - matrixban
    
};
