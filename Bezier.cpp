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
#include "Bezier.h"

FALCOR_EXPORT_D3D12_AGILITY_SDK

Bezier::Bezier(const SampleAppConfig& config) : SampleApp(config) {}

Bezier::~Bezier() {}

void Bezier::nk_precalc(int n)
{
    float fn = 1;

    for (int i = 0; i <= n; i++)
    {
        nChoosek[i] = 1;
    }

    float k1 = 1;
    float k2 = fn;

    for (int i = 1; i <= n / 2; i++)
    {
        nChoosek[i] = nChoosek[i - 1] * (n - i + 1) / i;
        nChoosek[n - i] = nChoosek[i];
    }
}

void Bezier::cheb_precalc(int n) {

    for (int i = 0; i <= n; i++)
    {
        chebisev[n-i] = 0.5+ 0.5*cos((2 * (float)i +1) / ((float)(n+1) * 2) * M_PI);
    }

}


struct Vertex
{
    float3 pos;
    float3 col;
};

void initSquare(ref<Device> pDevice, ref<Buffer>& pVB, ref<Vao>& pVao)
{
    std::vector<Vertex> vertices;
     // front
    // front
    vertices.push_back({float3(-0.5, -0.5, +0.5), float3(1, 0, 1)});
    vertices.push_back({float3(+0.5, -0.5, +0.5), float3(1, 0, 0)});
    vertices.push_back({float3(-0.5, +0.5, +0.5), float3(1, 1, 0)});
    vertices.push_back({float3(+0.5, +0.5, +0.5), float3(0, 1, 1)});
    // back
    vertices.push_back({float3(+0.5, -0.5, -0.5), float3(1, 1, 1)});
    vertices.push_back({float3(-0.5, -0.5, -0.5), float3(0, 0, 1)});
    vertices.push_back({float3(+0.5, +0.5, -0.5), float3(1, 0, 1)});
    vertices.push_back({float3(-0.5, +0.5, -0.5), float3(1, 1, 0)});
    // right
    vertices.push_back({float3(+0.5, -0.5, +0.5), float3(0, 1, 1)});
    vertices.push_back({float3(+0.5, -0.5, -0.5), float3(1, 1, 0)});
    vertices.push_back({float3(+0.5, +0.5, +0.5), float3(0, 1, 0)});
    vertices.push_back({float3(+0.5, +0.5, -0.5), float3(1, 1, 0)});
    // left
    vertices.push_back({float3(-0.5, -0.5, -0.5), float3(1, 0, 0)});
    vertices.push_back({float3(-0.5, -0.5, +0.5), float3(1, 0, 1)});
    vertices.push_back({float3(-0.5, +0.5, -0.5), float3(0, 1, 1)});
    vertices.push_back({float3(-0.5, +0.5, +0.5), float3(0, 0, 1)});
    // top
    vertices.push_back({float3(-0.5, +0.5, +0.5), float3(1, 1, 1)});
    vertices.push_back({float3(+0.5, +0.5, +0.5), float3(1, 0, 1)});
    vertices.push_back({float3(-0.5, +0.5, -0.5), float3(0, 1, 1)});
    vertices.push_back({float3(+0.5, +0.5, -0.5), float3(1, 1, 0)});
    // bottom
    vertices.push_back({float3(-0.5, -0.5, -0.5), float3(1, 0, 1)});
    vertices.push_back({float3(+0.5, -0.5, -0.5), float3(1, 0, 0)});
    vertices.push_back({float3(-0.5, -0.5, +0.5), float3(1, 0, 1)});
    vertices.push_back({float3(+0.5, -0.5, +0.5), float3(0, 1, 0)});

    
    std::vector<Vertex> indices(36);
    int index = 0;

    for (int i = 0; i < 6 * 4; i += 4)
    {
        indices[index + 0] = vertices[i + 0];
        indices[index + 1] = vertices[i + 1];
        indices[index + 2] = vertices[i + 2];
        indices[index + 3] = vertices[i + 1];
        indices[index + 4] = vertices[i + 3];
        indices[index + 5] = vertices[i + 2];
        index += 6;
    }

    sizeof(Vertex);
    // Create VB
    const uint32_t vbSize = (uint32_t)(sizeof(Vertex) * indices.size());
    pVB = pDevice->createBuffer(vbSize, ResourceBindFlags::Vertex, MemoryType::DeviceLocal, (void*)indices.data());
    assert(pVB);

    // Create VAO
    ref<VertexLayout> pLayout = VertexLayout::create();
    ref<VertexBufferLayout> pBufLayout = VertexBufferLayout::create();
    pBufLayout->addElement("POSITION", offsetof(Vertex, pos), ResourceFormat::RGB32Float, 1, 0);
    pBufLayout->addElement("COLOR", offsetof(Vertex, col), ResourceFormat::RGB32Float, 1, 1);
    pLayout->addBufferLayout(0, pBufLayout);

    Vao::BufferVec buffers{pVB};
    pVao = Vao::create(Vao::Topology::TriangleStrip, pLayout, buffers);
    assert(pVao);
}

void Bezier::onLoad(RenderContext* pRenderContext)
{
    initSquare(getDevice(), mpVbo, mpVao);

    ProgramDesc d;
    d.addShaderLibrary("Samples/Bezier/Shaders/bezier2d.vs.slang").vsEntry("main");
    d.addShaderLibrary("Samples/Bezier/Shaders/bezier2d.ps.slang").psEntry("main");
    program = Program::create(getDevice(), d);
    state = GraphicsState::create(getDevice());
    state->setProgram(program);
    state->setVao(mpVao);

    vars = ProgramVars::create(getDevice(), program.get());

    camera = Camera::create();
    camera->setPosition(float3(2, 2, 2));
    camera->setTarget(float3(0, 0, 0));
    camera->setUpVector(float3(0, 1, 0));
    camera->setAspectRatio(1280.0f / 720.0f);
    camera->setNearPlane(0.01f);
    camera->setFarPlane(1000.0f);
    camera->beginFrame();


    //calculating matrix for curve fitting
    nk_precalc(bezierdegree);
    cheb_precalc(bezierdegree);
   
    base = bernsteinmatrix();


    std::vector<float> nk_temp;
    for (int i = 0; i <= bezierdegree; i++)
    {
        nk_temp.push_back(nChoosek[i]);
    }
    nk = getDevice()->createBuffer(
        sizeof(float) * (bezierdegree + 1),
        ResourceBindFlags::UnorderedAccess,
        MemoryType::DeviceLocal,
       nk_temp.data()
    );
    
}


std::vector<float> Bezier::bernsteinmatrix() {

    Eigen::MatrixXf B(bezierdegree + 1, bezierdegree + 1);

    for (int i = 0; i <= bezierdegree; i++)
    {

        float t = chebisev[i];
        for (int j = 0; j <= bezierdegree; j++)
        {
            B(i, j) = pow(t, j) * pow(1 - t, bezierdegree - j) * nChoosek[j];
        }
    }

    std::cout << B << std::endl;

    Eigen::MatrixXf pseudoinverse = (B.transpose() * B).inverse() * B.transpose();

    std::vector<float> linearisedpi;

    for (int i = 0; i < (bezierdegree + 1); i++)
    {
        for (int j = 0; j < (bezierdegree+1) ; j++)
        {
            linearisedpi.push_back(pseudoinverse(i, j));
        }
    }

    b = getDevice()->createBuffer(
        sizeof(float)*(bezierdegree + 1) * (bezierdegree + 1), ResourceBindFlags::UnorderedAccess, MemoryType::DeviceLocal, linearisedpi.data()
    );

    return linearisedpi;

}

void Bezier::onResize(uint32_t width, uint32_t height)
{
    mAspectRatio = (float(width) / float(height));
}

void Bezier::onFrameRender(RenderContext* pRenderContext, const ref<Fbo>& pTargetFbo)
{
     const float4 clearColor(1.f, 1.f, 1.f, 1);
    pRenderContext->clearFbo(pTargetFbo.get(), clearColor, 1.0f, 0, FboAttachmentType::All);

     state->setFbo(pTargetFbo);
     ShaderVar pVars = vars->getRootVar();

     float4x4 m = {2, 0, 0, 0, 0, 2, 0, 0, 0, 0, 2, 0, 0, 0, 0, 2};

    pVars["vsCb"]["model"] = m;
    pVars["vsCb"]["viewproj"] = camera->getViewProjMatrix();
    pVars["psCb"]["eye"] = camera->getPosition();
    //pVars["psCb"]["bezierdegree"] = bezierdegree;
    program->addDefine("N",std::to_string(bezierdegree));
    program->addDefine("BEZ", std::to_string(bez_alg));
    program->addDefine("NK", std::to_string(nk_input));
    pVars["x0"] = b;
    if (nk_input == 0)
    {
        pVars["nChoosek"] = nk;
    }
    else
    {
        pVars["nChoosek"] = nk;
    }
   
    pRenderContext->draw(state.get(), vars.get(), 36, 0);

}

int runMain(int argc, char** argv)
{
    SampleAppConfig config;
    config.windowDesc.width = 1280;
    config.windowDesc.height = 720;
    config.windowDesc.resizableWindow = true;
    config.windowDesc.enableVSync = true;
    config.windowDesc.title = "Falcor Shader Toy";

    Bezier Bezier(config);
    return Bezier.run();
}

int main(int argc, char** argv)
{
    return catchAndReportAllExceptions([&]() { return runMain(argc, argv); });
}
