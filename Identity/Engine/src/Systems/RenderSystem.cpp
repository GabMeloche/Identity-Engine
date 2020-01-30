#include <stdafx.h>
#include <Systems/RenderSystem.h>
#include <Managers/ModelManager.h>
#include "Tools/DirectX/GraphicsMacros.h"
#include "Rendering/Light.h"
#include "Tools/ImGUI/imgui.h"
#include "Tools/ImGUI/imgui_impl_dx11.h"
#include <Input/Input.h>

Engine::Systems::RenderSystem::RenderSystem(Rendering::Renderer* p_renderer): m_renderer(p_renderer)
{
}

void Engine::Systems::RenderSystem::DrawScene()
{
    static float tmp = 0.0f;
    //tmp += 0.05f;

    if (!m_renderer)
    {
        std::string error("in Engine::Systems::RenderSystem::DrawScene(): cannot draw scene because Renderer* m_renderer is nullptr");
        MessageBox(nullptr, error.c_str(), "Error", MB_ICONWARNING | MB_OK);
        return;
    }

    HRESULT hr;
    float offset = 0;

    for (auto& model : m_models)
    {
        for (auto& mesh : model.second->GetMeshes())
        {
            mesh->Bind(m_renderer->m_pContext);

            // ********* WIP ********* //

            // create constant buffer for transform matrix
            struct VertexConstantBuffer
            {
                Matrix4F model;
                Matrix4F view;
                Matrix4F normalModel;
                Matrix4F projection;
            };

            Vector3D quat{ 0, 1, 0 };
            Matrix4F model = Matrix4F::CreateTransformation(Vector3F(tmp, 0.0f, 0.0f),
                Quaternion::CreateFromAxisAngle(quat, GPM::Tools::Utils::ToRadians(180.0f)),
                Vector3F{ 0.02f, 0.02f, 0.02f });

            Matrix4F normalModel = Matrix4F::Inverse(model);

            m_camera.UpdateVectors();

            if (_INPUT->keyboard.IsKeyHeld(Input::Keyboard::W))
                m_camera.m_position -= m_camera.m_forward * m_camera.m_speed;
            if (_INPUT->keyboard.IsKeyHeld(Input::Keyboard::S))
                m_camera.m_position += m_camera.m_forward * m_camera.m_speed;
            if (_INPUT->keyboard.IsKeyHeld(Input::Keyboard::A))
                m_camera.m_position -= Vector3F::Cross(m_camera.m_forward, m_camera.m_up).Normalized() * m_camera.m_speed;
            if (_INPUT->keyboard.IsKeyHeld(Input::Keyboard::D))
                m_camera.m_position += Vector3F::Cross(m_camera.m_forward, m_camera.m_up).Normalized() * m_camera.m_speed;

            Matrix4F view = m_camera.GetViewMatrix();
            Matrix4F perspective = m_camera.GetPerspectiveMatrix();


            model.Transpose();
            view.Transpose();
            normalModel.Transpose();
            perspective.Transpose();

            const VertexConstantBuffer vcb{ model, view, normalModel,perspective };

            Microsoft::WRL::ComPtr<ID3D11Buffer> vertexConstantBuffer;
            D3D11_BUFFER_DESC                    vertexBufferDesc = {};
            vertexBufferDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
            vertexBufferDesc.Usage = D3D11_USAGE_DYNAMIC;        //Dynamic - values can change
            vertexBufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
            vertexBufferDesc.MiscFlags = 0u;
            vertexBufferDesc.ByteWidth = sizeof(vcb);
            vertexBufferDesc.StructureByteStride = 0u;
            D3D11_SUBRESOURCE_DATA VertexConstantShaderData = {};
            VertexConstantShaderData.pSysMem = &vcb;
            GFX_THROW_INFO(m_renderer->m_pDevice->CreateBuffer(&vertexBufferDesc, &VertexConstantShaderData, &vertexConstantBuffer));

            //bind the buffer to the shader
            m_renderer->m_pContext->VSSetConstantBuffers(0u, 1u, vertexConstantBuffer.GetAddressOf());

            struct PixelConstantBuffer
            {
                Rendering::Light lightSource;
                float lightShininess;
                Vector3F cameraPos;
                float padding;
            };

            Rendering::Light dirLight{};
            dirLight.position = Vector3F(-cos(0.0f) * 40.0f, 40.0f, -40.0f);
            dirLight.ambient = Vector3F(0.1f, 0.1f, 0.1f);
            dirLight.diffuse = Vector3F(1.0f, 1.0f, 0.95f);
            dirLight.specular = Vector3F(1.0f, 1.0f, 0.95f);
            dirLight.direction = Vector3F(-0.5f, -0.5f, -0.5f).Normalized();

            if (ImGui::Begin("Lighting Tool"))
            {
                ImGui::SliderFloat("LightPosX", &dirLight.position.x, -40.0f, 40.0f, "%.1f");
                ImGui::SliderFloat("LightPosY", &dirLight.position.y, -40.0f, 40.0f, "%.1f");
                ImGui::SliderFloat("LightPosZ", &dirLight.position.z, -40.0f, 40.0f, "%.1f");
            }ImGui::End();

            const PixelConstantBuffer pcb{ dirLight.position, dirLight.ambient, dirLight.diffuse,
                                          dirLight.specular, dirLight.direction, 32.0f, m_camera.GetPosition(), 0.0f };

            Microsoft::WRL::ComPtr<ID3D11Buffer> pixelConstantBuffer;
            D3D11_BUFFER_DESC                    pixelBufferDesc = {};
            pixelBufferDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
            pixelBufferDesc.Usage = D3D11_USAGE_DYNAMIC;        //Dynamic - values can change
            pixelBufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
            pixelBufferDesc.MiscFlags = 0u;
            pixelBufferDesc.ByteWidth = sizeof(pcb);
            pixelBufferDesc.StructureByteStride = 0u;

            D3D11_SUBRESOURCE_DATA PixelConstantShaderData = {};
            PixelConstantShaderData.pSysMem = &pcb;
            GFX_THROW_INFO(m_renderer->m_pDevice->CreateBuffer(&pixelBufferDesc, &PixelConstantShaderData, &pixelConstantBuffer));

            //bind the buffer to the shader
            m_renderer->m_pContext->PSSetConstantBuffers(0u, 1u, pixelConstantBuffer.GetAddressOf());

            m_renderer->LoadPixelShader(L"../Engine/Resources/Shaders/PixelShader.cso");
            m_renderer->LoadVertexShader(L"../Engine/Resources/Shaders/VertexShader.cso");

            m_renderer->m_pContext->OMSetRenderTargets(1u, m_renderer->m_pTarget.GetAddressOf(), m_renderer->m_pDepthStencil.Get());

            //set primitive draw
            m_renderer->m_pContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

            //create input layout
            Microsoft::WRL::ComPtr<ID3D11InputLayout> inputLayout;
            const D3D11_INPUT_ELEMENT_DESC            inputDesc[] =
            {
                {"Position", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
                {"TxCoord", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 12u, D3D11_INPUT_PER_VERTEX_DATA, 0},
                {"Normal", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 20u, D3D11_INPUT_PER_VERTEX_DATA, 0}
            };
            GFX_THROW_INFO(m_renderer->m_pDevice->CreateInputLayout(inputDesc,
                std::size(inputDesc),
                m_renderer->m_blob->GetBufferPointer(),
                m_renderer->m_blob->GetBufferSize(),
                &inputLayout));

            m_renderer->m_pContext->IASetInputLayout(inputLayout.Get());

            ImGui_ImplDX11_Init(m_renderer->m_pDevice.Get(), m_renderer->m_pContext.Get());

            GFX_THROW_INFO_ONLY(m_renderer->m_pContext->DrawIndexed(static_cast<UINT>(mesh->m_indices.size()), 0u, 0u));
        }
        offset += 3.0f;
    }
}

void Engine::Systems::RenderSystem::Update()
{
    DrawScene();
}

uint32_t Engine::Systems::RenderSystem::AddModel(const std::string& p_path, const std::string& p_name)
{
    if (Manager::ModelManager::GetInstance()->FindModel(p_name))
    {
        std::string error("in Engine::Systems::RenderSystem::AddModel(const std::string& p_path, const std::string& p_name): Could not add model with name " +
            p_name + " because it already exists");
        MessageBox(nullptr, error.c_str(), "Error", MB_ICONWARNING | MB_OK);
        return -1;
    }

    static uint32_t id = 0;

    std::shared_ptr newModel = Manager::ModelManager::GetInstance()->AddModel(p_path, p_name);

    if (newModel)
    {
        m_models.insert_or_assign(id, newModel);
        uint32_t tmpId = id;
        ++id;
        return tmpId;
    }
    else
        return -1;
}
