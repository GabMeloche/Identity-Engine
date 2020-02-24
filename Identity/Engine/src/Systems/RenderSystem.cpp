#include <stdafx.h>
#include <Systems/RenderSystem.h>
#include <Containers/ModelContainer.h>
#include "Tools/DirectX/GraphicsMacros.h"
#include "Rendering/Lights/Light.h"
#include "Tools/ImGUI/imgui.h"
#include "Tools/ImGUI/imgui_impl_dx11.h"
#include <Input/Input.h>
#include <Containers/GameObjectContainer.h>
#include <Containers/TransformContainer.h>
#include "Components/ModelComponent.h"
#include "Containers/CameraContainer.h"
#include <Containers/EventContainer.h>
#include <Rendering/Buffers/VertexConstantBuffer.h>
#include <Containers/LightContainer.h>

//Example of how to use events
Engine::Systems::RenderSystem::RenderSystem()
{
    /*Containers::EventContainer::AddEvent("NoActiveCamera");
    Event& event = Containers::EventContainer::GetEvent("NoActiveCamera");
    event.AddListener(this, &RenderSystem::ResetActiveCamera);*/
}

void Engine::Systems::RenderSystem::DrawScene()
{
    HRESULT hr;

    if (Containers::LightContainer::GetLights().begin()->second)
        std::cout << "be";

    std::shared_ptr<Rendering::Lights::ILight> ILight = Containers::LightContainer::GetLights().begin()->second;
    std::shared_ptr<Rendering::Lights::Light> light1 = std::dynamic_pointer_cast<Rendering::Lights::Light>(Containers::LightContainer::GetLights().begin()->second);
    Rendering::Lights::Light::LightData& light = light1->GetLightData();

    std::shared_ptr<Rendering::Camera> camera = Containers::CameraContainer::GetCamera(m_activeCamera);

    //TODO: Light will be moved soon
    if (ImGui::Begin("Lighting Tool"))
    {
        ImGui::SliderFloat("LightPosX", &light.position.x, -40.0f, 40.0f, "%.1f");
        ImGui::SliderFloat("LightPosY", &light.position.y, -40.0f, 40.0f, "%.1f");
        ImGui::SliderFloat("LightPosZ", &light.position.z, -40.0f, 40.0f, "%.1f");
        ImGui::SliderFloat("LightColR", &light.color.x, 0.0f, 1.0f, "%.1f");
        ImGui::SliderFloat("LightColG", &light.color.y, 0.0f, 1.0f, "%.1f");
        ImGui::SliderFloat("LightColB", &light.color.z, 0.0f, 1.0f, "%.1f");
        ImGui::SliderFloat("Ambient LightX", &light.ambient.x, 0.0f, 1.0f, "%.1f");
        ImGui::SliderFloat("Ambient LightY", &light.ambient.y, 0.0f, 1.0f, "%.1f");
        ImGui::SliderFloat("Ambient LightZ", &light.ambient.z, 0.0f, 1.0f, "%.1f");
    }ImGui::End();

    for (auto& gameObject : Containers::GameObjectContainer::GetAllGameObjects())
    {
        for (auto& component : gameObject.second->GetAllComponents())
        {
            if (std::shared_ptr<Components::ModelComponent> modelComp = std::dynamic_pointer_cast<Components::ModelComponent>(Containers::ComponentContainer::FindComponent(component)))
            {
                auto& meshes = Containers::ModelContainer::FindModel(modelComp->GetModel())->GetMeshes();

                for (auto mesh : meshes)
                {
                    mesh->Bind(Rendering::Renderer::GetInstance()->GetContext());

                    mesh->GetMaterial().GetShader().GenConstantBuffers();

                    Matrix4F model = Containers::TransformContainer::FindTransform(gameObject.second->GetTransformID())->GetTransformMatrix();

                    Matrix4F normalModel = Matrix4F::Inverse(model);

                    Matrix4F view = camera->GetViewMatrix();
                    Matrix4F perspective = camera->GetPerspectiveMatrix();

                    model.Transpose();
                    view.Transpose();
                    normalModel.Transpose();
                    perspective.Transpose();

                    Rendering::Buffers::VCB vcb { model, view, normalModel,perspective };

                    D3D11_MAPPED_SUBRESOURCE msr;
                    Rendering::Renderer::GetInstance()->GetContext()->Map(mesh->GetMaterial().GetShader().GetVCB().GetBuffer().Get(), 0u, D3D11_MAP_WRITE_DISCARD, 0u, &msr);
                    memcpy(msr.pData, &vcb, sizeof(vcb));
                    Rendering::Renderer::GetInstance()->GetContext()->Unmap(mesh->GetMaterial().GetShader().GetVCB().GetBuffer().Get(), 0u);
                    //mesh->GetMaterial().GetShader().GetVCB().Update(vcb);

                    const Vector4F reversedXLightPos = Vector4F(light.position.x * -1, light.position.y, light.position.z, 1.0f);
                    const Rendering::Buffers::PCB pcb { reversedXLightPos, light.ambient, light.diffuse,
                                                        light.specular ,light.direction, light.color,
                                                                        light.shininess,camera->GetPosition()};
                    mesh->GetMaterial().GetShader().GetPCB().Update(pcb);
                    Rendering::Renderer::GetInstance()->SetRenderTarget();
                    
                    GFX_THROW_INFO_ONLY(Rendering::Renderer::GetInstance()->GetContext()->DrawIndexed(static_cast<UINT>(mesh->GetIndices().size()), 0u, 0u));
                }
            }
        }
    }

}

void Engine::Systems::RenderSystem::Update()
{
    DrawScene();

    if (Containers::CameraContainer::GetCamera(m_activeCamera))
    {
        int width, height;
        Rendering::Renderer::GetInstance()->GetResolution(width, height);
        Containers::CameraContainer::GetCamera(m_activeCamera)->UpdateCamera(width, height);
    }
}

uint32_t Engine::Systems::RenderSystem::AddLight(Rendering::Lights::Light& p_light)
{
    //TODO: create lightManager and load light into it rather than into rendersystem
    std::shared_ptr newLight = std::make_shared<Rendering::Lights::Light>(p_light);

    if (newLight)
    {
        const uint32_t tmpId = Tools::IDCounter::GetNewID();
        m_lights.insert_or_assign(tmpId, newLight);
        return tmpId;
    }
    else
        return -1;
}

void Engine::Systems::RenderSystem::ResetActiveCamera()
{
    m_activeCamera = -1;
}

void Engine::Systems::RenderSystem::SetActiveCamera(int32_t p_id)
{
    m_activeCamera = p_id;
}
