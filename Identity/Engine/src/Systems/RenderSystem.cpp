#include <stdafx.h>
#include <Systems/RenderSystem.h>
#include <Containers/ModelContainer.h>
#include <Tools/DirectX/GraphicsMacros.h>
#include <Rendering/Lights/Light.h>
#include <Tools/ImGUI/imgui.h>
#include <Input/Input.h>
#include <Systems/TransformSystem.h>
#include <Systems/CameraSystem.h>
#include <Rendering/Buffers/VertexConstantBuffer.h>
#include <Containers/LightContainer.h>
#include <Scene/SceneGraph/SceneNode.h>
#include "Containers/ColliderContainer.h"
#include <Components/BoxCollider.h>

#define DEBUG_MODE true

constexpr bool DRAW_TO_TEXTURE = false;

void Engine::Systems::RenderSystem::DrawScene(float p_deltaTime)
{
    HRESULT hr;
    Rendering::Renderer::GetInstance()->GetContext()->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

    std::shared_ptr<Rendering::Lights::ILight> ILight = Containers::LightContainer::GetLights().begin()->second;
    std::shared_ptr<Rendering::Lights::Light> light1 = std::dynamic_pointer_cast<Rendering::Lights::Light>(Containers::LightContainer::GetLights().begin()->second);

    Rendering::Lights::Light::LightData& light = light1->GetLightData();

    auto camera = Containers::CameraSystem::GetCamera(m_activeCamera);

    float* pos[3] = { &light.position.x, &light.position.y, &light.position.z };

    //TODO: Light will be moved soon
    if (ImGui::Begin("Lighting Tool"))
    {
        ImGui::DragFloat3("LightPos", *pos, 0.1f, -90.0f, 90.0f, "%.1f");
        ImGui::SliderFloat("LightColR", &light.color.x, 0.0f, 1.0f, "%.1f");
        ImGui::SliderFloat("LightColG", &light.color.y, 0.0f, 1.0f, "%.1f");
        ImGui::SliderFloat("LightColB", &light.color.z, 0.0f, 1.0f, "%.1f");
        ImGui::SliderFloat("SpecColR", &light.specular.x, 0.0f, 1.0f, "%.1f");
        ImGui::SliderFloat("SpecColG", &light.specular.y, 0.0f, 1.0f, "%.1f");
        ImGui::SliderFloat("SpecColB", &light.specular.z, 0.0f, 1.0f, "%.1f");
        ImGui::SliderFloat("Ambient LightX", &light.ambient.x, 0.0f, 1.0f, "%.1f");
        ImGui::SliderFloat("Ambient LightY", &light.ambient.y, 0.0f, 1.0f, "%.1f");
        ImGui::SliderFloat("Ambient LightZ", &light.ambient.z, 0.0f, 1.0f, "%.1f");
    }ImGui::End();

/* W.I.P. 
    for (auto& gameObject : Containers::GameObjectContainer::GetAllGameObjects())
    {
        if (ImGui::Begin("ObjectInfo"))
        {
            ImGui::Text(gameObject.second->GetName().c_str());
            ImGui::Text("Forward: %f | %f | %f", gameObject.second->GetTransform()->GetForward().x, gameObject.second->GetTransform()->GetForward().y, gameObject.second->GetTransform()->GetForward().z);
            ImGui::Text("Up: %f | %f | %f", gameObject.second->GetTransform()->GetUp().x, gameObject.second->GetTransform()->GetUp().y, gameObject.second->GetTransform()->GetUp().z);
            ImGui::Text("Right: %f | %f | %f", gameObject.second->GetTransform()->GetRight().x, gameObject.second->GetTransform()->GetRight().y, gameObject.second->GetTransform()->GetRight().z);
            ImGui::Text("Position: %f | %f | %f", gameObject.second->GetTransform()->GetPosition().x, gameObject.second->GetTransform()->GetPosition().y, gameObject.second->GetTransform()->GetPosition().z);

        }ImGui::End();
    }*/

    for (auto& sceneNode : Scene::SceneGraph::GetInstance()->GetRootSceneNodes())
    {
        DrawSceneNode(sceneNode.second);
    }

    if (DEBUG_MODE)
    {
        for (auto collider: Containers::ColliderContainer::GetColliders())
        {
            auto model = collider.second->GetModel();
            auto mesh = model->GetMeshes()[0];
            mesh->GenerateBuffers(Rendering::Renderer::GetInstance()->GetDevice());
            mesh->Bind(Rendering::Renderer::GetInstance()->GetContext());

            Matrix4F modelMatrix = collider.second->GetWorldMatrix();
            Matrix4F normalModel = Matrix4F::Inverse(modelMatrix);

            Matrix4F view = camera->GetViewMatrix();
            Matrix4F perspective = camera->GetPerspectiveMatrix();
            
            Rendering::Buffers::VCB vcb{ modelMatrix, view, normalModel,perspective };
            mesh->GetMaterial().GetShader().GetVCB().Update(vcb);
            const Vector3F cameraPos = camera->GetPosition();

            const Rendering::Buffers::PCB pcb{ Vector4F::zero, Vector4F::one, Vector4F::one,
                                            Vector4F::zero, Vector4F::one,
                                                            1.0f,Vector3F{},Vector3F::zero, 0.0f };
            mesh->GetMaterial().GetShader().GetPCB().Update(pcb);
            Rendering::Renderer::GetInstance()->GetContext()->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINESTRIP);

            if (DRAW_TO_TEXTURE)
            {
                Rendering::Renderer::GetInstance()->Bind(Rendering::Renderer::GetInstance()->GetRenderTextures()[0].GetTarget(), Rendering::Renderer::GetInstance()->GetRenderTextures()[0].GetDepthStencilView());
                GFX_THROW_INFO_ONLY(Rendering::Renderer::GetInstance()->GetContext()->DrawIndexed(static_cast<UINT>(mesh->GetIndices().size()), 0u, 0u));
                Rendering::Renderer::GetInstance()->Bind();
            }
            else
            {
                Rendering::Renderer::GetInstance()->Bind();
                GFX_THROW_INFO_ONLY(Rendering::Renderer::GetInstance()->GetContext()->DrawIndexed(static_cast<UINT>(mesh->GetIndices().size()), 0u, 0u));
            }
            Rendering::Renderer::GetInstance()->GetContext()->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        }
    }

    if (DRAW_TO_TEXTURE)
    {
        auto camera = Containers::CameraSystem::GetCamera(m_activeCamera);

        // I have to invert Y value for some reason I don't understand yet.
        std::vector<Geometry::Vertex> quadvtx{ Geometry::Vertex{ Vector3F{-1.0f, 1.0f, 0.f}, Vector2F{0.0f, -1.0f}, Vector3F::zero },
                                               Geometry::Vertex{ Vector3F{1.0f, 1.0f, 0.f}, Vector2F{1.0f, -1.0f}, Vector3F::zero },
                                               Geometry::Vertex{ Vector3F{-1.0f, -1.0f, 0.f}, Vector2F{0.0f, 0.0f}, Vector3F::zero },
                                               Geometry::Vertex{ Vector3F{1.0f, -1.0f, 0.f}, Vector2F{1.0f, 0.0f}, Vector3F::zero } };
        std::vector<unsigned short> quadidx = { 3,2,1,1,2,0 };

        //The quad is the screen "camera rect" we might want to store it somewhere later.
        ObjectElements::Mesh quad{ quadvtx, quadidx };
        quad.GenerateBuffers(Rendering::Renderer::GetInstance()->GetDevice());

        quad.Bind(Rendering::Renderer::GetInstance()->GetContext());

        const Rendering::Buffers::VCB vcb{ Matrix4F::identity, Matrix4F::identity, Matrix4F::identity,Matrix4F::identity };
        quad.GetMaterial().GetShader().GetVCB().Update(vcb);

        const Rendering::Buffers::PCB pcb{ Vector4F::zero, Vector4F::one, Vector4F::one,
                                            Vector4F::zero, Vector4F::one,
                                                            1.0f,Vector3F{},Vector3F::zero, 0.0f };
        quad.GetMaterial().GetShader().GetPCB().Update(pcb);

        quad.GetMaterial().GetTexture().SetTexSRV(Rendering::Renderer::GetInstance()->GetRenderTextures()[0].GetShaderResourceView());

        Rendering::Renderer::GetInstance()->Bind();
        GFX_THROW_INFO_ONLY(Rendering::Renderer::GetInstance()->GetContext()->DrawIndexed(static_cast<UINT>(quad.GetIndices().size()), 0u, 0u));
    }
}

void Engine::Systems::RenderSystem::DrawSceneNode(std::shared_ptr<Scene::SceneNode> p_sceneNode)
{
    auto camera = Containers::CameraSystem::GetCamera(m_activeCamera);
    auto mesh = p_sceneNode->GetMesh();
    std::shared_ptr<Rendering::Lights::Light> light1 = std::dynamic_pointer_cast<Rendering::Lights::Light>(Containers::LightContainer::GetLights().begin()->second);
    Rendering::Lights::Light::LightData& light = light1->GetLightData();

    if (mesh != nullptr)
    {
        mesh->Bind(Rendering::Renderer::GetInstance()->GetContext());

        Matrix4F model = Containers::TransformSystem::FindTransform(p_sceneNode->GetTransform())->GetWorldTransformMatrix();

        Matrix4F normalModel = Matrix4F::Inverse(model);

        Matrix4F view = camera->GetViewMatrix();
        Matrix4F perspective = camera->GetPerspectiveMatrix();

        Rendering::Buffers::VCB vcb{ model, view, normalModel,perspective };
        mesh->GetMaterial().GetShader().GetVCB().Update(vcb);
        const Vector3F cameraPos = camera->GetPosition();

        const Vector4F reversedXLightPos = Vector4F(light.position.x, light.position.y, -light.position.z, 1.0f);
        const Rendering::Buffers::PCB pcb{ reversedXLightPos, light.ambient, light.diffuse,
                                            light.specular , light.color,
                                                            light.shininess,Vector3F{},Vector3{cameraPos.x, cameraPos.y, cameraPos.z}, 0.0f };


        mesh->GetMaterial().GetShader().GetPCB().Update(pcb);
        if (DRAW_TO_TEXTURE)
        {
            Rendering::Renderer::GetInstance()->Bind(Rendering::Renderer::GetInstance()->GetRenderTextures()[0].GetTarget(), Rendering::Renderer::GetInstance()->GetRenderTextures()[0].GetDepthStencilView());
            GFX_THROW_INFO_ONLY(Rendering::Renderer::GetInstance()->GetContext()->DrawIndexed(static_cast<UINT>(mesh->GetIndices().size()), 0u, 0u));
            Rendering::Renderer::GetInstance()->Bind();
        }
        else
        {
            Rendering::Renderer::GetInstance()->Bind();
            GFX_THROW_INFO_ONLY(Rendering::Renderer::GetInstance()->GetContext()->DrawIndexed(static_cast<UINT>(mesh->GetIndices().size()), 0u, 0u));
        }
    }

    for (auto child : p_sceneNode->GetChildren())
    {
        DrawSceneNode(child);
    }
}

void Engine::Systems::RenderSystem::IUpdate(float p_deltaTime)
{
    Scene::SceneGraph::GetInstance()->UpdateScene(p_deltaTime);
    DrawScene(p_deltaTime);
}

void Engine::Systems::RenderSystem::ResetActiveCamera()
{
    m_activeCamera = -1;
}

void Engine::Systems::RenderSystem::SetActiveCamera(int32_t p_id)
{
    m_activeCamera = p_id;
}