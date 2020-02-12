#include <stdafx.h>
#include <Core/App.h>
#include <Tools/ImGUI/imgui.h>
#include <Tools/ImGUI/imgui_impl_win32.h>
#include <Tools/ImGUI/imgui_impl_dx11.h>
#include <3DLoader/ObjectElements/Transform.h>
#include <Events/IEventCallback.h>
#include "Events/Event.h"
#include <Systems/RenderSystem.h>
#include <Input/Input.h>
#include <Objects/GameObject.h>
#include "Components/ModelComponent.h"
#include "Components/CameraComponent.h"
#include "Containers/CameraContainer.h"

using namespace Engine::Core;

App::App() : m_window(800, 600, "Engine Window")
{
    Input::Input::InitInput();
}

App::App(int p_width, int p_height, const char* p_name) : m_window(p_width, p_height, p_name)
{
    Input::Input::InitInput();
}

int App::Run() const
{
    Containers::ModelContainer::GetInstance()->SetGraphicsDevice(m_window.GetRenderer().GetDevice());

    Systems::RenderSystem renderSystem(&m_window.GetRenderer());

    Objects::GameObject gameObject;
    Objects::GameObject gameObject2;
    Objects::GameObject camera;

    gameObject.GetTransform()->Translate(Vector3F{3.0f, 0.0f, 4.0f});
    gameObject2.GetTransform()->Translate(Vector3F{6.0f, 0.0f, -4.0f});
    gameObject.GetTransform()->Scale(Vector3F{0.02f, 0.02f, 0.02f});
    gameObject2.GetTransform()->Scale(Vector3F{0.02f, 0.02f, 0.02f});

    int width  = 1024;
    int height = 768;

    int32_t cameraComponentID = camera.AddComponent<Components::CameraComponent>(width, height);
    gameObject.AddComponent<Components::ModelComponent>("../Engine/Resources/statue.obj", "statue");
    gameObject2.AddComponent<Components::ModelComponent>("../Engine/Resources/Lambo.obj", "lambo");

    renderSystem.SetActiveCamera(camera.FindComponent<Components::CameraComponent>()->GetCamera()->GetID());

    Rendering::Light dirLight{};

    dirLight.position  = Vector4F(40.0f, 40.0f, -40.0f, 1.0f);
    dirLight.ambient   = Vector4F(0.1f, 0.1f, 0.1f, 1.0f);
    dirLight.diffuse   = Vector4F(1.0f, 1.0f, 0.95f, 1.0f);
    dirLight.specular  = Vector4F(1.0f, 1.0f, 0.95f, 1.0f);
    dirLight.direction = Vector4F(-0.5f, -0.5f, -0.5f, 1.0f).Normalize();
    dirLight.color     = Vector4F(1.0f, 1.0f, 1.0f, 1.0f);
    dirLight.shininess = 64.0f;

    //TODO: move to LightContainer once class is finalized.
    renderSystem.AddLight(dirLight);

    while (true)
    {
        if (const auto eCode = Rendering::Window::ProcessMessage())
        {
            return *eCode;
        }

        DoFrame(renderSystem);
    }
}

void App::DoFrame(Engine::Systems::RenderSystem& p_renderSystem) const
{
    m_window.GetRenderer().ClearBuffer(0.3f, 0.3f, 0.3f);
    if (_INPUT->keyboard.IsKeyHeld('R'))
        m_window.GetRenderer().ClearBuffer(1.0f, 0.0f, 0.0f);

    if (_INPUT->keyboard.IsKeyHeld('G'))
        m_window.GetRenderer().ClearBuffer(0.0f, 1.0f, 0.0f);

    if (_INPUT->keyboard.IsKeyHeld('B'))
        m_window.GetRenderer().ClearBuffer(0.0f, 0.0f, 1.0f);


    if (_INPUT->keyboard.IsKeyDown('F'))
    {
        m_window.GetRenderer().SetFullscreen(!m_window.GetRenderer().GetFullscreenState());
    }

    ImGui_ImplDX11_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();

    p_renderSystem.UpdateCamera();
    p_renderSystem.Update();

    static bool show_demo_window = true;
    ImGui::Begin("Identity UI Tools");
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::End();

    ImGui::Render();
    ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

    ImGuiIO& io = ImGui::GetIO(); (void)io;
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
    }

    m_window.GetRenderer().EndFrame();
}
