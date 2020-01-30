#pragma once
#include <Export.h>
#include <WinSetup.h>
#include <Rendering/Renderer.h>
#include <memory>
#include <Input/Keyboard.h>
#include <optional>
#include <Input/Mouse.h>

namespace Engine::Rendering
{
    class API_ENGINE Window
    {
    private:
        class API_ENGINE WindowClass
        {
        public:
            static const char* GetName();
            static HINSTANCE GetInstance();

        private:
            WindowClass();
            ~WindowClass();
            WindowClass(const WindowClass&) = delete;
            WindowClass& operator=(const WindowClass&) = delete;

            static constexpr const char* m_windowClassName = "Identity Engine";
            static WindowClass m_windowClass;
            HINSTANCE m_hinstance;
        };

    public:
        Window(int p_width, int p_height, const char* p_name);
        ~Window();
        Window() = delete;

        /*
         @brief Return the DirectX context of the window
         */
        [[nodiscard]] Engine::Rendering::Renderer& GetRenderer() const;
        void SetTitle(const std::string& title) const;
        /*
         @brief Check if there is a message in the queue
         If there is no message, the method will return an empty optional and continue
         If there is a message, the method will return the message
         */
        static std::optional<int> ProcessMessage();
    private:
        static LRESULT CALLBACK HandleMsgSetup(const HWND p_hwnd, const UINT p_msg, const WPARAM p_wParam, const LPARAM p_lParam);
        static LRESULT CALLBACK HandleMsgThunk(const HWND p_hwnd, const UINT p_msg, const WPARAM p_wParam, const LPARAM p_lParam);
        LRESULT HandleMsg(const HWND p_hwnd, const UINT p_msg, const WPARAM p_wParam, const LPARAM p_lParam) const;

        int m_width;
        int m_height;
        HWND m_hwnd;

        std::unique_ptr<Renderer> m_renderer;
    };
}
