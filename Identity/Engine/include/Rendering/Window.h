#pragma once
#include <Export.h>
#include <WinSetup.h>
#include <Rendering/Graphics.h>
#include <memory>

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

        Graphics& GetGraphics() const;

    private:
        static LRESULT CALLBACK HandleMsgSetup(HWND p_hwnd, UINT p_msg, WPARAM p_wParam, LPARAM p_lParam);
        static LRESULT CALLBACK HandleMsgThunk(HWND p_hwnd, UINT p_msg, WPARAM p_wParam, LPARAM p_lParam);
        LRESULT HandleMsg(HWND p_hwnd, UINT p_msg, WPARAM p_wParam, LPARAM p_lParam);

        int m_width;
        int m_height;
        HWND m_hwnd;

        std::unique_ptr<Graphics> m_graphics;
    };
}