#pragma once
#include <Export.h>
#include <Rendering/Lights/ILight.h>

namespace Engine {
    namespace Components {
        class Light;
    }
}

namespace Engine::Containers
{
    class API_ENGINE LightContainer
    {
    public:
        ~LightContainer();

        LightContainer(const LightContainer&) = delete;
        LightContainer(const LightContainer&&) = delete;

        static LightContainer* GetInstance();

        /**
         * @brief NON_FUNCTIONAL: Decomment code once ILight class is functional.
         */
        static int32_t AddLight(std::shared_ptr<Rendering::Lights::ILight> p_light);
        static std::map<int32_t, std::shared_ptr<Rendering::Lights::ILight>>& GetLights() { return GetInstance()->m_lights; }
        static std::shared_ptr<Rendering::Lights::ILight> FindLight(int32_t p_id);

    private:
        LightContainer() = default;

        inline static LightContainer* m_instance = nullptr;
        std::map<int32_t, std::shared_ptr<Rendering::Lights::ILight>> m_lights;
    };
}
