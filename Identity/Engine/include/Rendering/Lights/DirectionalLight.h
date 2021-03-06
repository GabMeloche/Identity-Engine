#pragma once
#include <Rendering/Lights/ILight.h>
#include <Windows.h>

namespace Engine::Rendering::Lights
{
    class DirectionalLight : public ILight
    {
    public:
        DirectionalLight();
        DirectionalLight(const Vector4F& p_position, const Vector4F&  p_ambient, const Vector4F& p_diffuse, const Vector4F& p_specular,
              const Vector4F& p_direction, const float& p_shininess);
        DirectionalLight(const LightData& p_lightData);
        DirectionalLight(const DirectionalLight&) = delete;
        DirectionalLight(DirectionalLight&&) = delete;

        bool operator==(Rendering::Lights::ILight* p_other) override;
        void Deserialize(std::vector<std::string>& p_block) override;
        void Serialize(std::ostream& p_stream) override;

    };
}
