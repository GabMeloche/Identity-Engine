#pragma once
#include <Export.h>
#include <Components/IComponent.h>
#include <Physics/Rigidbody.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <LinearMath/btDefaultMotionState.h>
#include <3DLoader/ObjectElements/Model.h>
#include <Components/Transform.h>
#include <Physics/ICollider.h>


namespace Engine::Physics
{
    class CollisionInfo;
}

namespace Engine::Components
{

    class API_ENGINE BoxCollider: public Physics::ICollider
    {
    public:
        BoxCollider(Objects::GameObject* p_gameObject, std::shared_ptr<BoxCollider> p_other);
        BoxCollider(Objects::GameObject* p_gameObject);
        BoxCollider(Objects::GameObject* p_gameObject, std::vector<std::string> p_block);
        BoxCollider(const BoxCollider&) = default;
        ~BoxCollider();

        void Serialize(std::ostream& p_stream) override;
        void Deserialize(Objects::GameObject* p_gameObject, std::vector<std::string>& p_block) override;
        /***
         * @brief returns Bullet Physics' Matrix that concerns this BoxCollider.
         */
        
        btBoxShape* GetBtBoxShape() { return m_box; }
        Vector3F& GetDimensions() { return m_dimensions; }
        
        

        void SetMass(float p_mass);
        void SetDimensions(const GPM::Vector3F& p_dimensions);

        bool operator==(IComponent* p_other) override { return false; }
        bool DeleteFromMemory() override;
        void SetActive(bool p_active) override;

        bool RemoveComponent() override;
        void SetPositionOffset(GPM::Vector3F p_offset) override; 

    private:
        /**
         * @brief Builds a Model that visually represents this BoxCollider's transform. 
         */
        ObjectElements::Model ConstructBox();

        Vector3F m_dimensions;
        btBoxShape* m_box;
    };
}
