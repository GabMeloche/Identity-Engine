#pragma once
#include <Export.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <LinearMath/btAlignedObjectArray.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>

namespace Engine {
    namespace Components {
        class BoxCollider;
        class SphereCollider;
    }
}

namespace Engine::Systems
{
    class API_ENGINE PhysicsSystem
    {
    public:
        ~PhysicsSystem();

        static PhysicsSystem* GetInstance();

        static std::shared_ptr<Physics::ICollider> AddCollider(Physics::ICollider* p_collider);

        static void RemoveCollider(int32_t p_id);

        static std::map<int32_t, std::shared_ptr<Physics::ICollider>>& GetColliders() { return GetInstance()->m_colliders; }
        static btDiscreteDynamicsWorld* GetWorld() { return GetInstance()->m_dynamicsWorld; }
        static void Update(const float p_deltaTime);
        static void FixedUpdate();
        static void BulletTickCallback(btDynamicsWorld* p_world, btScalar p_timeStep);
        static std::shared_ptr<Engine::Physics::ICollider> FindCollider(const btCollisionObject* p_collisionObject);

    private:
        PhysicsSystem();

        float m_fixedUpdateCounter = 0.0f;

        inline static PhysicsSystem* m_instance;
        std::map<int32_t, std::shared_ptr<Physics::ICollider>> m_colliders;

        //TODO: create class that wraps bullet classes
        btDefaultCollisionConfiguration* m_collisionConfiguration;
        btCollisionDispatcher* m_dispatcher;
        btBroadphaseInterface* m_overlappingPairCache;
        btSequentialImpulseConstraintSolver* m_solver;

        btDiscreteDynamicsWorld* m_dynamicsWorld;
    };
}
