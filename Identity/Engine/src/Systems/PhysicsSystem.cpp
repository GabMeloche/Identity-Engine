#include <stdafx.h>

#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>

#include <Components/BoxCollider.h>
#include <Components/SphereCollider.h>
#include <Physics/CollisionInfo.h>
#include <Objects/GameObject.h>
#include <Systems/PhysicsSystem.h>
#include <Tools/Time.h>

Engine::Systems::PhysicsSystem::PhysicsSystem()
{
    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
    m_overlappingPairCache = new btDbvtBroadphase();
    m_solver = new btSequentialImpulseConstraintSolver;

    m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_overlappingPairCache, m_solver, m_collisionConfiguration);

    m_dynamicsWorld->setGravity(btVector3(0.0f, -10.0f, 0.0f));
    m_dynamicsWorld->setInternalTickCallback(BulletTickCallback);
}

Engine::Systems::PhysicsSystem::~PhysicsSystem()
{
    delete m_collisionConfiguration;
    delete m_dispatcher;
    delete m_overlappingPairCache;
    delete m_solver;
    delete m_dynamicsWorld;
    delete m_instance;
}

Engine::Systems::PhysicsSystem* Engine::Systems::PhysicsSystem::GetInstance()
{
    if (m_instance == nullptr)
    {
        m_instance = new PhysicsSystem();
    }

    return m_instance;
}

std::shared_ptr<Engine::Physics::ICollider> Engine::Systems::PhysicsSystem::AddCollider(Physics::ICollider* p_collider)
{
    auto coll = std::shared_ptr<Physics::ICollider>(p_collider);
    GetInstance()->m_colliders.insert_or_assign(p_collider->GetID(), coll);
    GetInstance()->m_dynamicsWorld->addRigidBody(coll->GetBtRigidbody());
    return coll;
}

void Engine::Systems::PhysicsSystem::RemoveCollider(int32_t p_id)
{
    Managers::ResourceManager::RemoveModel(GetInstance()->m_colliders.at(p_id)->GetModel()->GetID());
    GetInstance()->m_colliders.erase(p_id);
}

void Engine::Systems::PhysicsSystem::Update(const float p_deltaTime)
{
    for (auto& collider : GetInstance()->m_colliders)
    {
        if (!collider.second->IsActive())
            continue;

        //Calculate Collider's position with offset
        btTransform trans;
        trans.setIdentity();
        auto position = collider.second->GetGameObject()->GetTransform()->GetPosition();
        auto rotation = collider.second->GetGameObject()->GetTransform()->GetRotation();

        Vector3D offsetD = collider.second->GetOffset();
        Quaternion q = rotation * offsetD * Quaternion::Conjugate(rotation);
        offsetD = { q.GetXAxisValue(), q.GetYAxisValue(), q.GetZAxisValue() };
        Vector3D offset = offsetD;
        position += offset;

        //Set Collider's transform in Bullet Physics world
        trans.setIdentity();
        trans.setOrigin(btVector3(position.x, position.y, position.z));
        trans.setRotation(btQuaternion(rotation.GetXAxisValue(), rotation.GetYAxisValue(), rotation.GetZAxisValue(), rotation.w));

        collider.second->GetBtRigidbody()->setWorldTransform(trans);
        collider.second->GetMotionState()->setWorldTransform(trans);
    }
}

//Called every 0.016 seconds, or ~60 times per second in Core::App
void Engine::Systems::PhysicsSystem::FixedUpdate()
{
    btTransform trans;

    //Set collision variables before simulation
    for (auto& collider : GetInstance()->m_colliders)
    {
        if (collider.second->IsActive())
        {
            collider.second->SetHasCollidedLastFrame(collider.second->GetHasCollidedThisFrame());
            collider.second->SetHasCollidedThisFrame(false);
        }
    }

    //Bullet simulate physics
    GetInstance()->m_dynamicsWorld->stepSimulation(0.016f, 0.0f);

    //Update collider positions
    for (auto& collider : GetInstance()->m_colliders)
    {
        if (collider.second->IsActive())
        {
            //TODO: wrap all Bullet math variables
            collider.second->GetMotionState()->getWorldTransform(trans);
            Vector3F offset = collider.second->GetOffset();

            btVector3& collPos = trans.getOrigin();
            btQuaternion collRot = trans.getRotation();
            btQuaternion quatOffset(offset.x, offset.y, offset.z, 0.0f);
            btQuaternion qpq = collRot * quatOffset * collRot.inverse();

            collider.second->GetGameObject()->GetTransform()->SetPosition(Vector3F(collPos.getX(), collPos.getY(), collPos.getZ()) -
                Vector3F(qpq.getX(), qpq.getY(), qpq.getZ()));

            collider.second->GetGameObject()->GetTransform()->SetRotation(Quaternion(collRot.getX(), collRot.getY(),
                collRot.getZ(), collRot.getW()));

            collider.second->GetGameObject()->GetTransform()->UpdateWorldTransformMatrix();
        }
    }

    //Call Collision events
    for (auto& collider : GetInstance()->m_colliders)
    {
        if (collider.second->IsActive())
        {
            collider.second->ActOnCollisionInfo();
        }
    }
}

//Called during m_dynamicsWorld->StepSimulation(); Checks for collisions and creates CollisionInfos if needed
void Engine::Systems::PhysicsSystem::BulletTickCallback(btDynamicsWorld* p_world, btScalar p_timeStep)
{
    int numManifolds = GetInstance()->m_dynamicsWorld->getDispatcher()->getNumManifolds();
     
    for (int i = 0; i < numManifolds; i++)
    {
        btPersistentManifold* contactManifold = GetInstance()->m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject* obA = static_cast<const btCollisionObject*>(contactManifold->getBody0());
        const btCollisionObject* obB = static_cast<const btCollisionObject*>(contactManifold->getBody1());

        int numContacts = contactManifold->getNumContacts();

        for (int j = 0; j < numContacts; j++)
        {
            btManifoldPoint& pt = contactManifold->getContactPoint(j);

            if (pt.getDistance() < 0.02f)
            {
                const btVector3& ptA = pt.getPositionWorldOnA();
                const btVector3& ptB = pt.getPositionWorldOnB();
                const btVector3& normalOnB = pt.m_normalWorldOnB;

                if (auto collA = FindCollider(obA))
                {
                    if (auto collB = FindCollider(obB))
                    {
                        collA->SetCollisionInfo(std::make_shared<Physics::CollisionInfo>(
                            GPM::Vector3F{ ptA.getX(), ptA.getY(), ptA.getZ() },
                            GPM::Vector3F{ normalOnB.getX(), normalOnB.getY(), normalOnB.getZ() },
                            collB->GetGameObject()));

                        collA->SetHasCollidedThisFrame(true);

                        collB->SetCollisionInfo(std::make_shared<Physics::CollisionInfo>(
                            GPM::Vector3F{ ptB.getX(), ptB.getY(), ptB.getZ() },
                            GPM::Vector3F{ normalOnB.getX(), normalOnB.getY(), normalOnB.getZ() },
                            collA->GetGameObject()));

                        collB->SetHasCollidedThisFrame(true);
                    }
                }
            }
        }

    }
}


std::shared_ptr<Engine::Physics::ICollider> Engine::Systems::PhysicsSystem::FindCollider(
    const btCollisionObject* p_collisionObject)
{
    for (auto& collider : GetInstance()->m_colliders)
    {
        if (collider.second->IsActive())
        {
            if (static_cast<const btCollisionObject*>(collider.second->GetBtRigidbody()) == p_collisionObject)
                return collider.second;
        }
    }

    return nullptr;
}