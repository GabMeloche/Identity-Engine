#include <stdafx.h>
#include <Components/SphereCollider.h>
#include <Objects/GameObject.h>

#include <Components/Sound.h>
#include <Components/BoxCollider.h>
#include <Managers/SceneManager.h>
#include <Scene/Scene.h>

void Engine::Physics::ICollider::OnCollisionEnter()
{
    if (m_gameObject->FindComponentOfType<Components::Sound>())
    {
        m_gameObject->FindComponentOfType<Components::Sound>()->Stop();
        m_gameObject->FindComponentOfType<Components::Sound>()->PlaySound();
    }
    if (m_gameObject->GetName() == "Ball")
    {
        auto support = Managers::SceneManager::GetActiveScene()->GetGameObject("Steep1SupportRight");
        if (support && m_collisionInfo->GetCollision()->GetName() == "Steep1")
            support->FindComponentOfType<Components::BoxCollider>()->SetMass(10);
    }
    if (m_gameObject->GetName() == "Ball" && m_collisionInfo->GetCollision()->GetName() == "Steep2")
    {
        auto support = Managers::SceneManager::GetActiveScene()->GetGameObject("Steep2SupportLeft");
        if (support)
            support->FindComponentOfType<Components::BoxCollider>()->SetMass(10);
    }
    if (m_gameObject->GetName() == "Ball" && m_collisionInfo->GetCollision()->GetName() == "Steep3")
    {
        auto support = Managers::SceneManager::GetActiveScene()->GetGameObject("Steep3SupportRight");
        if (support)
            support->FindComponentOfType<Components::BoxCollider>()->SetMass(10);
    }
    if (m_gameObject->GetName() == "FlyingLink" && m_collisionInfo->GetCollision()->GetName() == "Stoper")
    {
        m_collisionInfo->GetCollision()->FindComponentOfType<Components::BoxCollider>()->SetMass(1);
    }

    std::string str(m_gameObject->GetName() + " On collision enter with " + m_collisionInfo->GetCollision()->GetName() + "\n");
    //OutputDebugString(str.c_str());
}

void Engine::Physics::ICollider::OnCollisionStay()
{
    std::string str(m_gameObject->GetName() + " On collision stay with " + m_collisionInfo->GetCollision()->GetName() + "\n");
    //OutputDebugString(str.c_str());
}

void Engine::Physics::ICollider::OnCollisionExit()
{
    if (m_gameObject->GetName() == "Ball (1)" && m_collisionInfo->GetCollision()->GetName() == "Steep (1)")
    {
        std::string str();
        //OutputDebugString(str.c_str());
    }
}

void Engine::Physics::ICollider::ActOnCollisionInfo()
{
    if (m_hasCollidedThisFrame && m_hasCollidedLastFrame)
        OnCollisionStay();
    else if (m_hasCollidedThisFrame && !m_hasCollidedLastFrame)
        OnCollisionEnter();
    else if (!m_hasCollidedThisFrame && m_hasCollidedLastFrame)
    {
        OnCollisionExit();
        m_collisionInfo = nullptr;
        m_hasCollidedLastFrame = false;
        m_hasCollidedThisFrame = false;
    }
}


Matrix4F Engine::Physics::ICollider::GetWorldMatrix() const
{
    btScalar    m[16];
    btTransform trans;

    m_motionState->getWorldTransform(trans);
    trans.getOpenGLMatrix(m);

    Matrix4F mat
    {
        m[0], m[1], m[2], m[3],
        m[4], m[5], m[6], m[7],
        m[8], m[9], m[10], m[11],
        m[12], m[13], m[14], m[15]
    };

    return mat.Transpose();
}

GPM::Vector3F Engine::Physics::ICollider::GetVelocity()
{
    if (!m_rigidbody)
        return GPM::Vector3F::zero;

    const btVector3& vel = m_rigidbody->getLinearVelocity();
    return std::move(GPM::Vector3F(vel.getX(), vel.getY(), vel.getZ()));
}

bool Engine::Physics::ICollider::IsColliding()
{
    if (m_hasCollidedThisFrame && m_hasCollidedLastFrame)
        return true;

    return false;
}
