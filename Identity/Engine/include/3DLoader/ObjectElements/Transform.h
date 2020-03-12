#pragma once
#include <Export.h>
#include <GPM/GPM.h>
#include <Objects/IObject.h>

namespace Engine::ObjectElements
{
    class API_ENGINE Transform : public Objects::IObject
    {
    public:
        Transform();

        Transform(const std::string& p_name);
        Transform(Vector3F& p_position);
        ~Transform() = default;

        Transform(const Transform& p_other);
        //Transform(const Transform&& p_other) noexcept;

        /** 
         * @brief Adds the given vector to the current position.
         */
        void Translate(const Vector3F& p_vector);

        /**
         *@brief Rotates transform by a vector of Euler angles.
         *@warning: may contain error at m_rotation *= quat
         */
        void RotateWithEulerAngles(const Vector3F& p_euler);

        /**
         *@brief Multiplies the Transform's current scale by the given vector.
          
         */
        void Scale(const Vector3F& p_scale);


        inline void SetPosition(const Vector3F& p_position) { m_position = p_position; }
        inline void SetScale(const Vector3F& p_scale) { m_scale = p_scale; }
        inline void SetRotation(const Quaternion& p_rotation)
        {
            m_rotation = p_rotation;
            CalculateAxes();
        }

        inline void SetWorldTransformMatrix(const Matrix4F& p_transform) { m_worldTransform = p_transform; }
        inline void SetLocalTransformMatrix(const Matrix4F& p_transform) { m_localTransform = p_transform; }
        void UpdateWorldTransformMatrix();

        inline void SetParent(int32_t p_parent) { m_parent = p_parent; }

        [[nodiscard]] Vector3F GetEuler() const;
        inline Vector3F& GetPosition() { return m_position; }
        inline Vector3F& GetForward() { return m_forward; }
        inline Vector3F& GetRight() { return m_right; }
        inline Vector3F& GetUp() { return m_up; }
        inline Vector3F& GetScale() { return m_scale; }
        inline Quaternion& GetRotation() { return m_rotation; }
        inline std::shared_ptr<Transform> GetParent() const;
        inline Matrix4F& GetWorldTransformMatrix() { return m_worldTransform; }
        inline Matrix4F& GetLocalTransformMatrix() { return m_localTransform; }

    private:
        void CalculateAxes();

        int32_t m_parent = -1;
        Vector3F m_position;
        Vector3F m_forward;
        Vector3F m_right;
        Vector3F m_up;
        Vector3F m_scale;
        Quaternion m_rotation;

        Matrix4F m_worldTransform;
        Matrix4F m_localTransform;
    };
}
