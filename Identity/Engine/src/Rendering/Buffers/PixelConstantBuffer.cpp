#include <stdafx.h>

#include <Tools/DirectX/GraphicsMacros.h>

#include <Rendering/Buffers/PixelConstantBuffer.h>

using namespace Engine::Rendering::Buffers;

void PixelConstantBuffer::GenBuffers()
{
    HRESULT hr;

    D3D11_BUFFER_DESC pixelBufferDesc              = {};
    pixelBufferDesc.BindFlags                      = D3D11_BIND_CONSTANT_BUFFER;
    pixelBufferDesc.Usage                          = D3D11_USAGE_DYNAMIC;        //Dynamic - values can change
    pixelBufferDesc.CPUAccessFlags                 = D3D11_CPU_ACCESS_WRITE;
    pixelBufferDesc.MiscFlags                      = 0u;
    pixelBufferDesc.ByteWidth                      = sizeof(m_pcb);
    pixelBufferDesc.StructureByteStride            = 0u;
    D3D11_SUBRESOURCE_DATA PixelConstantShaderData = {};
    PixelConstantShaderData.pSysMem                = &m_pcb;
    GFX_THROW_INFO(Renderer::GetInstance()->GetDevice()->CreateBuffer(&pixelBufferDesc, &PixelConstantShaderData, &m_buffer
                   ));
}

void PixelConstantBuffer::Bind() const
{
    Renderer::GetInstance()->GetContext()->PSSetConstantBuffers(0u, 1u, m_buffer.GetAddressOf());
}

void PixelConstantBuffer::Unbind() const
{
    Renderer::GetInstance()->GetContext()->PSSetConstantBuffers(0u, 0u, nullptr);
}

void PixelConstantBuffer::Update(const PCB& p_filledBuffer) const
{
    D3D11_MAPPED_SUBRESOURCE msr;
    Renderer::GetInstance()->GetContext()->Map(m_buffer.Get(), 0u, D3D11_MAP_WRITE_DISCARD, 0u, &msr);
    memcpy(msr.pData, &p_filledBuffer, sizeof(p_filledBuffer));
    Renderer::GetInstance()->GetContext()->Unmap(m_buffer.Get(), 0u);
}
