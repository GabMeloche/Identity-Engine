struct VSOut
{
    float4 col : Colour;
    float4 pos : SV_Position;
    float3 worldPos : WPos;
    float3 norm : Normal;
    float2 tex : TexCoord;
};

cbuffer CBuf
{
    matrix model;
    matrix view;
    matrix normalModel;
    matrix projection;
};

VSOut main( float3 vpos : Position, float3 vnorm: Normal, float2 tex : TxCoord)
{
    VSOut vso;
    vso.col = float4(1.0f, 1.0f, 1.0f, 1.0f);
    vso.pos = mul(float4(vpos.x, vpos.y, vpos.z, 1.0f), mul(model, mul(view , projection)));
    vso.worldPos = float4(mul(float4(vpos, 1.0f), model)).rgb;
    vso.norm = float3(mul(normalModel, float4(vnorm, 1)).rgb);
    vso.tex = tex;
    return vso;
}
