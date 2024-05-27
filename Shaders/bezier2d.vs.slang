struct vsOut
{
    float4 pos : SV_POSITION;
    float3 col : COLOR;
    float3 pixelpos : POSITION;
};

struct vsIn
{
    float3 pos : POSITION;
    float3 col : COLOR;
};

cbuffer vsCb
{
    float4x4 viewproj;
    float4x4 model;
    
};


vsOut main(vsIn input)
{
    vsOut output;
    
        float3 p = mul(model,float4(input.pos, 1)).xyz;
        output.pos = mul(viewproj,float4(p, 1));
        output.col = input.col;
        output.pixelpos = p;
   
    return output;
}
