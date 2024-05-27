#include "raymarch.slang"

#define PI 3.14159265359

cbuffer psCb
{
    float3 eye;
    int bezierdegree;
};

RWStructuredBuffer<float> x0;
RWStructuredBuffer<float> nChoosek;
//static const tömb nchoosek
// define tipusra

 struct psIn
{
    float4 pos : SV_POSITION;
    float3 col : COLOR;
    float3 pixelpos : POSITION;
};

float map (float3 p){
    //return abs(p.x-0.7);
    //return abs(length(p)-0.3);
    return 2*p.y*(p.y*p.y-3*p.x*p.x)*(1-p.z*p.z)+(p.x*p.x+p.y*p.y)*(p.x*p.x+p.y*p.y) - (9*p.z*p.z-1)*(1-p.z*p.z);
}

float4 main (psIn input) : SV_TARGET
{

float3 t0 = input.pixelpos;

    Ray r;
   
        r.v = normalize(t0-eye);
        r.p = t0;

        Box box;
        box.center = float3(0);
        box.radius = 1;

      // float3 t1 = intersectBox(box,r,false);

     float3 t1 = r.p+r.v*3;
    float f[20];

    for (int i = 0; i <= N;i++){

        float ch = 0.5+ 0.5*cos((2 * (float)(N-i) +1) / ((float)(N+1) * 2) * PI);

        f[i] = map(t0 +(t1-t0)*ch);
    }

    float b[20];


    for (int i = 0; i <= N; i++){
        b[i] = 0;

        for (int j = 0; j <= N; j++){
            b[i] += (f[j]*x0[i*(N+1)+j]);

         }
    }


    float4 t = rayMarch(input.pixelpos,t0,t1,b);

    if (t.w == 0)
        return float4(1,1,1,1);

    return float4(t.xyz,1);
}
