#include "bezier_eval.slang"


struct Ray{
    float3 v;
    float3 p;
}

float maxx(float3 v)
{
    return max(max(v.x, v.y), v.z);
}

struct Box
{
    float3 center;
    float3 radius;

};
// based on http://www.jcgt.org/published/0007/03/04/paper-lowres.pdf
bool intersectBox(Box box, Ray ray, bool front)
{
    ray.p = ray.p - box.center;
    float winding = front ? 1 : -1;
    float3 sgn = -sign(ray.v);
    float3 d = box.radius * winding * sgn - ray.p;
    d /= ray.v;
#define TEST_BOX(U, VW) (d.U >= 0.0) && all(abs(ray.p.VW + ray.v.VW * d.U)< box.radius.VW)
    bool3 test = bool3(TEST_BOX(x, yz), TEST_BOX(y, zx), TEST_BOX(z, xy));
#undef TEST_BOX
    sgn = test.x ? float3(sgn.x, 0, 0) : (test.y ? float3(0, sgn.y, 0) : float3(0, 0, test.z ? sgn.z : 0));
    float dist = (sgn.x != 0) ? d.x : ((sgn.y != 0) ? d.y : d.z);
    return (sgn.x != 0) || (sgn.y != 0) || (sgn.z != 0);
}



float4 rayMarch(float3 pixelpos, float3 t0,float3 t1, type b[20])
{
    Ray r;
   
        r.v = normalize(t1-t0);
        r.p = pixelpos;

       


    #if BEZ == 2 || BEZ == 5
          for (int i = 0; i <= N; i++)
             {
             nChoosek[i] = 1;
            }

         for (int i = 1; i <= N / 2; i++)
         {
             nChoosek[i] = nChoosek[i - 1] * (N - i + 1) / i;
             nChoosek[N - i] = nChoosek[i];
         }
    #endif

    
    type t = 0.0f;
    int i;
    float h;
    
    int n = 200;
    type step =1/(type)n ;
    float3 pos = r.p + t * r.v;
   
       
    for (i = 0; i <=n; i++)
    {
        pos = r.p + (float)t * length(t1-t0)* r.v;

        //h = f_p_casteljau_2d(b,t);
        h = bezier_eval(b,t);
       //h = map(pos);

        if (h < 0.01)
            break;

        t += step;
     
    }

    float3 hitpos = pos;
    float hit = 1;
    if (i >= n)
        hit = 0;
    return float4(hitpos, hit);
}


