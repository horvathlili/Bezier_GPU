
#ifndef N
#define N 0
#endif

#ifndef BEZ
#define BEZ 0
#endif


float bezier_eval(float w_y[20],float t){

#if BEZ == 0
    return f_p_casteljau_2d(w_y,t);
#endif
#if BEZ == 1
    return f_p_new_2d(w_y,t);
#endif
#if BEZ == 2
    return f_p_3(w_y,t);
#endif
#if BEZ == 3
    return f_p_3(w_y,t);
#endif
#if BEZ == 4
    return f_p_estrin_2d(w_y,t);
#endif
#if BEZ == 5
    return f_p_estrin_2d(w_y,t);
#endif

}


float f_p_casteljau_2d(float q_y[20],float t)
{
  
    float t1 = 1.0 - t;
 
    for (int k = 1; k <= N; k++)
    {
        for (int i = 0; i <= N - k; i++)
        {
            q_y[i] = t1 * q_y[i] + t * q_y[i + 1];
        }
    }
    return q_y[0];
}

float f_p_new_2d(float w_y[20], float t)
{
    float h = 1.0;
    float u = 1.0 - t;
    int n1 = N + 1;
    
    float q = w_y[0];
    if (t <= 0.5)
    {
        u = t / u;
        for (int k = 1; k <= N; k++)
        {
            h = h * u * (n1 - k);
            h = h / (k + h);
            float h1 = 1 - h;
            
            q = h1 * q + h * w_y[k];
        }
    }
    else
    {
        u = u / t;
        for (int k = 1; k <= N; k++)
        {
            h = h * (n1 - k);
            h = h / (k * u + h);
            float h1 = 1 - h;
            
            q = h1 * q + h * w_y[k];
        }
    }
    return q;
}


float f_p_2(float w_y[20], float t) {

    float p_x = 0;
    float p_y = 0;

    float t1[N+1];
    float t2[N+1];

    t1[0] = 1.0f;
    t2[N] = 1.0f;

    for (int i = 1; i <= N; i++) {
        t1[i] = t1[i - 1] * t;
        t2[N - i] = t2[N - i + 1] * (1.0 - t);
    }

    for (int i = 0; i <= N; i++) {


        //p_x += (nChoosek[i] * t1[i] * t2[i] * w_x[i]);
        p_y += (nChoosek[i] * t1[i] * t2[i] * w_y[i]);

    }

   return p_y;

}

float f_p_3(float w_y[20], float t) {

    float p_x = 0;
    float p_y = 0;


    float t1 = 1;

    for (int i = 0; i <= N; i++) {

        //p_x = p_x * (1 - t) + nChoosek[i] * t1 * w_x[i];
        p_y = p_y * (1 - t) + nChoosek[i] * t1 * w_y[i];

        t1 *= t;

    }

   

    return p_y;
}



float f_p_estrin_2d(float w_y[20], float t)
{
    
    const int pairCount = (N + 1) / 2;
    float t2 = 1;
    float mt = (1 - t)*(1-t);
    
    float p_x = 0;
    float p_y = 0;
  
    for (int i = 0; i < pairCount; ++i)
    {
        //p_x = p_x * mt + (w_x[2 * i] * nChoosek[2 * i] * (1 - t) + w_x[2 * i+1] * nChoosek[2 * i+1] * t) * t2;
        p_y = p_y * mt + (w_y[2 * i] * nChoosek[2 * i] * (1 - t) + w_y[2 * i + 1] * nChoosek[2 * i + 1] * t) * t2;

        t2 *= (t * t);
    }
   // q_x[0] += (n % 2) ? p_x : p_x * (1-t) + w_x[n] *t2;
    p_y = (N % 2) ? p_y : p_y * (1 - t) + w_y[N] * t2;

    return p_y;
}
