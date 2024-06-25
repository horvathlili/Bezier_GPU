
#ifndef N
#define N 0
#endif

#ifndef type
#define type float
#endif

#ifndef BEZ
#define BEZ 0
#endif


#ifndef NK
#define NK 0
#endif

 #if NK == 0
    RWStructuredBuffer<float> nChoosek;
    static int offset = 0;
#endif

    #if NK == 1
 static const int nChoosek[] = {
        1, 1,
        1, 2, 1,
        1, 3, 3, 1,
        1, 4, 6, 4, 1,
        1, 5, 10, 10, 5, 1,
        1, 6, 15, 20, 15, 6, 1,
        1, 7, 21, 35, 35, 21, 7, 1,
        1, 8, 28, 56, 70, 56, 28, 8, 1,
        1, 9, 36, 84, 126, 126, 84, 36, 9, 1,
        1, 10, 45, 120, 210, 252, 210, 120, 45, 10, 1,
        1, 11, 55, 165, 330, 462, 462, 330, 165, 55, 11, 1,
        1, 12, 66, 220, 495, 792, 924, 792, 495, 220, 66, 12, 1,
        1, 13, 78, 286, 715, 1287, 1716, 1716, 1287, 715, 286, 78, 13, 1,
        1, 14, 91, 364, 1001, 2002, 3003, 3432, 3003, 2002, 1001, 364, 91, 14, 1,
        1, 15, 105, 455, 1365, 3003, 5005, 6435, 6435, 5005, 3003, 1365, 455, 105, 15, 1,
        1, 16, 120, 560, 1820, 4368, 8008, 11440, 12870, 11440, 8008, 4368, 1820, 560, 120, 16, 1,
        1, 17, 136, 680, 2380, 6188, 12376, 19448, 24310, 24310, 19448, 12376, 6188, 2380, 680, 136, 17, 1,
        1, 18, 153, 816, 3060, 8568, 18564, 31824, 43758, 48620, 43758, 31824, 18564, 8568, 3060, 816, 153, 18, 1,
        1, 19, 171, 969, 3876, 11628, 27132, 50388, 75582, 92378, 92378, 75582, 50388, 27132, 11628, 3876, 969, 171, 19, 1,
        1, 20, 190, 1140, 4845, 15504, 38760, 77520, 125970, 167960, 184756, 167960, 125970, 77520, 38760, 15504, 4845, 1140, 190, 20, 1,
    };
    static const int offsets[] = { 0, 2, 5, 9, 14, 20, 27, 35, 44, 54, 65, 77, 90, 104, 119, 135, 152, 170, 189, 209, 230, };
    static const int offset = offsets[N-1];

#endif


type bezier_eval(type w_y[20],type t){



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
        p_y += (nChoosek[offset + i] * t1[i] * t2[i] * w_y[i]);

    }

   return p_y;

}

float f_p_3(float w_y[20], float t) {

    float p_x = 0;
    float p_y = 0;


    float t1 = 1;

    for (int i = 0; i <= N; i++) {

        //p_x = p_x * (1 - t) + nChoosek[i] * t1 * w_x[i];
        p_y = p_y * (1 - t) + nChoosek[offset + i] * t1 * w_y[i];

        t1 *= t;

    }

   

    return p_y;
}



type f_p_estrin_2d(type w_y[20], type t)
{
    
    const int pairCount = (N + 1) / 2;
    type t2 = 1;
    type mt = (1 - t)*(1-t);
    
    type p_x = 0;
    type p_y = 0;
  
    for (int i = 0; i < pairCount; ++i)
    {
        //p_x = p_x * mt + (w_x[2 * i] * nChoosek[2 * i] * (1 - t) + w_x[2 * i+1] * nChoosek[2 * i+1] * t) * t2;
        p_y = p_y * mt + (w_y[2 * i] * (type)nChoosek[offset +  2 * i] * (1 - t) + w_y[2 * i + 1] * (type)nChoosek[offset +  2 * i + 1] * t) * t2;

        t2 *= (t * t);
    }
   // q_x[0] += (n % 2) ? p_x : p_x * (1-t) + w_x[n] *t2;
    p_y = (N % 2) ? p_y : p_y * (1 - t) + w_y[N] * t2;

    return p_y;
}
