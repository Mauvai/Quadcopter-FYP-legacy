typedef long long int int64; 
typedef int int32; //int 32 is default into size for this system
typedef unsigned int uint32;

int32 fixmulf(int32 a, int32 b)
{
    return (a * b) >> 16;
}





// Perform a fixed point multiplication using a 64-bit intermediate result to
// prevent overflow problems.
int32 fixmul(int32 a, int32 b)
{
        return (int32)(((int64)a * b) >> 16);
}





int32 fixdiv(int32 a, int32 b)
{
#if 0
    return (int32)((((int64)a) << 16) / b);
#else
   // The following produces the same results as the above but 
   // generates fewer instructions 
   union 
   {
        int64 a;
        struct 
        {
            int32 l;
            int32 h;
        };
    } x;
    x.l = a << 16;
    x.h = a >> (sizeof(int32) * 8 - 16);
    return (int32)(x.a / b);
#endif
}




