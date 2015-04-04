#ifndef _FIXED_H
#define _FIXED_H


typedef long long int int64; 
typedef int int32; //int 32 is default into size for this system


inline int32 fixmulf(int32 a, int32 b);
inline int32 fixmul( int32 a, int32 b);
inline int32 fixdiv( int32 a, int32 b);


class fixed
{
    public:
    fixed() {}
    fixed(int32 i) : intValue(i << 16) {}
    fixed(float f) : intValue(float2fix(f)) {}
    fixed(double f) : intValue(float2fix((float)f)) {}
	  
	  
    friend inline int32 fixmulf(int32 a, int32 b);
    friend inline int32 fixmul( int32 a, int32 b);
    friend inline int32 fixdiv( int32 a, int32 b);

	

    inline float f()
    {
        return (float)intValue / (1 << 16);
    }  
    inline int i()
    {
        return intValue;
    } 
   
    
    int intValue; 
    float fix2float(int32 f)
    {
		return (float)f / (1 << 16);
    }

    int32 float2fix(float f)
    {
	    return (int32)(f * (1 << 16));
    }


    fixed operator += (fixed f) { intValue += f.intValue; return *this; }
    fixed operator -= (fixed f) { intValue -= f.intValue; return *this; }
    fixed operator *= (fixed f) { intValue = fixmul(intValue, f.intValue); return *this; }
    fixed operator /= (fixed f) { intValue = fixdiv(intValue, f.intValue); return *this; }
		
    fixed operator == (int32 i) { intValue += (i<<16); return *this; }
    fixed operator -= (int32 i) { intValue -= (i<<16); return *this; }
    fixed operator *= (int32 i) { intValue *= i;       return *this; }
    fixed operator /= (int32 i) { intValue /= i;       return *this; }
	
    /*
    fixed operator - () const { fixed x; x.intValue = -intValue; return x; }
    fixed operator + (fixed r) const { fixed x = *this; x += r; return x;}
    fixed operator - (fixed r) const { fixed x = *this; x -= r; return x;}
    fixed operator * (fixed r) const { fixed x = *this; x *= r; return x;}
    fixed operator / (fixed r) const { fixed x = *this; x /= r; return x;}
    
    bool operator == (fixed i) const { return intValue == i.intValue; }
    bool operator != (fixed i) const { return  !(*this == i);         }
    bool operator <  (fixed i) const { return intValue <  i.intValue; }
    bool operator >  (fixed i) const { return intValue >  i.intValue; }
    bool operator <= (fixed i) const { return intValue <= i.intValue; }
    bool operator >= (fixed i) const { return intValue >= i.intValue; }
    
    fixed operator + (int32 r) const { fixed x = *this; x += r; return x;}
    fixed operator - (int32 r) const { fixed x = *this; x -= r; return x;}
    fixed operator * (int32 r) const { fixed x = *this; x *= r; return x;}
    fixed operator / (int32 r) const { fixed x = *this; x /= r; return x;}
    */
    friend bool operator== (const fixed &f1, const fixed &f2);
    friend bool operator== (const fixed &f,  const int   &i );
    friend bool operator== (const int   &i,  const fixed &f );
    
    friend bool operator!= (const fixed &f1, const fixed &f2);
    friend bool operator!= (const fixed &f,  const int   &i );
    friend bool operator!= (const int   &i,  const fixed &f );

    friend bool operator<  (const fixed &f1, const fixed &f2);
    friend bool operator<  (const fixed &f,  const int   &i );
    friend bool operator<  (const int   &i,  const fixed &f );

    friend bool operator>  (const fixed &f1, const fixed &f2);
    friend bool operator>  (const fixed &f,  const int   &i );
    friend bool operator>  (const int   &i,  const fixed &f );

    friend bool operator<= (const fixed &f1, const fixed &f2);
    friend bool operator<= (const fixed &f,  const int   &i );
    friend bool operator<= (const int   &i,  const fixed &f );
    
    friend bool operator>= (const fixed &f1, const fixed &f2);
    friend bool operator>= (const fixed &f,  const int   &i );
    friend bool operator>= (const int   &i,  const fixed &f );

    friend fixed operator-(const fixed &f);


    friend fixed operator+ (const fixed &f1, const fixed &f2);
    friend fixed operator+ (const fixed &f,  const int   &i );
    friend fixed operator+ (const int   &i,  const fixed &f );
    
    friend fixed operator- (const fixed &f1, const fixed &f2);
    friend fixed operator- (const fixed &f,  const int   &i );
    friend fixed operator- (const int   &i,  const fixed &f );
    
    friend fixed operator* (const fixed &f1, const fixed &f2);
    friend fixed operator* (const fixed &f,  const int   &i );
    friend fixed operator* (const int   &i,  const fixed &f );

    friend fixed operator/ (const fixed &f1, const fixed &f2);
    friend fixed operator/ (const fixed &f,  const int   &i );
    friend fixed operator/ (const int   &i,  const fixed &f );

};





inline int32 fixmulf(int32 a, int32 b)
    {
    return (a * b) >> 16;
    }

    // Perform a fixed point multiplication using a 64-bit intermediate result to
    // prevent overflow problems.
    inline int32 fixmul(int32 a, int32 b)
    {
        return (int32)(((int64)a * b) >> 16);
    }


    inline int32 fixdiv(int32 a, int32 b)
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
    
#endif