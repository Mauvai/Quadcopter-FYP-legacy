#include "fixed_func.h"

#ifndef _FIXED_H
#define _FIXED_H



typedef long long int int64; 
typedef int int32; //int 32 is default into size for this system
typedef unsigned int uint32;



class fixed
{
    public:
    fixed() : intValue(0) {}
    fixed(int32 i)  : intValue(i << 16) {}
    fixed(float f)  : intValue(float2fix(f)) {}
    fixed(double f) : intValue(float2fix((float)f)) {}
    fixed(const fixed& f) : intValue(f.intValue)  {}
	  
   
    void set(int i)  //this is baaaad.... but its needed to directly set intValue... 
                     // WHICH YOU SHOULD NOT DO unless you MUST and know what it means
    {
        intValue = i;
    }        
    

    
    inline float f()
    {
        return (float)intValue / (1 << 16);
    }  
    inline int i()
    {
        return intValue;
    } 
   
    
    
    
    float fix2float(int32 f)
    {
		return (float)f / (1 << 16);
    }

    int32 float2fix(float f)
    {
	    return (int32)(f * (1 << 16));
    }
    
    fixed& operator= (const fixed &f) {intValue = f.intValue; return *this;}

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

    friend fixed operator- (const fixed &f);  //unary minus overload


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


    private:
    int intValue; 
};






    
#endif