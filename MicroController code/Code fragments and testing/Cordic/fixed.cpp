#include "fixed.h"
#include "fixed_func.h"



fixed operator+(const fixed &f1, const fixed &f2){fixed temp; temp.intValue = f1.intValue + f2.intValue; return temp;}
fixed operator+(const fixed &f,  const int   &i ){fixed temp; temp.intValue = f.intValue  + (i<<16);     return temp;}
fixed operator+(const int   &i,  const fixed &f ){fixed temp; temp.intValue = f.intValue  + (i<<16);     return temp;}
 	

fixed operator-(const fixed &f1, const fixed &f2){fixed temp; temp.intValue = f1.intValue - f2.intValue; return temp;}
fixed operator-(const fixed &f,  const int   &i ){fixed temp; temp.intValue = f.intValue  - (i<<16);     return temp;}
fixed operator-(const int   &i,  const fixed &f ){fixed temp; temp.intValue = (i<<16)     - f.intValue;  return temp;}

                                                  
fixed operator*(const fixed &f1, const fixed &f2){fixed temp; temp.intValue = fixmul(f1.intValue, f2.intValue); return temp;}
fixed operator*(const fixed &f,  const int   &i ){fixed temp; temp.intValue = f.intValue * i;                   return temp;}
fixed operator*(const int   &i,  const fixed &f ){fixed temp; temp.intValue = f.intValue * i;                   return temp;}

fixed operator/(const fixed &f1, const fixed &f2){fixed temp; temp.intValue = fixdiv(f1.intValue, f2.intValue); return temp;}
fixed operator/(const fixed &f,  const int   &i ){fixed temp; temp.intValue = f.intValue/i;              return temp;}
fixed operator/(const int   &i,  const fixed &f ){fixed temp; temp.intValue = fixdiv(i<<16, f.intValue); return temp; }

fixed operator-(const fixed &f){return 0 - f;}


bool operator== (const fixed &f1, const fixed &f2){return f1.intValue == f2.intValue;}
bool operator== (const fixed &f,  const int   &i ){return f.intValue  == (i<<16);    }
bool operator== (const int   &i,  const fixed &f ){return (i<<16)     == f.intValue; }

bool operator!= (const fixed &f1, const fixed &f2){return f1.intValue != f2.intValue;}
bool operator!= (const fixed &f,  const int   &i ){return f.intValue  != (i<<16);    }
bool operator!= (const int   &i,  const fixed &f ){return i           != f.intValue; }

bool operator<  (const fixed &f1, const fixed &f2){return f1.intValue <  f2.intValue;}
bool operator<  (const fixed &f,  const int   &i ){return f.intValue  <  (i<<16);    }
bool operator<  (const int   &i,  const fixed &f ){return (i<<16)     <  f.intValue; }

bool operator>  (const fixed &f1, const fixed &f2){return f1.intValue >  f2.intValue;}
bool operator>  (const fixed &f,  const int   &i ){return f.intValue  >  (i<<16);    }
bool operator>  (const int   &i,  const fixed &f ){return (i<<16)     >  f.intValue; }

bool operator<= (const fixed &f1, const fixed &f2){return f1.intValue <= f2.intValue;}
bool operator<= (const fixed &f,  const int   &i ){return f.intValue  <= (i<<16);    }
bool operator<= (const int   &i,  const fixed &f ){return (i<<16)     <= f.intValue; }

bool operator>= (const fixed &f1, const fixed &f2){return f1.intValue >= f2.intValue;}
bool operator>= (const fixed &f,  const int   &i ){return f.intValue  >= (i<<16);    }
bool operator>= (const int   &i,  const fixed &f ){return (i<<16)     >= f.intValue; }





