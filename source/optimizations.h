#ifndef OPTIMIZATIONS_H
#define OPTIMIZATIONS_H
//#include "optimizations.h"

// if x < min return ( min );
// if x > max return ( max );
// return ( x );

// NOTA: questa � meglio del C

#ifdef __arm__
__inline int clip2(int x, const int min, const int max)
{
  asm(
    "CMP %0, %1 \n"
    "ITEE LT \n"
    "MOVLT %0, %1 \n"
    "CMPGE %0, %2 \n"
    "MOVGE %0, %2"
    : "+r"(x) : "r"(min), "r"(max) : "cc"
    );
  return(x);
}

// NOTA: questa � meglio del C in quanto il C non � cos� furbo da fare ITT GE
__inline int clip2_m0(int x, const int max)
{
  asm(
    "BICS %0, %0, %0, ASR #31 \n"
    "ITT GE \n"
    "CMPGE %0, %1 \n"
    "MOVGE %0, %1 \n"
    : "+r"(x) : "r"(max) : "cc"
    );
  return(x);
}
#else
__inline int clip2(int x, const int min, const int max)
{
  return( x < min ? min : (x > max ? max : x) );
}
__inline int clip2_m0(int x, const int max)
{
  return( x < 0 ? 0 : (x > max ? max : x) );
}
#endif

#endif