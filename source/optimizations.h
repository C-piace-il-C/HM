#ifndef OPTIMIZATIONS_H
#define OPTIMIZATIONS_H
//#include "optimizations.h"

// if x < min return ( min );
// if x > max return ( max );
// return ( x );

// NOTA: questa è meglio del C

#ifdef __arm__
__inline int abs_i(int x)
{
  asm (
    "ADD %0, %0, %0, ASR #31 \n"
	"EOR %0, %0, %0, ASR #31 \n"
	: "+r"(x) : : "r0"
  );
  return(x);
}

// QUESTA LA VOGLIO OTTIMIZZARE
__inline short abs_s(short x)
{
  asm (
    "MOV %0, %0, LSL #16 \n"
    "ADD %0, %0, %0, ASR #31 \n"
	"EOR %0, %0, %0, ASR #31 \n"
	"MOV %0, %0, ASR #16 \n"
	: "+r"(x)
  );
  return(x);
}

__inline int clip2(int x, const int min, const int max)
{
  asm (
    "CMP %0, %1 \n"
    "ITEE LT \n"
    "MOVLT %0, %1 \n"
    "CMPGE %0, %2 \n"
    "MOVGE %0, %2"
    : "+r"(x) : "r"(min), "r"(max) : "cc"
    );
  return(x);
}

__inline int clip2_m0(int x, const int max)
{
  asm (
    "BICS %0, %0, %0, ASR #31 \n"
    "ITT PL \n"
    "CMPPL %0, %1 \n"
    "MOVPL %0, %1 \n"
    : "+r"(x) : "r"(max) : "cc"
  );
  return(x);
}

__inline short clip2_m0s(short x, const short max)
{
  asm (
    "MOV R0, %0, LSL #16 \n"
    "BICS %0, %0, R0, ASR #31 \n"
    "ITT PL \n"
    "CMPPL %0, %1 \n"
    "MOVPL %0, %1 \n"
    : "+r"(x) : "r"(max) : "cc","r0"
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