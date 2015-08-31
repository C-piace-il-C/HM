#ifndef _OPT_2
#define _OPT_2

short __attribute__((always_inline)) clip2_m0s(short x, const short max)
{
  asm(
	"MOV   R0, %0, LSL #16     \n"  // R0 <- (x << 16)
	"BICS  %0, %0, R0, ASR #31 \n"  // if( x < 0 ) x = 0;
	"ITT   PL                  \n"  // else
	"CMPPL %0, %1              \n"  // if( x > max ) 
	"MOVPL %0, %1              \n"  //   x = max;
	: "+r"(x) : "r"(max) : "cc","r0"
  );
  return(x);
}

#endif //_OPT_2