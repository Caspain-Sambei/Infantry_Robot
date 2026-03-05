#ifndef __FIR_H__
#define __FIR_H__

#define K_Dynamic_Calc              1
#define K_ChassisFellowPosition      2
#define K_ChassisFellowSpeed         3

float LowPass(float Sample_Pre,float OutPut_Last,int Mode);

#endif
