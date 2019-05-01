/* Produced by CVXGEN, 2018-03-06 05:22:48 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.Aeq[0])-rhs[1]*(params.Aeq[6])-rhs[2]*(params.Aeq[12])-rhs[3]*(params.Aeq[18])-rhs[4]*(params.Aeq[24])-rhs[5]*(params.Aeq[30])-rhs[6]*(params.Aeq[36])-rhs[7]*(params.Aeq[42])-rhs[8]*(params.Aeq[48])-rhs[9]*(params.Aeq[54])-rhs[10]*(params.Aeq[60])-rhs[11]*(params.Aeq[66])-rhs[12]*(params.Aeq[72]);
  lhs[1] = -rhs[0]*(params.Aeq[1])-rhs[1]*(params.Aeq[7])-rhs[2]*(params.Aeq[13])-rhs[3]*(params.Aeq[19])-rhs[4]*(params.Aeq[25])-rhs[5]*(params.Aeq[31])-rhs[6]*(params.Aeq[37])-rhs[7]*(params.Aeq[43])-rhs[8]*(params.Aeq[49])-rhs[9]*(params.Aeq[55])-rhs[10]*(params.Aeq[61])-rhs[11]*(params.Aeq[67])-rhs[12]*(params.Aeq[73]);
  lhs[2] = -rhs[0]*(params.Aeq[2])-rhs[1]*(params.Aeq[8])-rhs[2]*(params.Aeq[14])-rhs[3]*(params.Aeq[20])-rhs[4]*(params.Aeq[26])-rhs[5]*(params.Aeq[32])-rhs[6]*(params.Aeq[38])-rhs[7]*(params.Aeq[44])-rhs[8]*(params.Aeq[50])-rhs[9]*(params.Aeq[56])-rhs[10]*(params.Aeq[62])-rhs[11]*(params.Aeq[68])-rhs[12]*(params.Aeq[74]);
  lhs[3] = -rhs[0]*(params.Aeq[3])-rhs[1]*(params.Aeq[9])-rhs[2]*(params.Aeq[15])-rhs[3]*(params.Aeq[21])-rhs[4]*(params.Aeq[27])-rhs[5]*(params.Aeq[33])-rhs[6]*(params.Aeq[39])-rhs[7]*(params.Aeq[45])-rhs[8]*(params.Aeq[51])-rhs[9]*(params.Aeq[57])-rhs[10]*(params.Aeq[63])-rhs[11]*(params.Aeq[69])-rhs[12]*(params.Aeq[75]);
  lhs[4] = -rhs[0]*(params.Aeq[4])-rhs[1]*(params.Aeq[10])-rhs[2]*(params.Aeq[16])-rhs[3]*(params.Aeq[22])-rhs[4]*(params.Aeq[28])-rhs[5]*(params.Aeq[34])-rhs[6]*(params.Aeq[40])-rhs[7]*(params.Aeq[46])-rhs[8]*(params.Aeq[52])-rhs[9]*(params.Aeq[58])-rhs[10]*(params.Aeq[64])-rhs[11]*(params.Aeq[70])-rhs[12]*(params.Aeq[76]);
  lhs[5] = -rhs[0]*(params.Aeq[5])-rhs[1]*(params.Aeq[11])-rhs[2]*(params.Aeq[17])-rhs[3]*(params.Aeq[23])-rhs[4]*(params.Aeq[29])-rhs[5]*(params.Aeq[35])-rhs[6]*(params.Aeq[41])-rhs[7]*(params.Aeq[47])-rhs[8]*(params.Aeq[53])-rhs[9]*(params.Aeq[59])-rhs[10]*(params.Aeq[65])-rhs[11]*(params.Aeq[71])-rhs[12]*(params.Aeq[77]);
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.Aeq[0])-rhs[1]*(params.Aeq[1])-rhs[2]*(params.Aeq[2])-rhs[3]*(params.Aeq[3])-rhs[4]*(params.Aeq[4])-rhs[5]*(params.Aeq[5]);
  lhs[1] = -rhs[0]*(params.Aeq[6])-rhs[1]*(params.Aeq[7])-rhs[2]*(params.Aeq[8])-rhs[3]*(params.Aeq[9])-rhs[4]*(params.Aeq[10])-rhs[5]*(params.Aeq[11]);
  lhs[2] = -rhs[0]*(params.Aeq[12])-rhs[1]*(params.Aeq[13])-rhs[2]*(params.Aeq[14])-rhs[3]*(params.Aeq[15])-rhs[4]*(params.Aeq[16])-rhs[5]*(params.Aeq[17]);
  lhs[3] = -rhs[0]*(params.Aeq[18])-rhs[1]*(params.Aeq[19])-rhs[2]*(params.Aeq[20])-rhs[3]*(params.Aeq[21])-rhs[4]*(params.Aeq[22])-rhs[5]*(params.Aeq[23]);
  lhs[4] = -rhs[0]*(params.Aeq[24])-rhs[1]*(params.Aeq[25])-rhs[2]*(params.Aeq[26])-rhs[3]*(params.Aeq[27])-rhs[4]*(params.Aeq[28])-rhs[5]*(params.Aeq[29]);
  lhs[5] = -rhs[0]*(params.Aeq[30])-rhs[1]*(params.Aeq[31])-rhs[2]*(params.Aeq[32])-rhs[3]*(params.Aeq[33])-rhs[4]*(params.Aeq[34])-rhs[5]*(params.Aeq[35]);
  lhs[6] = -rhs[0]*(params.Aeq[36])-rhs[1]*(params.Aeq[37])-rhs[2]*(params.Aeq[38])-rhs[3]*(params.Aeq[39])-rhs[4]*(params.Aeq[40])-rhs[5]*(params.Aeq[41]);
  lhs[7] = -rhs[0]*(params.Aeq[42])-rhs[1]*(params.Aeq[43])-rhs[2]*(params.Aeq[44])-rhs[3]*(params.Aeq[45])-rhs[4]*(params.Aeq[46])-rhs[5]*(params.Aeq[47]);
  lhs[8] = -rhs[0]*(params.Aeq[48])-rhs[1]*(params.Aeq[49])-rhs[2]*(params.Aeq[50])-rhs[3]*(params.Aeq[51])-rhs[4]*(params.Aeq[52])-rhs[5]*(params.Aeq[53]);
  lhs[9] = -rhs[0]*(params.Aeq[54])-rhs[1]*(params.Aeq[55])-rhs[2]*(params.Aeq[56])-rhs[3]*(params.Aeq[57])-rhs[4]*(params.Aeq[58])-rhs[5]*(params.Aeq[59]);
  lhs[10] = -rhs[0]*(params.Aeq[60])-rhs[1]*(params.Aeq[61])-rhs[2]*(params.Aeq[62])-rhs[3]*(params.Aeq[63])-rhs[4]*(params.Aeq[64])-rhs[5]*(params.Aeq[65]);
  lhs[11] = -rhs[0]*(params.Aeq[66])-rhs[1]*(params.Aeq[67])-rhs[2]*(params.Aeq[68])-rhs[3]*(params.Aeq[69])-rhs[4]*(params.Aeq[70])-rhs[5]*(params.Aeq[71]);
  lhs[12] = -rhs[0]*(params.Aeq[72])-rhs[1]*(params.Aeq[73])-rhs[2]*(params.Aeq[74])-rhs[3]*(params.Aeq[75])-rhs[4]*(params.Aeq[76])-rhs[5]*(params.Aeq[77]);
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.Aineq[0])-rhs[1]*(params.Aineq[14])-rhs[2]*(params.Aineq[28])-rhs[3]*(params.Aineq[42])-rhs[4]*(params.Aineq[56])-rhs[5]*(params.Aineq[70])-rhs[6]*(params.Aineq[84])-rhs[7]*(params.Aineq[98])-rhs[8]*(params.Aineq[112])-rhs[9]*(params.Aineq[126])-rhs[10]*(params.Aineq[140])-rhs[11]*(params.Aineq[154])-rhs[12]*(params.Aineq[168]);
  lhs[1] = -rhs[0]*(params.Aineq[1])-rhs[1]*(params.Aineq[15])-rhs[2]*(params.Aineq[29])-rhs[3]*(params.Aineq[43])-rhs[4]*(params.Aineq[57])-rhs[5]*(params.Aineq[71])-rhs[6]*(params.Aineq[85])-rhs[7]*(params.Aineq[99])-rhs[8]*(params.Aineq[113])-rhs[9]*(params.Aineq[127])-rhs[10]*(params.Aineq[141])-rhs[11]*(params.Aineq[155])-rhs[12]*(params.Aineq[169]);
  lhs[2] = -rhs[0]*(params.Aineq[2])-rhs[1]*(params.Aineq[16])-rhs[2]*(params.Aineq[30])-rhs[3]*(params.Aineq[44])-rhs[4]*(params.Aineq[58])-rhs[5]*(params.Aineq[72])-rhs[6]*(params.Aineq[86])-rhs[7]*(params.Aineq[100])-rhs[8]*(params.Aineq[114])-rhs[9]*(params.Aineq[128])-rhs[10]*(params.Aineq[142])-rhs[11]*(params.Aineq[156])-rhs[12]*(params.Aineq[170]);
  lhs[3] = -rhs[0]*(params.Aineq[3])-rhs[1]*(params.Aineq[17])-rhs[2]*(params.Aineq[31])-rhs[3]*(params.Aineq[45])-rhs[4]*(params.Aineq[59])-rhs[5]*(params.Aineq[73])-rhs[6]*(params.Aineq[87])-rhs[7]*(params.Aineq[101])-rhs[8]*(params.Aineq[115])-rhs[9]*(params.Aineq[129])-rhs[10]*(params.Aineq[143])-rhs[11]*(params.Aineq[157])-rhs[12]*(params.Aineq[171]);
  lhs[4] = -rhs[0]*(params.Aineq[4])-rhs[1]*(params.Aineq[18])-rhs[2]*(params.Aineq[32])-rhs[3]*(params.Aineq[46])-rhs[4]*(params.Aineq[60])-rhs[5]*(params.Aineq[74])-rhs[6]*(params.Aineq[88])-rhs[7]*(params.Aineq[102])-rhs[8]*(params.Aineq[116])-rhs[9]*(params.Aineq[130])-rhs[10]*(params.Aineq[144])-rhs[11]*(params.Aineq[158])-rhs[12]*(params.Aineq[172]);
  lhs[5] = -rhs[0]*(params.Aineq[5])-rhs[1]*(params.Aineq[19])-rhs[2]*(params.Aineq[33])-rhs[3]*(params.Aineq[47])-rhs[4]*(params.Aineq[61])-rhs[5]*(params.Aineq[75])-rhs[6]*(params.Aineq[89])-rhs[7]*(params.Aineq[103])-rhs[8]*(params.Aineq[117])-rhs[9]*(params.Aineq[131])-rhs[10]*(params.Aineq[145])-rhs[11]*(params.Aineq[159])-rhs[12]*(params.Aineq[173]);
  lhs[6] = -rhs[0]*(params.Aineq[6])-rhs[1]*(params.Aineq[20])-rhs[2]*(params.Aineq[34])-rhs[3]*(params.Aineq[48])-rhs[4]*(params.Aineq[62])-rhs[5]*(params.Aineq[76])-rhs[6]*(params.Aineq[90])-rhs[7]*(params.Aineq[104])-rhs[8]*(params.Aineq[118])-rhs[9]*(params.Aineq[132])-rhs[10]*(params.Aineq[146])-rhs[11]*(params.Aineq[160])-rhs[12]*(params.Aineq[174]);
  lhs[7] = -rhs[0]*(params.Aineq[7])-rhs[1]*(params.Aineq[21])-rhs[2]*(params.Aineq[35])-rhs[3]*(params.Aineq[49])-rhs[4]*(params.Aineq[63])-rhs[5]*(params.Aineq[77])-rhs[6]*(params.Aineq[91])-rhs[7]*(params.Aineq[105])-rhs[8]*(params.Aineq[119])-rhs[9]*(params.Aineq[133])-rhs[10]*(params.Aineq[147])-rhs[11]*(params.Aineq[161])-rhs[12]*(params.Aineq[175]);
  lhs[8] = -rhs[0]*(params.Aineq[8])-rhs[1]*(params.Aineq[22])-rhs[2]*(params.Aineq[36])-rhs[3]*(params.Aineq[50])-rhs[4]*(params.Aineq[64])-rhs[5]*(params.Aineq[78])-rhs[6]*(params.Aineq[92])-rhs[7]*(params.Aineq[106])-rhs[8]*(params.Aineq[120])-rhs[9]*(params.Aineq[134])-rhs[10]*(params.Aineq[148])-rhs[11]*(params.Aineq[162])-rhs[12]*(params.Aineq[176]);
  lhs[9] = -rhs[0]*(params.Aineq[9])-rhs[1]*(params.Aineq[23])-rhs[2]*(params.Aineq[37])-rhs[3]*(params.Aineq[51])-rhs[4]*(params.Aineq[65])-rhs[5]*(params.Aineq[79])-rhs[6]*(params.Aineq[93])-rhs[7]*(params.Aineq[107])-rhs[8]*(params.Aineq[121])-rhs[9]*(params.Aineq[135])-rhs[10]*(params.Aineq[149])-rhs[11]*(params.Aineq[163])-rhs[12]*(params.Aineq[177]);
  lhs[10] = -rhs[0]*(params.Aineq[10])-rhs[1]*(params.Aineq[24])-rhs[2]*(params.Aineq[38])-rhs[3]*(params.Aineq[52])-rhs[4]*(params.Aineq[66])-rhs[5]*(params.Aineq[80])-rhs[6]*(params.Aineq[94])-rhs[7]*(params.Aineq[108])-rhs[8]*(params.Aineq[122])-rhs[9]*(params.Aineq[136])-rhs[10]*(params.Aineq[150])-rhs[11]*(params.Aineq[164])-rhs[12]*(params.Aineq[178]);
  lhs[11] = -rhs[0]*(params.Aineq[11])-rhs[1]*(params.Aineq[25])-rhs[2]*(params.Aineq[39])-rhs[3]*(params.Aineq[53])-rhs[4]*(params.Aineq[67])-rhs[5]*(params.Aineq[81])-rhs[6]*(params.Aineq[95])-rhs[7]*(params.Aineq[109])-rhs[8]*(params.Aineq[123])-rhs[9]*(params.Aineq[137])-rhs[10]*(params.Aineq[151])-rhs[11]*(params.Aineq[165])-rhs[12]*(params.Aineq[179]);
  lhs[12] = -rhs[0]*(params.Aineq[12])-rhs[1]*(params.Aineq[26])-rhs[2]*(params.Aineq[40])-rhs[3]*(params.Aineq[54])-rhs[4]*(params.Aineq[68])-rhs[5]*(params.Aineq[82])-rhs[6]*(params.Aineq[96])-rhs[7]*(params.Aineq[110])-rhs[8]*(params.Aineq[124])-rhs[9]*(params.Aineq[138])-rhs[10]*(params.Aineq[152])-rhs[11]*(params.Aineq[166])-rhs[12]*(params.Aineq[180]);
  lhs[13] = -rhs[0]*(params.Aineq[13])-rhs[1]*(params.Aineq[27])-rhs[2]*(params.Aineq[41])-rhs[3]*(params.Aineq[55])-rhs[4]*(params.Aineq[69])-rhs[5]*(params.Aineq[83])-rhs[6]*(params.Aineq[97])-rhs[7]*(params.Aineq[111])-rhs[8]*(params.Aineq[125])-rhs[9]*(params.Aineq[139])-rhs[10]*(params.Aineq[153])-rhs[11]*(params.Aineq[167])-rhs[12]*(params.Aineq[181]);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.Aineq[0])-rhs[1]*(params.Aineq[1])-rhs[2]*(params.Aineq[2])-rhs[3]*(params.Aineq[3])-rhs[4]*(params.Aineq[4])-rhs[5]*(params.Aineq[5])-rhs[6]*(params.Aineq[6])-rhs[7]*(params.Aineq[7])-rhs[8]*(params.Aineq[8])-rhs[9]*(params.Aineq[9])-rhs[10]*(params.Aineq[10])-rhs[11]*(params.Aineq[11])-rhs[12]*(params.Aineq[12])-rhs[13]*(params.Aineq[13]);
  lhs[1] = -rhs[0]*(params.Aineq[14])-rhs[1]*(params.Aineq[15])-rhs[2]*(params.Aineq[16])-rhs[3]*(params.Aineq[17])-rhs[4]*(params.Aineq[18])-rhs[5]*(params.Aineq[19])-rhs[6]*(params.Aineq[20])-rhs[7]*(params.Aineq[21])-rhs[8]*(params.Aineq[22])-rhs[9]*(params.Aineq[23])-rhs[10]*(params.Aineq[24])-rhs[11]*(params.Aineq[25])-rhs[12]*(params.Aineq[26])-rhs[13]*(params.Aineq[27]);
  lhs[2] = -rhs[0]*(params.Aineq[28])-rhs[1]*(params.Aineq[29])-rhs[2]*(params.Aineq[30])-rhs[3]*(params.Aineq[31])-rhs[4]*(params.Aineq[32])-rhs[5]*(params.Aineq[33])-rhs[6]*(params.Aineq[34])-rhs[7]*(params.Aineq[35])-rhs[8]*(params.Aineq[36])-rhs[9]*(params.Aineq[37])-rhs[10]*(params.Aineq[38])-rhs[11]*(params.Aineq[39])-rhs[12]*(params.Aineq[40])-rhs[13]*(params.Aineq[41]);
  lhs[3] = -rhs[0]*(params.Aineq[42])-rhs[1]*(params.Aineq[43])-rhs[2]*(params.Aineq[44])-rhs[3]*(params.Aineq[45])-rhs[4]*(params.Aineq[46])-rhs[5]*(params.Aineq[47])-rhs[6]*(params.Aineq[48])-rhs[7]*(params.Aineq[49])-rhs[8]*(params.Aineq[50])-rhs[9]*(params.Aineq[51])-rhs[10]*(params.Aineq[52])-rhs[11]*(params.Aineq[53])-rhs[12]*(params.Aineq[54])-rhs[13]*(params.Aineq[55]);
  lhs[4] = -rhs[0]*(params.Aineq[56])-rhs[1]*(params.Aineq[57])-rhs[2]*(params.Aineq[58])-rhs[3]*(params.Aineq[59])-rhs[4]*(params.Aineq[60])-rhs[5]*(params.Aineq[61])-rhs[6]*(params.Aineq[62])-rhs[7]*(params.Aineq[63])-rhs[8]*(params.Aineq[64])-rhs[9]*(params.Aineq[65])-rhs[10]*(params.Aineq[66])-rhs[11]*(params.Aineq[67])-rhs[12]*(params.Aineq[68])-rhs[13]*(params.Aineq[69]);
  lhs[5] = -rhs[0]*(params.Aineq[70])-rhs[1]*(params.Aineq[71])-rhs[2]*(params.Aineq[72])-rhs[3]*(params.Aineq[73])-rhs[4]*(params.Aineq[74])-rhs[5]*(params.Aineq[75])-rhs[6]*(params.Aineq[76])-rhs[7]*(params.Aineq[77])-rhs[8]*(params.Aineq[78])-rhs[9]*(params.Aineq[79])-rhs[10]*(params.Aineq[80])-rhs[11]*(params.Aineq[81])-rhs[12]*(params.Aineq[82])-rhs[13]*(params.Aineq[83]);
  lhs[6] = -rhs[0]*(params.Aineq[84])-rhs[1]*(params.Aineq[85])-rhs[2]*(params.Aineq[86])-rhs[3]*(params.Aineq[87])-rhs[4]*(params.Aineq[88])-rhs[5]*(params.Aineq[89])-rhs[6]*(params.Aineq[90])-rhs[7]*(params.Aineq[91])-rhs[8]*(params.Aineq[92])-rhs[9]*(params.Aineq[93])-rhs[10]*(params.Aineq[94])-rhs[11]*(params.Aineq[95])-rhs[12]*(params.Aineq[96])-rhs[13]*(params.Aineq[97]);
  lhs[7] = -rhs[0]*(params.Aineq[98])-rhs[1]*(params.Aineq[99])-rhs[2]*(params.Aineq[100])-rhs[3]*(params.Aineq[101])-rhs[4]*(params.Aineq[102])-rhs[5]*(params.Aineq[103])-rhs[6]*(params.Aineq[104])-rhs[7]*(params.Aineq[105])-rhs[8]*(params.Aineq[106])-rhs[9]*(params.Aineq[107])-rhs[10]*(params.Aineq[108])-rhs[11]*(params.Aineq[109])-rhs[12]*(params.Aineq[110])-rhs[13]*(params.Aineq[111]);
  lhs[8] = -rhs[0]*(params.Aineq[112])-rhs[1]*(params.Aineq[113])-rhs[2]*(params.Aineq[114])-rhs[3]*(params.Aineq[115])-rhs[4]*(params.Aineq[116])-rhs[5]*(params.Aineq[117])-rhs[6]*(params.Aineq[118])-rhs[7]*(params.Aineq[119])-rhs[8]*(params.Aineq[120])-rhs[9]*(params.Aineq[121])-rhs[10]*(params.Aineq[122])-rhs[11]*(params.Aineq[123])-rhs[12]*(params.Aineq[124])-rhs[13]*(params.Aineq[125]);
  lhs[9] = -rhs[0]*(params.Aineq[126])-rhs[1]*(params.Aineq[127])-rhs[2]*(params.Aineq[128])-rhs[3]*(params.Aineq[129])-rhs[4]*(params.Aineq[130])-rhs[5]*(params.Aineq[131])-rhs[6]*(params.Aineq[132])-rhs[7]*(params.Aineq[133])-rhs[8]*(params.Aineq[134])-rhs[9]*(params.Aineq[135])-rhs[10]*(params.Aineq[136])-rhs[11]*(params.Aineq[137])-rhs[12]*(params.Aineq[138])-rhs[13]*(params.Aineq[139]);
  lhs[10] = -rhs[0]*(params.Aineq[140])-rhs[1]*(params.Aineq[141])-rhs[2]*(params.Aineq[142])-rhs[3]*(params.Aineq[143])-rhs[4]*(params.Aineq[144])-rhs[5]*(params.Aineq[145])-rhs[6]*(params.Aineq[146])-rhs[7]*(params.Aineq[147])-rhs[8]*(params.Aineq[148])-rhs[9]*(params.Aineq[149])-rhs[10]*(params.Aineq[150])-rhs[11]*(params.Aineq[151])-rhs[12]*(params.Aineq[152])-rhs[13]*(params.Aineq[153]);
  lhs[11] = -rhs[0]*(params.Aineq[154])-rhs[1]*(params.Aineq[155])-rhs[2]*(params.Aineq[156])-rhs[3]*(params.Aineq[157])-rhs[4]*(params.Aineq[158])-rhs[5]*(params.Aineq[159])-rhs[6]*(params.Aineq[160])-rhs[7]*(params.Aineq[161])-rhs[8]*(params.Aineq[162])-rhs[9]*(params.Aineq[163])-rhs[10]*(params.Aineq[164])-rhs[11]*(params.Aineq[165])-rhs[12]*(params.Aineq[166])-rhs[13]*(params.Aineq[167]);
  lhs[12] = -rhs[0]*(params.Aineq[168])-rhs[1]*(params.Aineq[169])-rhs[2]*(params.Aineq[170])-rhs[3]*(params.Aineq[171])-rhs[4]*(params.Aineq[172])-rhs[5]*(params.Aineq[173])-rhs[6]*(params.Aineq[174])-rhs[7]*(params.Aineq[175])-rhs[8]*(params.Aineq[176])-rhs[9]*(params.Aineq[177])-rhs[10]*(params.Aineq[178])-rhs[11]*(params.Aineq[179])-rhs[12]*(params.Aineq[180])-rhs[13]*(params.Aineq[181]);
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*params.Q[0])+rhs[1]*(2*params.Q[13])+rhs[2]*(2*params.Q[26])+rhs[3]*(2*params.Q[39])+rhs[4]*(2*params.Q[52])+rhs[5]*(2*params.Q[65])+rhs[6]*(2*params.Q[78])+rhs[7]*(2*params.Q[91])+rhs[8]*(2*params.Q[104])+rhs[9]*(2*params.Q[117])+rhs[10]*(2*params.Q[130])+rhs[11]*(2*params.Q[143])+rhs[12]*(2*params.Q[156]);
  lhs[1] = rhs[0]*(2*params.Q[1])+rhs[1]*(2*params.Q[14])+rhs[2]*(2*params.Q[27])+rhs[3]*(2*params.Q[40])+rhs[4]*(2*params.Q[53])+rhs[5]*(2*params.Q[66])+rhs[6]*(2*params.Q[79])+rhs[7]*(2*params.Q[92])+rhs[8]*(2*params.Q[105])+rhs[9]*(2*params.Q[118])+rhs[10]*(2*params.Q[131])+rhs[11]*(2*params.Q[144])+rhs[12]*(2*params.Q[157]);
  lhs[2] = rhs[0]*(2*params.Q[2])+rhs[1]*(2*params.Q[15])+rhs[2]*(2*params.Q[28])+rhs[3]*(2*params.Q[41])+rhs[4]*(2*params.Q[54])+rhs[5]*(2*params.Q[67])+rhs[6]*(2*params.Q[80])+rhs[7]*(2*params.Q[93])+rhs[8]*(2*params.Q[106])+rhs[9]*(2*params.Q[119])+rhs[10]*(2*params.Q[132])+rhs[11]*(2*params.Q[145])+rhs[12]*(2*params.Q[158]);
  lhs[3] = rhs[0]*(2*params.Q[3])+rhs[1]*(2*params.Q[16])+rhs[2]*(2*params.Q[29])+rhs[3]*(2*params.Q[42])+rhs[4]*(2*params.Q[55])+rhs[5]*(2*params.Q[68])+rhs[6]*(2*params.Q[81])+rhs[7]*(2*params.Q[94])+rhs[8]*(2*params.Q[107])+rhs[9]*(2*params.Q[120])+rhs[10]*(2*params.Q[133])+rhs[11]*(2*params.Q[146])+rhs[12]*(2*params.Q[159]);
  lhs[4] = rhs[0]*(2*params.Q[4])+rhs[1]*(2*params.Q[17])+rhs[2]*(2*params.Q[30])+rhs[3]*(2*params.Q[43])+rhs[4]*(2*params.Q[56])+rhs[5]*(2*params.Q[69])+rhs[6]*(2*params.Q[82])+rhs[7]*(2*params.Q[95])+rhs[8]*(2*params.Q[108])+rhs[9]*(2*params.Q[121])+rhs[10]*(2*params.Q[134])+rhs[11]*(2*params.Q[147])+rhs[12]*(2*params.Q[160]);
  lhs[5] = rhs[0]*(2*params.Q[5])+rhs[1]*(2*params.Q[18])+rhs[2]*(2*params.Q[31])+rhs[3]*(2*params.Q[44])+rhs[4]*(2*params.Q[57])+rhs[5]*(2*params.Q[70])+rhs[6]*(2*params.Q[83])+rhs[7]*(2*params.Q[96])+rhs[8]*(2*params.Q[109])+rhs[9]*(2*params.Q[122])+rhs[10]*(2*params.Q[135])+rhs[11]*(2*params.Q[148])+rhs[12]*(2*params.Q[161]);
  lhs[6] = rhs[0]*(2*params.Q[6])+rhs[1]*(2*params.Q[19])+rhs[2]*(2*params.Q[32])+rhs[3]*(2*params.Q[45])+rhs[4]*(2*params.Q[58])+rhs[5]*(2*params.Q[71])+rhs[6]*(2*params.Q[84])+rhs[7]*(2*params.Q[97])+rhs[8]*(2*params.Q[110])+rhs[9]*(2*params.Q[123])+rhs[10]*(2*params.Q[136])+rhs[11]*(2*params.Q[149])+rhs[12]*(2*params.Q[162]);
  lhs[7] = rhs[0]*(2*params.Q[7])+rhs[1]*(2*params.Q[20])+rhs[2]*(2*params.Q[33])+rhs[3]*(2*params.Q[46])+rhs[4]*(2*params.Q[59])+rhs[5]*(2*params.Q[72])+rhs[6]*(2*params.Q[85])+rhs[7]*(2*params.Q[98])+rhs[8]*(2*params.Q[111])+rhs[9]*(2*params.Q[124])+rhs[10]*(2*params.Q[137])+rhs[11]*(2*params.Q[150])+rhs[12]*(2*params.Q[163]);
  lhs[8] = rhs[0]*(2*params.Q[8])+rhs[1]*(2*params.Q[21])+rhs[2]*(2*params.Q[34])+rhs[3]*(2*params.Q[47])+rhs[4]*(2*params.Q[60])+rhs[5]*(2*params.Q[73])+rhs[6]*(2*params.Q[86])+rhs[7]*(2*params.Q[99])+rhs[8]*(2*params.Q[112])+rhs[9]*(2*params.Q[125])+rhs[10]*(2*params.Q[138])+rhs[11]*(2*params.Q[151])+rhs[12]*(2*params.Q[164]);
  lhs[9] = rhs[0]*(2*params.Q[9])+rhs[1]*(2*params.Q[22])+rhs[2]*(2*params.Q[35])+rhs[3]*(2*params.Q[48])+rhs[4]*(2*params.Q[61])+rhs[5]*(2*params.Q[74])+rhs[6]*(2*params.Q[87])+rhs[7]*(2*params.Q[100])+rhs[8]*(2*params.Q[113])+rhs[9]*(2*params.Q[126])+rhs[10]*(2*params.Q[139])+rhs[11]*(2*params.Q[152])+rhs[12]*(2*params.Q[165]);
  lhs[10] = rhs[0]*(2*params.Q[10])+rhs[1]*(2*params.Q[23])+rhs[2]*(2*params.Q[36])+rhs[3]*(2*params.Q[49])+rhs[4]*(2*params.Q[62])+rhs[5]*(2*params.Q[75])+rhs[6]*(2*params.Q[88])+rhs[7]*(2*params.Q[101])+rhs[8]*(2*params.Q[114])+rhs[9]*(2*params.Q[127])+rhs[10]*(2*params.Q[140])+rhs[11]*(2*params.Q[153])+rhs[12]*(2*params.Q[166]);
  lhs[11] = rhs[0]*(2*params.Q[11])+rhs[1]*(2*params.Q[24])+rhs[2]*(2*params.Q[37])+rhs[3]*(2*params.Q[50])+rhs[4]*(2*params.Q[63])+rhs[5]*(2*params.Q[76])+rhs[6]*(2*params.Q[89])+rhs[7]*(2*params.Q[102])+rhs[8]*(2*params.Q[115])+rhs[9]*(2*params.Q[128])+rhs[10]*(2*params.Q[141])+rhs[11]*(2*params.Q[154])+rhs[12]*(2*params.Q[167]);
  lhs[12] = rhs[0]*(2*params.Q[12])+rhs[1]*(2*params.Q[25])+rhs[2]*(2*params.Q[38])+rhs[3]*(2*params.Q[51])+rhs[4]*(2*params.Q[64])+rhs[5]*(2*params.Q[77])+rhs[6]*(2*params.Q[90])+rhs[7]*(2*params.Q[103])+rhs[8]*(2*params.Q[116])+rhs[9]*(2*params.Q[129])+rhs[10]*(2*params.Q[142])+rhs[11]*(2*params.Q[155])+rhs[12]*(2*params.Q[168]);
}
void fillq(void) {
  work.q[0] = params.q0[0];
  work.q[1] = params.q0[1];
  work.q[2] = params.q0[2];
  work.q[3] = params.q0[3];
  work.q[4] = params.q0[4];
  work.q[5] = params.q0[5];
  work.q[6] = params.q0[6];
  work.q[7] = params.q0[7];
  work.q[8] = params.q0[8];
  work.q[9] = params.q0[9];
  work.q[10] = params.q0[10];
  work.q[11] = params.q0[11];
  work.q[12] = params.q0[12];
}
void fillh(void) {
  work.h[0] = params.b[0];
  work.h[1] = params.b[1];
  work.h[2] = params.b[2];
  work.h[3] = params.b[3];
  work.h[4] = params.b[4];
  work.h[5] = params.b[5];
  work.h[6] = params.b[6];
  work.h[7] = params.b[7];
  work.h[8] = params.b[8];
  work.h[9] = params.b[9];
  work.h[10] = params.b[10];
  work.h[11] = params.b[11];
  work.h[12] = params.b[12];
  work.h[13] = params.b[13];
}
void fillb(void) {
  work.b[0] = 0;
  work.b[1] = 0;
  work.b[2] = 0;
  work.b[3] = 0;
  work.b[4] = 0;
  work.b[5] = 0;
}
void pre_ops(void) {
}
