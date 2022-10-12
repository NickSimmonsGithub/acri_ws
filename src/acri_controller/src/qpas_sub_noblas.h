/*----------------------------------------------------------------------------
 
  +----------------------------------------------+
  | Written by Adrian Wills,                     |
  |            School of Elec. Eng. & Comp. Sci. |
  |            University of Newcastle,          |
  |            Callaghan, NSW, 2308, AUSTRALIA   |
  |                                              |
  | Last Revised  25 May 2007.                   |
  |                                              |
  | Copyright (C) Adrian Wills.                  |
  +----------------------------------------------+
 
The current version of this software is free of charge and 
openly distributed, BUT PLEASE NOTE:

This software must be referenced when used in a published work.

This software may not be re-distributed as a part of a commercial product. 
If you distribute it in a non-commercial products, please contact me first, 
to make sure you ship the most recent version.

This software is distributed in the hope that it will be useful, 
but WITHOUT ANY WARRANTY; without even the implied warranty of 
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

IF IT FAILS TO WORK, IT'S YOUR LOSS AND YOUR PROBLEM.

--------------------------------------------------------------------------------*/
#ifndef QPAS_SUB_H_
#define QPAS_SUB_H_

#define INT_INT

#ifdef INT_INT
#define varint int
#endif

#ifdef INT_LONG_INT
#define varint long int
#endif

#ifdef INT_LONG_LONG_INT
#define varint long long int
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* The only ruotine in qp_sub.c */
varint qpas_sub_noblas( varint n,
varint me,
varint mc,
varint nl,
varint nu,
double * H,
double * f,
double * A,
double * bin,
double * lin,
double * uin,
double * x,
double * lm,
varint display,
varint * numits,
varint * numadd,
varint * numdrop
);

#ifdef __cplusplus
};
#endif

#endif
