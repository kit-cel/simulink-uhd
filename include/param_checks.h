/*
* Copyright 2012 Communications Engineering Lab, KIT
*
* This is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 3, or (at your option)
* any later version.
*
* This software is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this software; see the file COPYING. If not, write to
* the Free Software Foundation, Inc., 51 Franklin Street,
* Boston, MA 02110-1301, USA.
*/

#ifndef PARAM_CHECKS
#define PARAM_CHECKS
    
// printf for "ssWarning"
#define ssWarningf(S, message, ...) \
    {char errMsg[512];sprintf(errMsg, message, __VA_ARGS__);ssWarning(S, errMsg);}

#define CHAR_OR_DIE(S, param) \
    if (!mxIsChar( ssGetSFcnParam(S,param) )) \
        {ssSetErrorStatusf(S,"? parameter to S-function must be a string", "");return;}
    
#define CELL_OR_DIE(S, param) \
    if (!mxIsCell( ssGetSFcnParam(S,param) )) \
        {ssSetErrorStatusf(S,"? parameter to S-function must be a cell", "");return;}

#define NUMERIC_OR_DIE(S, param) \
    if (!mxIsNumeric( ssGetSFcnParam(S,param) )) \
        {ssSetErrorStatusf(S,"? parameter to S-function must be numeric", "");return;}

#define NUMERIC_NOTEMPTY_OR_DIE(S, param) \
    if (!mxIsNumeric( ssGetSFcnParam(S,param) ) || mxIsEmpty( ssGetSFcnParam(S,param) ) ) \
        {ssSetErrorStatusf(S,"? parameter to S-function must be numeric", "");return;}

#endif /* PARAM_CHECKS */