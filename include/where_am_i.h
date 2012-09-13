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

#ifndef WHERE_AM_I_H
#define WHERE_AM_I_H

#ifdef _TRACE
#   ifdef MATLAB_MEX_FILE
#       define WHERE_AM_I(S, message, ...) ssPrintf("%s: %s. " message "\n", ssGetPath(S), __FUNCTION__, __VA_ARGS__)
#   else
#       define WHERE_AM_I(S, message, ...) ssPrintf("RTW: %s: %s. " message "\n", ssGetPath(S), __FUNCTION__, __VA_ARGS__)
#   endif
#else
#   define WHERE_AM_I(S, message, ...) (1)
#endif

#endif /* WHERE_AM_I_H */