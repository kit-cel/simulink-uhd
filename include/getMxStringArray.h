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

#ifndef GET_MX_STRING_ARRAY_H
#define GET_MX_STRING_ARRAY_H

#include <string>
#include <vector>
/* Matlab Simulink */
#include <simstruc.h>
#include <tmwtypes.h>

# ifndef  MATLAB_MEX_FILE
#   include <rt_matrx.h>
# endif

using namespace std;

vector<string> getMxStringArray(const mxArray *string_array_ptr);

#endif /* GET_MX_STRING_ARRAY_H */
