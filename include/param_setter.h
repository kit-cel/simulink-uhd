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

#ifndef PARAM_SETTER_H
#define PARAM_SETTER_H

#include <string>
/* boost */
#include <boost/smart_ptr.hpp>
#include <boost/function.hpp>
/* Matlab Simulink */
#include <simstruc.h>
#include <tmwtypes.h>

# ifndef  MATLAB_MEX_FILE
#   include <rt_matrx.h>
# endif

/* UHD */
#include <uhd/usrp/multi_usrp.hpp>
/* simulink-UHD */
#include <getMxStringArray.h>

using namespace std;
using namespace uhd::usrp;

template<typename T> void 
param_setter(
        multi_usrp::sptr usrp, 
        boost::function<void (multi_usrp*, T, size_t)> setter, 
        const mxArray* value,
        const size_t count 
);

#endif /* PARAM_SETTER_H */
