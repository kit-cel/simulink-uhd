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

#ifndef CONSOLE_OUTPUT_HELPERS
#define CONSOLE_OUTPUT_HELPERS

// redirect UHD output to matlab console
inline void print2Matlab(uhd::msg::type_t t, const std::string &msg)
{
    ssPrintf( "%s\n", msg.c_str() );
};

// printf for "ssSetErrorStatus"
// ssSetErrorStatus() expects a static character array, that's why we define this error string in a file-wide variable.
static char errorMessage[512];
#define ssSetErrorStatusf(S, message, ...) \
    sprintf(errorMessage, message, __VA_ARGS__); \
    ssSetErrorStatus(S, errorMessage);

#endif /* CONSOLE_OUTPUT_HELPERS */