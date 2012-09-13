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

/* Function to read multirow char array in string vector */

#include <getMxStringArray.h>
#include <boost/scoped_array.hpp> 

vector<string>
getMxStringArray(const mxArray *string_array_ptr)
{
    /* result */
    vector<string> string_vector;
    
    /* allocate and store chars in buffer */ 
    const size_t char_buffer_len = mxGetNumberOfElements(string_array_ptr) + 1;
    boost::scoped_array<char> char_buffer(new char[char_buffer_len]);
    
    /* fill char buffer */
    if (mxGetString(string_array_ptr, char_buffer.get(), (mwSize)char_buffer_len) != 0) 
        return string_vector;

    /* Get the shape of the input mxArray. */
    const size_t max_length = mxGetN(string_array_ptr); /* max length of strings */
    const size_t count  = mxGetM(string_array_ptr); /* number of strings */

    boost::scoped_array<char> string_buffer(new char[max_length+1]);
    /* terminate string, other elements will be set in folling for loop */
    string_buffer[max_length] = '\0';
  
    for (size_t n=0; n<count; n++)  {
        /* for each string get the column-major elements from char buffer */      
        for (size_t index=n, pos=0; index<char_buffer_len-1; index+=count)
            string_buffer[pos++] = char_buffer[index];
        /* now string_buffer holds the nth element (and possibly multiple terminating symbols) */
        string_vector.push_back(string(string_buffer.get()));
    }
    
    return string_vector;
}