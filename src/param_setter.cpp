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

/* Universal function to set various uhd params */

#include <param_setter.h>

template<typename T>
void param_setter(
    multi_usrp::sptr usrp, boost::function<void (multi_usrp*, T, size_t)> setter, 
    const mxArray* value, const size_t count
)
{
    if( ! mxIsEmpty(value) )
    {
        const size_t num_elems = mxGetNumberOfElements(value);

        /* one value per channel? if not assign first value to all channels */
        if( num_elems >= count)
        {                                    
            for(int i=0; i<count; i++)
            {
                const T v = static_cast<T*>(mxGetData(value))[i];
                setter(usrp.get(), v, i);
                /* ssPrintf("Setting %d to '%g'\n", i, v); */
            }
        }
        else
        {
            const T v = *static_cast<T*>(mxGetData(value));
            for(int i=0; i<count; i++) setter(usrp.get(), v, i);
            /* ssPrintf("Setting all to '%g'\n", v); */
        }
    }
}

template <>
void param_setter<string>(
    multi_usrp::sptr usrp, boost::function<void (multi_usrp*, string, size_t)> setter, 
    const mxArray* value, const size_t count
)
{        
    if( ! mxIsEmpty(value) )
    {
        const vector<string> svalues = getMxStringArray(value);
        const size_t num_elems = svalues.size();

        if( num_elems > 0 )
        {
            /* one value per channel? if not assign first value to all channels */
            if( num_elems == count)
            {                                    
                for(int i=0; i<count; i++)
                {              
                    setter(usrp.get(), svalues[i], i);
                }
            }
            else
            {
                for(int i=0; i<count; i++) setter(usrp.get(), svalues[0], i);
            }
        }
    }
}

template void param_setter<double> (
    multi_usrp::sptr usrp, boost::function<void (multi_usrp*, double, size_t)> setter, 
    const mxArray* value, const size_t count
);