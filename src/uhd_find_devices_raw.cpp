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

/* Find attached USRPs */

/* C++ Includes */
#include <string.h>
#include <simstruc.h>
#include <tmwtypes.h>
#include <mex.h>
#include <sstream>
#include <iostream>

/* UHD includes */
#include <uhd/exception.hpp>
#include <uhd/device.hpp>

/* #define DEBUG */

#define NUMBER_OF_FIELDS 4

using namespace std;


/* Entry point to C/C++ */
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    const char          *field_names[] = {"type", "addr", "name", "serial"};
    uhd::device_addrs_t device_addrs;
    uhd::device_addr_t  hint;
    string              identifier;
    uhd::device_addr_t  dev;
    int                 number_of_devices;
    
    
    #ifdef DEBUG
        mexPrintf("mexFunction\n");
    #endif

    /* ================================================================= */
    /* check input and ouput
    /* ================================================================= */
            
    if (nrhs == 0) {
        
        /* No hint given */
        hint = string();
    }
    else if (nrhs == 1 && mxIsChar(prhs[0])) {
        
        /* Get identifier */
        identifier = mxArrayToString(prhs[0]);
        
        #ifdef DEBUG
            mexPrintf("Has args\n");
        #endif
        
        /* check if identifier contains "=" */
        if ((int)identifier.find("=") > 0)
            hint=identifier;
        /* .. if not, assume identifier is an IP address */
        else
            hint=string("addr=") + identifier;
    }        
    else {
		mexErrMsgTxt(   "\nUsage: []=uhd_find_devices(identifier) \n\n"
                        "       This function finds UHD devices attached to the host. The identifier can be used to filter the output.\n\n"
                        "       identifier: A string that specifies the unique identifier of the UHD device.\n\n"
                        "       Possible identifiers are:\n\n"
                        "\t     IP address (addr), e.g. addr=192.168.55.3\n"
                        "\t     Serial number (serial), e.g. serial=EGR18WFRP\n"
                        "\t     Device Type (type), e.g. type=usrp2\n"
                        "\t     Device Name (name), e.g. name=myUSRP\n\n"
                        "       In the case of an IP Address, the hint addr= can be omitted.\n\n"
                        "Example: a=uhd_find_devices, or\n"
                        "         a=uhd_find_devices('serial=EGR18WFRP')\n\n"
                );
    }
            
    
    /* ================================================================= */
    /* find devices
    /* ================================================================= */
    
    #ifdef DEBUG
            mexPrintf("hint=%s\n",(hint.to_string()).c_str());
    #endif
    
    try {
    
        device_addrs.clear();
        device_addrs = uhd::device::find(hint); 
        number_of_devices=(int)device_addrs.size();
    }
    catch (uhd::exception &e) {
        
        mexErrMsgTxt(e.what());
    }
    
    /* if no device present output 0 and exit */
    if (number_of_devices == 0){
        plhs[0] = mxCreateDoubleScalar(0);
    }
    
    /**
     *  Save into cell array
     */
    
    #ifdef DEBUG
            mexPrintf("Number of detected devices: %d\n",number_of_devices);
    #endif
            
    mwSize dims[2] = {1,(mwSize)number_of_devices};
    
	/* ================================================================= */
    /* create output
    /* ================================================================= */
    
    /* Create output structure */
    plhs[0] = mxCreateStructArray(2, dims, NUMBER_OF_FIELDS, field_names);
    
    for (int i=0; i<number_of_devices; i++) {
    
        /* Get device string for device number i */
        
        dev=device_addrs[i];
        
        #ifdef DEBUG
            mexPrintf("Device string for number %d: %s\n",i+1,dev.to_string().c_str());
        #endif
                
        mxSetField(plhs[0],i,"type",mxCreateString(dev["type"].c_str()));
        mxSetField(plhs[0],i,"addr",mxCreateString(dev["addr"].c_str()));
        mxSetField(plhs[0],i,"name",mxCreateString(dev["name"].c_str()));
        mxSetField(plhs[0],i,"serial",mxCreateString(dev["serial"].c_str()));
    }
}