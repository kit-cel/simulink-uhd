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

/* get UHD sensor information */


#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME uhd_sensor

/* C++ */
#include <string>
#include <map>
#include <vector>
#include <functional>
/* boost */
#include <boost/smart_ptr.hpp>
/* Matlab Simulink */
#include <simstruc.h>
#include <tmwtypes.h>
#include <mex.h>
/* UHD */
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/exception.hpp>
#include <uhd/types/sensors.hpp>
        
/* simulink-uhd */
#include <where_am_i.h>
#include <console_output_helpers.h>
#include <param_checks.h>

using namespace std;
using namespace uhd::usrp;


/**************************************************************************************************
 * misc
 **************************************************************************************************/

enum SFcnParamsIndex
{
	ARGS = 0, /* hint to find usrp */
    CLASS, /* 0 for MBOARD, 1 for TX, 2 for RX */
    CHANNEL_OR_MBORAD, /* channel or mboard index */
    NAME, /* name of sensor */
    DTYPEID, /* output data type (simulink build in) */
    SAMPLETIME, /* sample time of block */
    
	/* NUM_PARAMS must be the last in this enum to correctly set the number of expected parameters */
	/* Neet trick by Enno Klasing <klasing@int.uni-karlsruhe.de> */
	NUM_PARAMS
};

enum PWorkIndex
{
	USRP = 0, /* USRP object */
    GETSENSORVALUE, /* fkt ptr to get sensor */
	P_WORK_LENGTH
};

enum SensorClass { MBOARD = 0, TX, RX };

/**************************************************************************************************
 * helper methods
 **************************************************************************************************/

/* typedef boost::function3<uhd::sensor_value_t, multi_usrp*, const std::string &, size_t> getSensorValue_t; */
typedef uhd::sensor_value_t (uhd::usrp::multi_usrp::*getSensorValue_t)( const std::string &, size_t ); 

/**************************************************************************************************
 * S-function methods
 **************************************************************************************************/

/* ========================================================================*/
#if defined(MATLAB_MEX_FILE) 
#define MDL_CHECK_PARAMETERS
static void mdlCheckParameters(SimStruct *S)
/* ========================================================================*/
{
	WHERE_AM_I(S, "");
    
    CHAR_OR_DIE(S,ARGS);
        
    NUMERIC_NOTEMPTY_OR_DIE(S,CLASS);
    NUMERIC_NOTEMPTY_OR_DIE(S,CHANNEL_OR_MBORAD);
    CHAR_OR_DIE(S,NAME);
    
    NUMERIC_NOTEMPTY_OR_DIE(S,DTYPEID);
    NUMERIC_NOTEMPTY_OR_DIE(S,SAMPLETIME);   
}
#endif /* MDL_CHECK_PARAMETERS */


/* ======================================================================== */
#define MDL_INITIAL_SIZES
static void mdlInitializeSizes(SimStruct *S)
/* ======================================================================== */
{
    /* SS_SIMMODE_NORMAL = 0, ...SIZES_CALL_ONLY = 1, ...RTWGEN = 2, ...EXTERNAL = 3 */
	WHERE_AM_I(S, "SimMode: %d.", ssGetSimMode(S));
    
    uhd::msg::register_handler(&print2Matlab);

    /* set Number of expected parameters and check for a mismatch. */
    ssSetNumSFcnParams(S, NUM_PARAMS);  
    #if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) return;
    } else {
         return; 
    }
    #endif

    /* set up tunability */
    ssSetSFcnParamTunable(S, ARGS      , SS_PRM_NOT_TUNABLE);
    ssSetSFcnParamTunable(S, CHANNEL_OR_MBORAD, SS_PRM_NOT_TUNABLE);
    ssSetSFcnParamTunable(S, NAME      , SS_PRM_NOT_TUNABLE);
    ssSetSFcnParamTunable(S, DTYPEID   , SS_PRM_NOT_TUNABLE);
    ssSetSFcnParamTunable(S, SAMPLETIME, SS_PRM_NOT_TUNABLE);
    ssSetSFcnParamTunable(S, CLASS     , SS_PRM_NOT_TUNABLE);

    /* sampling */
    ssSetNumSampleTimes(S, 1);

    /* Set number of input ports */
    if ( !ssSetNumInputPorts(S, 0)) return;

    /* Set number of output ports */
    if (!ssSetNumOutputPorts(S, 1)) return;

    /* configure ports */
    DTypeId id = (DTypeId)(double)mxGetScalar(ssGetSFcnParam(S, DTYPEID));
        
    ssSetOutputPortDataType(S, 0, id);
    ssSetOutputPortComplexSignal(S, 0, COMPLEX_NO);
    ssSetOutputPortMatrixDimensions(S, 0, 1, 1); 

    
    /* Prepare work Vectors */
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, P_WORK_LENGTH);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    
    /* Specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S, 0);
}



/* ======================================================================== */
#define MDL_INITIALIZE_SAMPLE_TIMES
#if defined(MDL_INITIALIZE_SAMPLE_TIMES)
static void mdlInitializeSampleTimes(SimStruct *S)
/* ======================================================================== */
{    
	WHERE_AM_I(S, "");
    
    /* get sample time from parameter */
    time_T period = (time_T)mxGetScalar( ssGetSFcnParam(S, SAMPLETIME));
    
    /* set sampling */
    ssSetSampleTime(S, 0, period);
    ssSetOffsetTime(S, 0, 0.0);
}
#endif 



/* ======================================================================== */
#define MDL_START  /* Change to #undef to remove function */
static void mdlStart(SimStruct *S)
/* ======================================================================== */
{
	WHERE_AM_I(S, "");
   
    /* Set options of this Block */
    ssSetOptions(S, ssGetOptions(S) | SS_OPTION_CALL_TERMINATE_ON_EXIT);
    
    /* connect usrp */
    ssSetPWorkValue(S, USRP, NULL); 
    try
    {
        boost::shared_ptr<const char> args(mxArrayToString(ssGetSFcnParam(S, ARGS)),&mxFree);       
        multi_usrp::sptr usrp = multi_usrp::make(string(args.get()));

        /* Create a copy of the sptr on the heap */ 
        ssSetPWorkValue(S, USRP, new multi_usrp::sptr( usrp )); 
        
/*         {
             ssPrintf("Available Sensors:\n");
             std::vector<std::string> MBnames = usrp->get_mboard_sensor_names();
             for (int i = 0; i < MBnames.size(); i++) ssPrintf("MB: %s\n", MBnames[i]);
             std::vector<std::string> TXnames = usrp->get_tx_sensor_names();
             for (int i = 0; i < TXnames.size(); i++) ssPrintf("TX: %s\n", TXnames[i]);
             std::vector<std::string> RXnames = usrp->get_rx_sensor_names();
             for (int i = 0; i < RXnames.size(); i++) ssPrintf("TX: %s\n", RXnames[i]);
         } */

        /* setup sensor function */ 
        SensorClass sensorClass = (SensorClass)(int)(double)mxGetScalar(ssGetSFcnParam(S, CLASS));
        /* fkt ptr */
        getSensorValue_t * getSensorValue = new getSensorValue_t(); 
        ssSetPWorkValue(S, GETSENSORVALUE, getSensorValue); 
        
        switch(sensorClass)
        {
            case MBOARD:
                *getSensorValue = &multi_usrp::get_mboard_sensor;
                break;
            case TX:
                *getSensorValue = &multi_usrp::get_tx_sensor;
                break;
            case RX:
                *getSensorValue = &multi_usrp::get_rx_sensor;
                break;
            default:
                ssSetErrorStatusf(S,"Unsupported sensor class", "");
        }    
        
    }    
    catch (uhd::exception &e)
    {
        ssSetErrorStatusf(S,"UHD: %s",e.what());
        return;
    }  
}

/* ======================================================================== */
#define MDL_OUTPUTS
static void mdlOutputs(SimStruct *S, int_T tid)
/* ======================================================================== */
{
	/*WHERE_AM_I(S, ""); */
    
    /* Get Name */
    const string name = mxArrayToString(ssGetSFcnParam(S, NAME)); 
    /* Get Channel */
    const int channel_or_mboard = (int)(double)mxGetScalar(ssGetSFcnParam(S, CHANNEL_OR_MBORAD));  
    
    try
    {
        /* USRP */
        multi_usrp::sptr usrp = *static_cast<multi_usrp::sptr*>(ssGetPWorkValue(S, USRP));  
        /* Pointer to sensor function */
        getSensorValue_t getSensorValue = *static_cast<getSensorValue_t*>(ssGetPWorkValue(S, GETSENSORVALUE));  
        
        /* get value */
        uhd::sensor_value_t value = (usrp.get()->*getSensorValue)(name, channel_or_mboard);
        
        switch(ssGetOutputPortDataType(S,0))
        {
            case SS_DOUBLE:
                *(double *)ssGetOutputPortSignal(S, 0) = value.to_real();
                break;
            case SS_INT32:
                *(int *)ssGetOutputPortSignal(S, 0) = value.to_int();
                break;
            case SS_BOOLEAN:
                *(bool *) ssGetOutputPortSignal(S, 0) = value.to_bool();
                break;
            default:
                ssSetErrorStatusf(S,"Unsupported sensor data type", "");
        }    
        
    }    
    catch (uhd::exception &e)
    {
        ssSetErrorStatusf(S,"UHD: %s", e.what());
        return;
    }    

}


/* ======================================================================== */
static void mdlTerminate(SimStruct *S)
/* ======================================================================== */
{
	WHERE_AM_I(S, "");
    
    if( ssGetPWorkValue(S, USRP) ) 
        delete static_cast<multi_usrp::sptr*>(ssGetPWorkValue(S, USRP));
    
    if( ssGetPWorkValue(S, GETSENSORVALUE) )
        delete static_cast<getSensorValue_t*>(ssGetPWorkValue(S, GETSENSORVALUE));  
        
}


/**************************************************************************************************
 * Required S-function trailer
 ***************************************************************************************************/
extern "C"
{
#   ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#       include "simulink.c"      /* MEX-file interface mechanism */
#   else
#       include "cg_sfun.h"       /* Code generation registration function */
#   endif
}
