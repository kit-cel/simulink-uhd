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

/* Interface betweeen a USRP (UHD) and Simulink; the USRP acts as a data source */

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME uhd_source

/* C++ */
#include <string>
#include <map>
/* boost */
#include <boost/smart_ptr.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>
/* matlab simulink */
#include <simstruc.h>
#include <tmwtypes.h>
/* UHD */
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/stream.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/exception.hpp>
/* simulink-uhd */ 
#include <param_checks.h>
#include <where_am_i.h>
#include <console_output_helpers.h>
#include <param_setter.h>

using namespace std;
using namespace uhd::usrp;


/**************************************************************************************************
 * misc
 **************************************************************************************************/

enum SFcnParamsIndex
{
    ARGS = 0, /* char */
    /* device config, each is a cell with one string per mboard*/
    SUBDEV_SPEC,
    CLOCK_SRC,
    TIME_SRC,    
    /* physical channel config */
	RF_FREQ,
    GAIN,
    BANDWIDTH, 
    ANTENNA,
    /* logical channel config */
    NUM_CHANNELS,
    DTYPEID,
    FRAME_LENGTH,
    USE_FRAMES,
    
    SAMPLETIME,
    
    /* NUM_PARAMS must be the last in this enum to correctly set the number
     * of expected parameters.
	 * By Enno Klasing <klasing@int.uni-karlsruhe.de> 
     */
	NUM_PARAMS
};

enum PWorkIndex
{
	USRP = 0, /* USRP object */
    METADATA, /* UHD metadata object */
    STREAMER, /* stream iface for transmission */
    
	P_WORK_LENGTH
};
 
enum IWorkIndex
{
    RF_FREQ_PORT_INDEX, /* port index of RF_FREQ signal, 0 if none */
	GAIN_PORT_INDEX,    /* port index of GAIN signal, 0 if none */
    
	I_WORK_LENGTH
};

enum RWorkIndex
{
    LAST_RF_FREQ, /* holds current RF_FREQ (for port based setting) */
	LAST_GAIN,    /* holds current GAIN (for port based setting) */
    
	R_WORK_LENGTH
};

/* translation between matlab and UHD data type desc */
map<const DTypeId, const string> type_translation = boost::assign::map_list_of
    (SS_DOUBLE, "fc64")(SS_SINGLE, "fc32")(SS_INT16 , "sc16");


/**************************************************************************************************
 * S-function methods
 **************************************************************************************************/

/* ======================================================================== */
#if defined(MATLAB_MEX_FILE) 
#define MDL_CHECK_PARAMETERS
static void mdlCheckParameters(SimStruct *S)
/* ======================================================================== */
{
	WHERE_AM_I(S, "");
    
    CHAR_OR_DIE(S,ARGS);
    
    CHAR_OR_DIE(S,SUBDEV_SPEC);
    CHAR_OR_DIE(S,CLOCK_SRC);
    CHAR_OR_DIE(S,TIME_SRC);
    
    NUMERIC_OR_DIE(S,RF_FREQ);
    NUMERIC_OR_DIE(S,GAIN);
    NUMERIC_OR_DIE(S,BANDWIDTH);
    CHAR_OR_DIE(S,ANTENNA);
    
    NUMERIC_NOTEMPTY_OR_DIE(S,NUM_CHANNELS);
    NUMERIC_NOTEMPTY_OR_DIE(S,DTYPEID);
    NUMERIC_NOTEMPTY_OR_DIE(S,FRAME_LENGTH);
    NUMERIC_NOTEMPTY_OR_DIE(S,USE_FRAMES);
    
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
    
    /* redirect uhd output to matlab console */
    uhd::msg::register_handler(&print2Matlab);

    /* set number of expected parameters and check for a mismatch. */
    ssSetNumSFcnParams(S, NUM_PARAMS);  
    #if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) return;
    } else {
         return; 
    }
    #endif
            
    /* sampling */
    ssSetNumSampleTimes(S, PORT_BASED_SAMPLE_TIMES);

    /* Set number of input ports and tunability */
    ssSetSFcnParamTunable(S, ARGS,    SS_PRM_NOT_TUNABLE);
        
    int_T num_input_ports = 0;     
    int optinal_ports[] = {RF_FREQ, GAIN};
    BOOST_FOREACH( int param_index, optinal_ports )
    {
        int_T tunable = SS_PRM_SIM_ONLY_TUNABLE;
        if( mxIsEmpty(ssGetSFcnParam(S,param_index)) )
        {
            /* create a port and fix param */
            num_input_ports++;
            tunable = SS_PRM_NOT_TUNABLE;
        }
        ssSetSFcnParamTunable(S, param_index, tunable);
    }
    
    /* set the resulting number of ports */
    if (!ssSetNumInputPorts(S, num_input_ports)) return;
    
    /* port properties */
    for(int_T port = 0; port<num_input_ports; port++)
    {
        /* RF_FREQ and GAIN both have the same port spec */
        ssSetInputPortMatrixDimensions(S, port, 1, 1);
        ssSetInputPortComplexSignal(S, port, COMPLEX_NO);
        ssSetInputPortDataType(S, port,  SS_DOUBLE);
        ssSetInputPortFrameData(S, port, FRAME_INHERITED);
        ssSetInputPortDirectFeedThrough(S, port, 1);
        ssSetInputPortSampleTime(S, port, INHERITED_SAMPLE_TIME); 
        ssSetInputPortOffsetTime(S, port, 0.0);
    }
    
    
    /* Set number of output ports */
    if (!ssSetNumOutputPorts(S, 1)) return;
    /* data port properties */
    const int_T port = 0;
    {
        /* get data port properties */
        const int_T frame_length = (int_T)(double)mxGetScalar(ssGetSFcnParam(S, FRAME_LENGTH));
        const int_T num_channels = (int_T)(double)mxGetScalar(ssGetSFcnParam(S, NUM_CHANNELS));
        const DTypeId id = (DTypeId)(double)mxGetScalar(ssGetSFcnParam(S, DTYPEID));
        const Frame_T outputsFrames =  ((double)mxGetScalar(ssGetSFcnParam(S, USE_FRAMES))>0.0)? FRAME_YES : FRAME_NO;
        const time_T period = (time_T)((double)mxGetScalar(ssGetSFcnParam(S, SAMPLETIME)) * frame_length);

        /* set data port properties */
        ssSetOutputPortMatrixDimensions(S, port, frame_length, num_channels);
        ssSetOutputPortComplexSignal(S, port, COMPLEX_YES);
        ssSetOutputPortDataType(S, port, id);
        ssSetOutputPortFrameData(S, port, outputsFrames);
        ssSetOutputPortSampleTime(S, port, period);
        ssSetOutputPortOffsetTime(S, port, 0.0);
    }

    /* Prepare work Vectors */
    ssSetNumRWork(S, R_WORK_LENGTH);
    ssSetNumIWork(S, I_WORK_LENGTH);
    ssSetNumPWork(S, P_WORK_LENGTH);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    
    /* Specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S, 0);
}

/* ======================================================================== */
static void mdlInitializeSampleTimes(SimStruct *S)
/* ======================================================================== */
{    
    /* PORT_BASED_SAMPLE_TIMES */
}


/* ======================================================================== */
#define MDL_ENABLE
void mdlEnable(SimStruct * S)
/* ======================================================================== */
{
	WHERE_AM_I(S, "");
    
    uhd::stream_cmd_t stream_cmd(
        uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS
    );
    stream_cmd.stream_now = true;
    
    try
    {
        multi_usrp::sptr usrp = *static_cast<multi_usrp::sptr*>(ssGetPWorkValue(S, USRP)); 
        usrp->issue_stream_cmd(stream_cmd);
    }
    catch (uhd::exception &e)
    {
        ssSetErrorStatusf(S,"UHD: %s", e.what());
    }  
}     

/* ======================================================================== */
#define MDL_DISABLE
void mdlDisable(SimStruct * S)
/* ======================================================================== */
{
	WHERE_AM_I(S, "");
    
    try
    {
        multi_usrp::sptr usrp = *static_cast<multi_usrp::sptr*>(ssGetPWorkValue(S, USRP)); 
        usrp->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    }
    catch (uhd::exception &e)
    {
        ssSetErrorStatusf(S,"UHD: %s", e.what());
    }      
}  



   
/* ======================================================================== */
#define MDL_PROCESS_PARAMETERS  
static void mdlProcessParameters(SimStruct *S)
/* ======================================================================== */
{
	WHERE_AM_I(S, "");
    
    /* get usrp object */
    multi_usrp::sptr usrp = *static_cast<multi_usrp::sptr*>(ssGetPWorkValue(S, USRP)); 
    
    try
    {
        const size_t num_chans = usrp->get_rx_num_channels();
        
        /* rf freq (center frequency) */
        param_setter<double>(usrp, &multi_usrp::set_rx_freq, ssGetSFcnParam(S, RF_FREQ), num_chans);

        /* rf gain */
        /* casting overloaded uhd function to remove ambiguity */
        param_setter<double>(usrp, 
                static_cast< void (multi_usrp::*)(double, size_t)>(&multi_usrp::set_rx_gain), 
                ssGetSFcnParam(S, GAIN), num_chans
        );
    }
    catch (uhd::exception &e)
    {
        ssSetErrorStatusf(S,"UHD: %s", e.what());
    }  
} 


/* ======================================================================== */
#define MDL_START  /* Change to #undef to remove function */
static void mdlStart(SimStruct *S)
/* ======================================================================== */
{
	WHERE_AM_I(S, "");
   
    /* Set options of this Block */
    ssSetOptions(S, ssGetOptions(S) | SS_OPTION_CALL_TERMINATE_ON_EXIT);
    
    /* assign optional ports */
    int next_port = 0; /* no fixed/other inputs */
    if( mxIsEmpty(ssGetSFcnParam(S,RF_FREQ)) ) 
        ssSetIWorkValue(S, RF_FREQ_PORT_INDEX, next_port++);
    if( mxIsEmpty(ssGetSFcnParam(S, GAIN)) ) 
        ssSetIWorkValue(S, GAIN_PORT_INDEX, next_port);
    
    /* set impossible values as last values */
    ssSetRWorkValue(S, LAST_RF_FREQ, -0.1);
    ssSetRWorkValue(S, LAST_GAIN, -1e7);
    
    /* connect and configure usrp */
    ssSetPWorkValue(S, USRP, NULL);
    try
    {
        /* get usrp(s) */
        boost::shared_ptr<const char> args(mxArrayToString(ssGetSFcnParam(S, ARGS)),&mxFree);       
        multi_usrp::sptr usrp = multi_usrp::make(string(args.get()));
              
        /* Create a copy of the sptr on the heap */
        ssSetPWorkValue(S, USRP, new multi_usrp::sptr ( usrp ));  
                    
        /******************************************************************
         * mboard config
         *****************************************************************/
        
        const size_t num_mboards = usrp->get_num_mboards();
                
        /* set clock source(s) */
        param_setter<string>(usrp, &multi_usrp::set_clock_source, ssGetSFcnParam(S, CLOCK_SRC), num_mboards);
        
        /* set time source(s) */
        param_setter<string>(usrp, &multi_usrp::set_time_source, ssGetSFcnParam(S, TIME_SRC), num_mboards);
        
        /* set subdev spec */
        param_setter<string>(usrp, &multi_usrp::set_rx_subdev_spec, ssGetSFcnParam(S, SUBDEV_SPEC), num_mboards);
                
        /* sample rate   */  
        double rate = ssGetOutputPortDimensions(S, 0)[0] / ssGetOutputPortSampleTime(S, 0);
        usrp->set_rx_rate(rate);
//         double diff = usrp->get_rx_rate() - rate;
//         if ( diff > 1e-5 || diff < -1e-5 )
//            ssWarningf(S, "Could not set rx rate to %f MSps (became %f MSps)", 
//                 rate/1e6, usrp->get_rx_rate()/1e6
//            );
        
        /******************************************************************
         * channel config
         *****************************************************************/
        
        const int num_chans = (int_T)(double)mxGetScalar(ssGetSFcnParam(S, NUM_CHANNELS));  
        /* check if there is one stream per channel */
        if (num_chans != usrp->get_rx_num_channels() )
        {
            ssSetErrorStatusf(S, "Number of channels does not match USRP configuration", "");
            return;
        }
        
        /* set antenna port(s) */
        param_setter<string>(usrp, &multi_usrp::set_rx_antenna, ssGetSFcnParam(S, ANTENNA), num_chans);

        /* RF filter bandwidth */
        param_setter<double>(usrp, &multi_usrp::set_rx_bandwidth, ssGetSFcnParam(S, BANDWIDTH), num_chans);
                
        /* RF gain & freq */
        mdlProcessParameters(S);
        
        
        /******************************************************************
         * streamer config
         *****************************************************************/
        
        /* metadata struct */
        uhd::rx_metadata_t *md = new uhd::rx_metadata_t(); 
        ssSetPWorkValue(S, METADATA, md);

        /* Get and config stream io */  
        uhd::stream_args_t stream_args(
            type_translation[ (int)(*(double*)mxGetData(ssGetSFcnParam(S, DTYPEID))) ], "sc16"
        );
        for (size_t chan = 0; chan < num_chans; chan++)
            stream_args.channels.push_back(chan); //linear mapping
        
        ssSetPWorkValue(S, STREAMER, new uhd::rx_streamer::sptr( usrp->get_rx_stream( stream_args )));
               
    }
    catch (uhd::exception &e)
    {
        ssSetErrorStatusf(S,"UHD: %s", e.what());
        return;
    }   

}


/* ======================================================================== */
#if defined(MATLAB_MEX_FILE) 
#define MDL_SIM_STATUS_CHANGE
static void mdlSimStatusChange(SimStruct *S, ssSimStatusChangeType simStatus)
/* ======================================================================== */
{
    WHERE_AM_I(S, "");
    
    if (simStatus == SIM_PAUSE) 
    { 
        mdlDisable(S);
    } 
    else if (simStatus == SIM_CONTINUE) 
    { 
        mdlEnable(S);        
    } 
} 
#endif 

/* ======================================================================== */
#define MDL_OUTPUTS
static void mdlOutputs(SimStruct *S, int_T tid)
/* ======================================================================== */
{
    /*WHERE_AM_I(S, ""); */
    
    /* import vars */
    multi_usrp::sptr usrp = *static_cast<multi_usrp::sptr*>(ssGetPWorkValue(S, USRP)); 
    uhd::rx_metadata_t *md = static_cast<uhd::rx_metadata_t*>(ssGetPWorkValue(S, METADATA)); 
    uhd::rx_streamer::sptr rx_stream = *static_cast<uhd::rx_streamer::sptr*>(ssGetPWorkValue(S, STREAMER));

    try
    {   
        const int_T rf_freq_port_index = ssGetIWorkValue(S, RF_FREQ_PORT_INDEX);
        if(mxIsEmpty(ssGetSFcnParam(S,RF_FREQ)) && ssIsSampleHit(S, ssGetInputPortSampleTimeIndex(S, rf_freq_port_index), tid) )
        {  
            /* set new rf freq */
            real_T new_rf_freq = *ssGetInputPortRealSignalPtrs(S,rf_freq_port_index)[0];
            if(ssGetRWorkValue(S, LAST_RF_FREQ) != new_rf_freq)
            {
                ssSetRWorkValue(S, LAST_RF_FREQ, new_rf_freq);
                /* remember value to avoid unnecessary updates */
                usrp->set_rx_freq(new_rf_freq);
            }
        }
        const int_T gain_port_index = ssGetIWorkValue(S, GAIN_PORT_INDEX);
        if(mxIsEmpty(ssGetSFcnParam(S,RF_FREQ)) && ssIsSampleHit(S, ssGetInputPortSampleTimeIndex(S, gain_port_index), tid) )
        {
            /* set new gain */
            real_T new_gain = *ssGetInputPortRealSignalPtrs(S,gain_port_index)[0];
            if(ssGetRWorkValue(S, LAST_GAIN) != new_gain)
            {
                ssSetRWorkValue(S, LAST_GAIN, new_gain);
                /* remember value to avoid unnecessary updates */
                usrp->set_rx_gain(new_gain);
            }
        }

        /* receive sample data */
        if(ssIsSampleHit(S, ssGetOutputPortSampleTimeIndex(S, 0), tid) )
        {
            const size_t num_chans = rx_stream->get_num_channels();
            const int samples_per_buffer = ssGetOutputPortDimensions(S, 0)[0];
            const int sizeof_buffer = samples_per_buffer * 
                    ssGetDataTypeSize(S, ssGetOutputPortDataType(S, 0));
            
            /* prepare buffer: put data pointer for each channel in vector */
            vector<const void *> rv;
            for(int chan=0; chan<num_chans; chan++)
                rv.push_back((void*)((char*)ssGetOutputPortSignal(S, 0) + chan*sizeof_buffer));
                
            /* fetch iq samples */
            rx_stream->recv(rv, samples_per_buffer, *md);  
            
        }
    }
    catch (uhd::exception &e)
    {
        ssSetErrorStatusf(S,"UHD: %s", e.what());
    }    
}


/* ======================================================================== */
static void mdlTerminate(SimStruct *S)
/* ======================================================================== */
{
	WHERE_AM_I(S, "");
    
    /* check if USRP object has been created */
    if( ssGetPWorkValue(S, USRP) ) 
    {        
        /* stop receiving */
        mdlDisable(S);
        
        /* clear usrp memory */
        delete static_cast<multi_usrp::sptr*>(ssGetPWorkValue(S, USRP));
    }
    
    /* clear metadata_t memory */
    if( ssGetPWorkValue(S, METADATA) ) 
        delete static_cast<uhd::rx_metadata_t*>(ssGetPWorkValue(S, METADATA));  
    
    /* clear io_type_t memory */
    if( ssGetPWorkValue(S, STREAMER) ) 
        delete static_cast<uhd::rx_streamer::sptr*>(ssGetPWorkValue(S, STREAMER));  
}


/**************************************************************************************************
 * Required S-function trailer
 **************************************************************************************************/
extern "C"
{
#   ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#       include "simulink.c"      /* MEX-file interface mechanism */
#   else
#       include "cg_sfun.h"       /* Code generation registration function */
#   endif
}


