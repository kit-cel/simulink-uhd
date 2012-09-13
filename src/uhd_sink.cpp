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

/* Interface betweeen the USRP (UHD) and Simulink, the USRP acts as a data sink */

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME uhd_sink

/* C++ */
#include <complex>
#include <string>
#include <map>
/* boost */
#include <boost/smart_ptr.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>
/* Matlab Simulink */
#include <simstruc.h>
#include <tmwtypes.h>
/* UHD */
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/stream.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/exception.hpp>
#include <uhd/types/ref_vector.hpp>
/* simulink-uhd */
#include <param_checks.h>
#include <where_am_i.h>
#include <console_output_helpers.h>
#include <param_setter.h>
#include <getMxStringArray.h>

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
    /* channel config */
	RF_FREQ,
    GAIN,
    BANDWIDTH, 
    ANTENNA,
            
    SAMPLETIME, /* double */
    /* NUM_PARAMS must be the last in this enum to correctly set the number of expected parameters
	 Neat trick by Enno Klasing <klasing@int.uni-karlsruhe.de> */
	NUM_PARAMS
};

#define SAMPLETIME_INHERINT -1

enum PWorkIndex
{
	USRP = 0, /* USRP object */
    METADATA, /* UHD metadata object */
    STREAMER,  /* stream iface for transmission */
    
    LAST_RF_FREQS,
    LAST_RF_GAINS,
    
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
    LAST_RF_FREQ, /* remember RF_FREQ (for port based setting) */
	LAST_GAIN,    /* remember GAIN (for port based setting) */
    
	R_WORK_LENGTH
};

map<const DTypeId, const string> type_translation = boost::assign::map_list_of
    (SS_DOUBLE, "fc64")(SS_SINGLE, "fc32")(SS_INT16 , "sc16");

typedef vector<double> dvec_t;
typedef boost::shared_ptr<dvec_t> dvec_sptr;

/**************************************************************************************************
 * helper methods
 **************************************************************************************************/

/* send an empty burst with EOB flag set */
/* will be called on pause, disable, terminate event */
void sendEOB(SimStruct *S)
{
    if( ssGetPWorkValue(S, STREAMER) && ssGetPWorkValue(S, METADATA) ) 
    {
        uhd::tx_streamer::sptr tx_stream = *static_cast<uhd::tx_streamer::sptr*>(ssGetPWorkValue(S, STREAMER));
        uhd::tx_metadata_t *md = static_cast<uhd::tx_metadata_t*>(ssGetPWorkValue(S, METADATA)); 
        
        /* check if a burst has been started */
        if( ! md->start_of_burst )
        {
            /* Send an end of burst */
            md->end_of_burst = true;
            try
            {
                /* create some dummy pointer vector */
                cint16_T foo;
                vector<const void *> rv;
                for(int chan=0; chan<tx_stream->get_num_channels(); chan++) 
                    rv.push_back(&foo);
                /* signal stream end */
                tx_stream->send(rv, 0, *md);
            }
            catch (uhd::exception &e)
            {
                ssSetErrorStatusf(S,"UHD: %s",e.what());
            }      
        }
    }
}
    

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
    
    CHAR_OR_DIE(S,SUBDEV_SPEC);
    
    CHAR_OR_DIE(S,CLOCK_SRC);
    CHAR_OR_DIE(S,TIME_SRC);
    
    NUMERIC_OR_DIE(S,RF_FREQ);
    NUMERIC_OR_DIE(S,GAIN);
    NUMERIC_NOTEMPTY_OR_DIE(S,BANDWIDTH);
    CHAR_OR_DIE(S,ANTENNA);
    
    NUMERIC_NOTEMPTY_OR_DIE(S,SAMPLETIME);
}
#endif /* MDL_CHECK_PARAMETERS */


/* ========================================================================*/
#define MDL_INITIAL_SIZES
static void mdlInitializeSizes(SimStruct *S)
/* ========================================================================*/
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
            
    /* sampling */
    ssSetNumSampleTimes(S, PORT_BASED_SAMPLE_TIMES);

    /* Set number of input ports and tunability */
    
    ssSetSFcnParamTunable(S, ARGS, SS_PRM_NOT_TUNABLE);
    int_T num_input_ports = 1; /* always have a data in */
    
    int optinal_ports[] = { RF_FREQ, GAIN };
    BOOST_FOREACH( int param_index, optinal_ports )
    {
        int_T tunable = SS_PRM_SIM_ONLY_TUNABLE;
        if( mxIsEmpty(ssGetSFcnParam(S,param_index)))
        {
            /* create a port and fix param */
            num_input_ports++;
            tunable = SS_PRM_NOT_TUNABLE;
        }
        ssSetSFcnParamTunable(S, param_index, tunable);
    }
    
    /* set the resulting number of ports */
    if (!ssSetNumInputPorts(S, num_input_ports)) return;
    
    /* DATA PORT */
    int_T port = 0;
    {
        /* allow for variable frame lengths and number of channels */
        ssSetInputPortMatrixDimensions(S, port, 
                DYNAMICALLY_SIZED, DYNAMICALLY_SIZED);
        /* require a complex signal */
        ssSetInputPortComplexSignal(S, port, COMPLEX_YES);
        /* let other block define data type */
        ssSetInputPortDataType(S, port,  DYNAMICALLY_TYPED);
        /* frame or no frame....who cares */
        ssSetInputPortFrameData(S, port, FRAME_INHERITED);
        /* need to access the input signals */
        ssSetInputPortDirectFeedThrough(S, port, 1);
        /* by this the interpolation in the usrp will be defined */
        ssSetInputPortSampleTime(S, port, INHERITED_SAMPLE_TIME);
        ssSetInputPortOffsetTime(S, port, 0.0);
    }
    /* OTHER PORTS */
    for( ++port; port<num_input_ports; port++)
    {
        /* RF_FREQ and GAIN both have the same port spec */
        ssSetInputPortMatrixDimensions(S, port, 1, DYNAMICALLY_SIZED);
        ssSetInputPortComplexSignal(S, port, COMPLEX_NO);
        ssSetInputPortDataType(S, port,  SS_DOUBLE);
        ssSetInputPortFrameData(S, port, FRAME_INHERITED);
        ssSetInputPortDirectFeedThrough(S, port, 1);
        ssSetInputPortSampleTime(S, port, INHERITED_SAMPLE_TIME); 
        ssSetInputPortOffsetTime(S, port, 0.0);
    }
    
    
    /* Set number of output ports */
    if (!ssSetNumOutputPorts(S, 0)) return;

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

/* ========================================================================*/
#if defined(MATLAB_MEX_FILE) 
#define MDL_SET_INPUT_PORT_SAMPLE_TIME
void mdlSetInputPortSampleTime(SimStruct *S, int_T port, real_T sampleTime, real_T offsetTime)
/* ========================================================================*/
{
	WHERE_AM_I(S, "");
    
    ssSetInputPortSampleTime(S, port, sampleTime);
}
#endif 


/* ========================================================================*/
#if defined(MATLAB_MEX_FILE) 
#define MDL_SET_OUTPUT_PORT_SAMPLE_TIME
void mdlSetOutputPortSampleTime(SimStruct *S, int_T port, real_T sampleTime, real_T offsetTime)
/* ========================================================================*/
{
	WHERE_AM_I(S, "");
    ssSetOutputPortSampleTime(S, port, sampleTime);
}
#endif 


/* ========================================================================*/
#if defined(MATLAB_MEX_FILE) 
#define MDL_SET_INPUT_PORT_DATA_TYPE
void mdlSetInputPortDataType(SimStruct *S, int_T port, DTypeId id)
/* ========================================================================*/
{
	WHERE_AM_I(S, "");
    
    if( type_translation.find(id) != type_translation.end() )
    {
         ssSetInputPortDataType(S, port, id);
    }
    else
    {
        ssSetErrorStatusf(S, "Inport data type not supported. Only int8, int16, signle or double.", "");
    }
}
#endif 


/* ========================================================================*/
#if defined(MATLAB_MEX_FILE) 
#define MDL_SET_DEFAULT_PORT_DIMENSION_INFO
void mdlSetDefaultPortDimensionInfo(SimStruct *S)
/* ========================================================================*/
{
	WHERE_AM_I(S, "");
    
    if (ssGetInputPortWidth(S, 0) == DYNAMICALLY_SIZED)     
        ssSetInputPortMatrixDimensions(S, 0, 368, 1 );
    
    if (ssGetInputPortWidth(S, 1) == DYNAMICALLY_SIZED)     
        ssSetInputPortMatrixDimensions(S, 1, 1, 1 );
    
}
#endif 


/* ========================================================================*/
#if defined(MATLAB_MEX_FILE) 
#define MDL_SET_INPUT_PORT_DIMENSION_INFO
void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
/* ========================================================================*/
{
	WHERE_AM_I(S, "");
    
    if ( dimsInfo->numDims == 1 || (dimsInfo->numDims == 2 && dimsInfo->dims[0] >= dimsInfo->dims[1] ) )
    {
        ssSetInputPortDimensionInfo(S, port, dimsInfo);
    }
    else
    {
        ssSetErrorStatusf(S, "count are frames, length are channels, need longer frames than number of channels", "");
    }        
}
#endif 


/* ========================================================================*/

static void mdlInitializeSampleTimes(SimStruct *S)
{    
    /* PORT_BASED_SAMPLE_TIMES */
}



/* ========================================================================*/
#define MDL_ENABLE
void mdlEnable(SimStruct * S)
/* ========================================================================*/
{
	WHERE_AM_I(S, "");
    /* Set SOB flag */
    uhd::tx_metadata_t *md = static_cast<uhd::tx_metadata_t*>(ssGetPWorkValue(S, METADATA)); 
    md->start_of_burst = true;   
}     

/* ========================================================================*/
#define MDL_DISABLE
void mdlDisable(SimStruct * S)
/* ========================================================================*/
{
	WHERE_AM_I(S, "");
    /* End current transmission */
    sendEOB(S); 
}  


/* ========================================================================*/
#define MDL_PROCESS_PARAMETERS  
static void mdlProcessParameters(SimStruct *S)
/* ========================================================================*/
{
	WHERE_AM_I(S, "");
    
    /* import vars */
    multi_usrp::sptr usrp = *static_cast<multi_usrp::sptr*>(ssGetPWorkValue(S, USRP)); 
    
    try
    {        
        const size_t num_chans = usrp->get_tx_num_channels();
        
        /* rf freq (center frequency) */
        param_setter<double>(usrp, &multi_usrp::set_tx_freq, ssGetSFcnParam(S, RF_FREQ), num_chans);

        /* rf gain */
        param_setter<double>(usrp, 
                static_cast< void (multi_usrp::*)(double, size_t)>(&multi_usrp::set_tx_gain), 
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
/* ========================================================================*/
{
	WHERE_AM_I(S, "");
   
    /* Set options of this Block */
    ssSetOptions(S, ssGetOptions(S) | SS_OPTION_CALL_TERMINATE_ON_EXIT);
    
    /* assign optional ports */
    int next_port = 1; /* 0 is data port */
    if( mxIsEmpty(ssGetSFcnParam(S,RF_FREQ))) 
        ssSetIWorkValue(S, RF_FREQ_PORT_INDEX, next_port++);
    if( mxIsEmpty(ssGetSFcnParam(S, GAIN))) 
        ssSetIWorkValue(S, GAIN_PORT_INDEX, next_port);
    
    /* set impossible values as last values */
    ssSetRWorkValue(S, LAST_RF_FREQ, -0.1);
    ssSetRWorkValue(S, LAST_GAIN, -1e7);
    /* connect and configure usrp */
    try
    {
        /* get usrp(s) */
        multi_usrp::sptr usrp = multi_usrp::make(string((getMxStringArray(ssGetSFcnParam(S, ARGS)))[0]));

        /* Create a copy of the sptr on the heap */ 
        ssSetPWorkValue(S, USRP, new multi_usrp::sptr( usrp )); 
            
        /******************************************************************
         * mboard config
         *****************************************************************/
        
        const size_t num_mboards = usrp->get_num_mboards();
                
        /* set clock source(s) */
        param_setter<string>(usrp, &multi_usrp::set_clock_source, ssGetSFcnParam(S, CLOCK_SRC), num_mboards);
        
        /* set time source(s) */
        param_setter<string>(usrp, &multi_usrp::set_time_source, ssGetSFcnParam(S, TIME_SRC), num_mboards);
        
        /* set subdev spec */
        param_setter<string>(usrp, &multi_usrp::set_tx_subdev_spec, ssGetSFcnParam(S, SUBDEV_SPEC), num_mboards);
                
        /* set sample rate */
        double rate = *static_cast<double*>(mxGetData(ssGetSFcnParam(S, SAMPLETIME)));
        if (rate == SAMPLETIME_INHERINT) 
            rate = ssGetInputPortDimensions(S, 0)[0] / ssGetInputPortSampleTime(S, 0);
        usrp->set_tx_rate(rate);
//         if ( usrp->get_tx_rate() != rate)
//            ssWarningf(S, "Could not set tx rate to %f MSps (became %f MSps)", 
//                 rate/1e6, usrp->get_tx_rate()/1e6
//            );
        
        /******************************************************************
         * channel config
         *****************************************************************/
        
        /* check if there is one stream per channel */
        const size_t num_chans = usrp->get_tx_num_channels();
        const size_t num_streams = (ssGetInputPortNumDimensions(S,0)>1) ? ssGetInputPortDimensions(S, 0)[1] : 1;
        if (num_chans != num_streams)
        {
            ssSetErrorStatusf(S, "Number of channels does not match USRP configuration", "");
            return;
        }
    
        /* set antenna port(s) */
        param_setter<string>(usrp, &multi_usrp::set_tx_antenna, ssGetSFcnParam(S, ANTENNA), num_chans);
        
        /* RF filter bandwidth */
        param_setter<double>(usrp, &multi_usrp::set_tx_bandwidth, ssGetSFcnParam(S, BANDWIDTH), num_chans);
                
        /* RF freq and gain */
        mdlProcessParameters(S);        
        
        /******************************************************************
         * streamer config
         *****************************************************************/
        
        /* Get and config stream io */   
        uhd::stream_args_t stream_args(type_translation[ (int)ssGetInputPortDataType(S,0) ], "sc16");
        for (size_t chan = 0; chan < num_streams; chan++)
            stream_args.channels.push_back(chan); //linear mapping
        
        ssSetPWorkValue(S, STREAMER, new uhd::tx_streamer::sptr( usrp->get_tx_stream( stream_args )));
        
        /* metadata struct */
        uhd::tx_metadata_t *md = new uhd::tx_metadata_t();
        md->start_of_burst = true;   
        ssSetPWorkValue(S, METADATA, md);
        
    }
    catch (uhd::exception &e)
    {
        ssSetErrorStatusf(S,"UHD: %s", e.what());
        return;
    }  
    
}


/* ========================================================================*/
#if defined(MATLAB_MEX_FILE) 
#define MDL_SIM_STATUS_CHANGE
static void mdlSimStatusChange(SimStruct *S, ssSimStatusChangeType simStatus)
/* ========================================================================*/
{
    WHERE_AM_I(S, "");
    
    if (simStatus == SIM_PAUSE) 
    { 
        /* End current transmission */
        sendEOB(S);
    } 
    else if (simStatus == SIM_CONTINUE) 
    { 
        /* Set SOB flag */
        uhd::tx_metadata_t *md = static_cast<uhd::tx_metadata_t*>(ssGetPWorkValue(S, METADATA)); 
        md->start_of_burst = true;   
    } 
} 
#endif 

/* ========================================================================*/
#define MDL_OUTPUTS
static void mdlOutputs(SimStruct *S, int_T tid)
/* ========================================================================*/
{
    /*WHERE_AM_I(S, ""); */
    
    /* import vars */
    multi_usrp::sptr usrp = *static_cast<multi_usrp::sptr*>(ssGetPWorkValue(S, USRP)); 
    uhd::tx_metadata_t *md = static_cast<uhd::tx_metadata_t*>(ssGetPWorkValue(S, METADATA)); 
    uhd::tx_streamer::sptr tx_stream = *static_cast<uhd::tx_streamer::sptr*>(ssGetPWorkValue(S, STREAMER));

    try
    {   
        /* RF freq */
        const int_T rf_freq_port_index = ssGetIWorkValue(S, RF_FREQ_PORT_INDEX);
        if(mxIsEmpty(ssGetSFcnParam(S,RF_FREQ)) && ssIsSampleHit(S, ssGetInputPortSampleTimeIndex(S, rf_freq_port_index), tid) )
        {  
            const real_T new_rf_freq = *ssGetInputPortRealSignalPtrs(S,rf_freq_port_index)[0];
            /* value changed? */
            if(ssGetRWorkValue(S, LAST_RF_FREQ) != new_rf_freq)
            {
                usrp->set_tx_freq(new_rf_freq);
                ssSetRWorkValue(S, LAST_RF_FREQ, new_rf_freq);
                // ssPrintf("RF freq = %g\n",new_rf_freq);
            }
        }
        
        /* gain */
        const int_T gain_port_index = ssGetIWorkValue(S, GAIN_PORT_INDEX);
        if(mxIsEmpty(ssGetSFcnParam(S,RF_FREQ)) && ssIsSampleHit(S, ssGetInputPortSampleTimeIndex(S, gain_port_index), tid) )
        {
            /* set new gain */
            const real_T new_gain = *ssGetInputPortRealSignalPtrs(S,gain_port_index)[0];
            if(ssGetRWorkValue(S, LAST_GAIN) != new_gain)
            {
                // ssPrintf("Gain = %g\n",new_gain);
                usrp->set_tx_gain(new_gain);
                ssSetRWorkValue(S, LAST_GAIN, new_gain);
            }
        }

        /* send first/next bunch of samples	*/
        if(ssIsSampleHit(S, ssGetInputPortSampleTimeIndex(S, 0), tid) )
        {             
            const size_t num_chans = tx_stream->get_num_channels();
            const int samples_per_buffer = ssGetInputPortDimensions(S, 0)[0];
            
            vector<const void *> rv;
            for(int chan=0; chan<num_chans; chan++)
                rv.push_back(ssGetInputPortSignalPtrs(S, 0)[chan * samples_per_buffer]);
                               
            /* send data */
            tx_stream->send(rv, samples_per_buffer, *md);
            
            /* now we're definitly in the middle of a burst */
            md->start_of_burst = false;
        }
    }
    catch (uhd::exception &e)
    {
        ssSetErrorStatusf(S,"UHD: %s", e.what());
    }    
}


/* ========================================================================*/
static void mdlTerminate(SimStruct *S)
/* ========================================================================*/
{
	WHERE_AM_I(S, "");
    
    /* check if usrp object has been created */
    if( ssGetPWorkValue(S, USRP) ) 
    {        
        /* end of burst */
        sendEOB(S);
        /* clear uhd memory */
        delete static_cast<multi_usrp::sptr*>(ssGetPWorkValue(S, USRP));
    }
    
    /* clear metadata_t memory */
    if( ssGetPWorkValue(S, METADATA) ) 
        delete static_cast<uhd::tx_metadata_t*>(ssGetPWorkValue(S, METADATA));  
    
    /* clear io_type_t memory */
    if( ssGetPWorkValue(S, STREAMER) ) 
        delete static_cast<uhd::tx_streamer::sptr*>(ssGetPWorkValue(S, STREAMER));  
    
    /* clear last rf freqs and gains memory */
    if( ssGetPWorkValue(S, LAST_RF_FREQS) ) 
        delete static_cast<dvec_sptr*>(ssGetPWorkValue(S, LAST_RF_FREQS));  
    if( ssGetPWorkValue(S, LAST_RF_GAINS) ) 
        delete static_cast<dvec_sptr*>(ssGetPWorkValue(S, LAST_RF_GAINS));  
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


