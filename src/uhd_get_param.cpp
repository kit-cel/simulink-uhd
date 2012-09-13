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

/* Get parameter(s) of the USRP */

#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <sstream>

/* Simulink */
#include <simstruc.h>
#include <tmwtypes.h>
#include <mex.h>

/* UHD */
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/subdev_spec.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/exception.hpp>
#include <uhd/device.hpp>
/* #define DEBUG */

using namespace std;
using namespace uhd::usrp;

static stringstream sstr_msg;

/* Utility functions - String operations */
int             param_has_args(string param);
int             param_get_num_args(string param);
vector<string>  param_get_args(string param,int num_args);
string          param_get_fcn_call(string param);


void print2Matlab(uhd::msg::type_t t, const std::string &msg)
{
    sstr_msg << msg;
};

/* Entry point to C/C++ */
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    int                 num_args;
    vector<string>      param_array;
    vector<string>      args;
    string              identifier;
    string              fcn_call;
    uhd::device_addr_t  device_addr;
    multi_usrp::sptr    usrp;
    uhd::device_addr_t  hint;
    
    typedef map<string, int> MapType;
    MapType fcn_call_map;
    
    /* USRP common */
    fcn_call_map.insert(std::pair<string, int>(string("master_clock_rate"),   1));
    fcn_call_map.insert(std::pair<string, int>(string("pp_string"),           2));
    fcn_call_map.insert(std::pair<string, int>(string("mboard_name"),         3));
    fcn_call_map.insert(std::pair<string, int>(string("num_mboards"),         4));    
    fcn_call_map.insert(std::pair<string, int>(string("time_synchronized"),   6));
    fcn_call_map.insert(std::pair<string, int>(string("sensor_info"),         12));
    
    /* RX */
    fcn_call_map.insert(std::pair<string, int>(string("rx_num_channels"),     5));
    fcn_call_map.insert(std::pair<string, int>(string("rx_rate"),             7));
    fcn_call_map.insert(std::pair<string, int>(string("rx_freq"),             8));
    fcn_call_map.insert(std::pair<string, int>(string("rx_freq_range"),       9));
    fcn_call_map.insert(std::pair<string, int>(string("rx_gain"),             10));
    fcn_call_map.insert(std::pair<string, int>(string("rx_antenna"),          13));
	fcn_call_map.insert(std::pair<string, int>(string("rx_antennas"),         14));
    fcn_call_map.insert(std::pair<string, int>(string("rx_subdev_spec"),      15));
    fcn_call_map.insert(std::pair<string, int>(string("rx_subdev_name"),      16));
    fcn_call_map.insert(std::pair<string, int>(string("rx_bandwidth"),        17));

    /* TX */
    fcn_call_map.insert(std::pair<string, int>(string("tx_num_channels"),     25));
    fcn_call_map.insert(std::pair<string, int>(string("tx_rate"),             26));
    fcn_call_map.insert(std::pair<string, int>(string("tx_freq"),             27));
    fcn_call_map.insert(std::pair<string, int>(string("tx_freq_range"),       28));
    fcn_call_map.insert(std::pair<string, int>(string("tx_gain"),             29));
    fcn_call_map.insert(std::pair<string, int>(string("tx_antenna"),          30));
	fcn_call_map.insert(std::pair<string, int>(string("tx_antennas"),         31));
    fcn_call_map.insert(std::pair<string, int>(string("tx_subdev_spec"),      32));
    fcn_call_map.insert(std::pair<string, int>(string("tx_subdev_name"),      33));
    fcn_call_map.insert(std::pair<string, int>(string("tx_bandwidth"),        34));
    
    /* */
    typedef map<const uhd::sensor_value_t::data_type_t, const DTypeId> DataType;
    DataType datatype_map;
    datatype_map.insert(std::pair<const uhd::sensor_value_t::data_type_t, const DTypeId>(uhd::sensor_value_t::BOOLEAN,SS_BOOLEAN));
    datatype_map.insert(std::pair<const uhd::sensor_value_t::data_type_t, const DTypeId>(uhd::sensor_value_t::INTEGER,SS_INT32));
    datatype_map.insert(std::pair<const uhd::sensor_value_t::data_type_t, const DTypeId>(uhd::sensor_value_t::REALNUM,SS_DOUBLE));
    datatype_map.insert(std::pair<const uhd::sensor_value_t::data_type_t, const DTypeId>(uhd::sensor_value_t::STRING,SS_DOUBLE));
    
       
    /* Register message handler */
    uhd::msg::register_handler(&print2Matlab);
    
    #ifdef DEBUG
        mexPrintf("mexFunction\n");
    #endif

	if (nrhs<2)
		mexErrMsgTxt(   "\nusage: []=uhd_get_param('identifier','param1',...,'paramN') \n\n"
                        "       identifier: IP-address of USRP device\n\n"
                        "       parameter list:\n\n"
                        "           master_clock_rate(mboard)\n"
                        "           pp_string\n"
                        "           mboard_name(mboard)\n"
                        "           num_mboards\n"
                        "           time_synchronized\n"
                        "           sensor_info\n"
                        "           rx_num_channels\n"
                        "           rx_rate(channel)\n"
                        "           rx_freq(channel)\n"
                        "           rx_freq_range(channel)\n"
                        "           rx_gain(name,channel)\n"
                        "           rx_antenna(channel)\n"
                        "           rx_antennas(channel)\n"
                        "           rx_subdev_spec(mboard)\n"
                        "           rx_subdev_name(channel)\n"
                        "           rx_bandwidth(channel)\n"
                        "           tx_num_channels\n"
                        "           tx_rate(channel)\n"
                        "           tx_freq(channel)\n"
                        "           tx_freq_range(channel)\n"
                        "           tx_gain(name,channel)\n"
                        "           tx_antenna(channel)\n"
                        "           tx_antennas(channel)\n"
                        "           tx_subdev_spec(mboard)\n"
                        "           tx_subdev_name(channel)\n"
                        "           tx_bandwidth(channel)\n\n"
                        "       Omit arguments (e.g. channel) to query all.\n\n"
                        "Example: [a b]=uhd_get_param('192.168.0.1','tx_subdev_name(0)','tx_bandwidth')"
                );
            
    /**********************************************************************
     **********************************************************************
     *  Get parameters
     **********************************************************************
     *********************************************************************/
    
    /* Get identifier */
    if(!mxIsChar(prhs[0])) {
            mexErrMsgTxt("Input must be of type char.");
    }
    identifier = mxArrayToString(prhs[0]);
    #ifdef DEBUG
        mexPrintf("identifier: %s\n",identifier.c_str());
	#endif
    
    int num_params=nrhs-1;
            
    for (int np=0; np<num_params; np++) {
        
        /* Check input to be sure it is of type char. */
        if(!mxIsChar(prhs[np+1])){
            mexErrMsgTxt("Input must be of type char.");
        }
        /* Copy the string data from prhs and place it into param_array. */ 
        param_array.push_back(mxArrayToString(prhs[np+1]));
        
        #ifdef DEBUG
            mexPrintf("param%d: %s\n",np,param_array[np].c_str());
        #endif
    }
    
    /**********************************************************************
     **********************************************************************
     *  Eval parameters
     **********************************************************************
     *********************************************************************/
    
    /* check if identifier contains "=" */
    if ((int)identifier.find("=") > 0)
        hint=identifier;
    /* .. if not, assume identifier is an IP address */
    else
        hint=string("addr=") + identifier;
    
    
    try {

        usrp = multi_usrp::make(hint);

        /*usrp->set_tx_subdev_spec((subdev_spec_t)"B:0",(size_t)0);*/
        
        #ifdef DEBUG
            mexPrintf("Creating USRP object.\n");
        #endif

        mwSize                          dims[2];
        double                          d_result;
        string                          s_result;
        vector<string>                  vs_result;
        vector<string>::const_iterator  iter;
        uhd::meta_range_t               r_result;
        uhd::usrp::subdev_spec_t        ss_result;

        const char                      *range_fields[] =           {"start", "stop", "step"};
        const char                      *antenna_fields[] =         {"channel","name"};
        const char                      *subdev_mboard_fields[] =   {"mboard","name"};
        const char                      *subdev_channel_fields[] =  {"channel","name"};
        const char                      *r_mboard_fields[] =        {"mboard","rate"};
        const char                      *r_channel_fields[] =       {"channel","rate"};
        const char                      *frequency_fields[] =       {"channel","frequency"};
        const char                      *bandwidth_fields[] =       {"channel","bandwidth"};
        const char                      *mb_fields[] =              {"mboard","name"};
        const char                      *gain_fields[] =            {"channel","name","gain","start", "stop", "step"};
        const char                      *sensor_fields[] =          {"identifier", "channel", "class", "name", "dTypeId"};

        int                             num_mboards =       (int)usrp->get_num_mboards();
        int                             num_tx_channels =   (int)usrp->get_tx_num_channels();
        int                             num_rx_channels =   (int)usrp->get_rx_num_channels();
        int                             num_structs;

        for (int np=0; np<num_params; np++) {

            #ifdef DEBUG
                mexPrintf("method %s has %d argument(s).\n",param_get_fcn_call(param_array[np]).c_str(),param_has_args(param_array[np]));
            #endif

            fcn_call=param_get_fcn_call(param_array[np]);
            num_args=0;

            if (param_has_args(param_array[np]) > 0) {

                num_args = param_get_num_args(param_array[np]);
                args = param_get_args(param_array[np],num_args);

                #ifdef DEBUG
                    mexPrintf("       |--> has %d argument(s): ",num_args);
                    for (int k=0;k<num_args;k++)
                        mexPrintf("%s, ",args[k].c_str());
                    mexPrintf("\n");
                #endif
            }

            #ifdef DEBUG
                mexPrintf("Number of args: %d\n",num_args);
                mexPrintf("Number of mboards: %d\n",num_mboards);
                mexPrintf("Number of tx-chan: %d\n",num_tx_channels);
                mexPrintf("Number of rx-chan: %d\n",num_rx_channels);
            #endif


            /******************************************************************
             *  Call subroutine for each parameter request
             *****************************************************************/
            switch (fcn_call_map.find(fcn_call)->second) {

                /**************************************************************
                 * master_clock_rate
                 * Get the master clock rate.
                 * virtual double get_master_clock_rate (size_t mboard=0)=0
                 *************************************************************/
                case 1:
                    /* specific mboard, given by first argument*/
                    if (num_args>0) {

                        d_result = usrp->get_master_clock_rate((size_t)atoi(args[0].c_str()));
                        plhs[np]=mxCreateDoubleScalar(d_result);
                    }

                    /* all mboards, returns struct */
                    else {

                        num_structs=num_mboards;
                        dims[0]=1; dims[1]=num_structs;

                        plhs[np]=mxCreateStructArray(2, dims, 2 , r_mboard_fields);

                        num_structs=0;

                        /* loop for all mboards */
                        for (int num_mboard=0;num_mboard<num_mboards;num_mboard++) {

                            d_result = usrp->get_master_clock_rate((size_t)num_mboard);

                            mxSetField(plhs[np],num_structs,"mboard",mxCreateDoubleScalar(num_mboard));
                            mxSetField(plhs[np],num_structs,"rate",mxCreateDoubleScalar(d_result));

                            num_structs++;          
                        }
                    }
                    break;

                /**************************************************************
                 * pp_string
                 * Get a printable summary for this USRP configuration.
                 * virtual std::string get_pp_string (void)=0
                 *************************************************************/    
                case 2:
                    s_result = usrp->get_pp_string();
                    plhs[np]=mxCreateString(s_result.c_str());
                    break;

                /**************************************************************
                 * mboard_name
                 * Get canonical name for this USRP motherboard.
                 * virtual std::string get_mboard_name (size_t mboard=0)=0
                 *************************************************************/    
                case 3:
                    /* specific mboard, given by first argument*/
                    if (num_args>0) {

                        s_result = usrp->get_mboard_name((size_t)atoi(args[0].c_str()));
                        plhs[np]=mxCreateString(s_result.c_str());
                    }

                    /* all mboards, returns struct */
                    else {

                        num_structs=num_mboards;
                        dims[0]=1; dims[1]=num_structs;

                        plhs[np]=mxCreateStructArray(2, dims, 2 , mb_fields);

                        num_structs=0;

                        /* loop for all mboards */
                        for (int num_mboard=0;num_mboard<num_mboards;num_mboard++) {

                            s_result = usrp->get_mboard_name((size_t)num_mboard);

                            mxSetField(plhs[np],num_structs,"mboard",mxCreateDoubleScalar(num_mboard));
                            mxSetField(plhs[np],num_structs,"name",mxCreateString(s_result.c_str()));

                            num_structs++;          
                        }
                    }
                    break;

                /**************************************************************
                 * num_mboards
                 * Get the number of USRP motherboards in this configuration.
                 * virtual size_t get_num_mboards (void)=0
                 *************************************************************/      
                case 4:
                    d_result = (double)usrp->get_num_mboards();
                    plhs[np]=mxCreateDoubleScalar(d_result);
                    break;

                /**************************************************************
                 * rx_num_channels
                 * Get the number of RX channels in this configuration. This is
                 * the number of USRPs times the number of RX channels per
                 * board, where the number of RX channels per board is 
                 * homogeneous among all USRPs. 
                 * virtual size_t get_rx_num_channels (void)=0
                 *************************************************************/     
                case 5:
                    d_result = (double)usrp->get_rx_num_channels();
                    plhs[np]=mxCreateDoubleScalar(d_result);
                    break;

                /**************************************************************
                 * time_synchronized
                 * Are the times across all motherboards in this configuration
                 * synchronized? Checks that all time registers are
                 * approximately close but not exact, given that the RTT may
                 * varying for a control packet transaction. 
                 * virtual bool get_time_synchronized (void)=0
                 *************************************************************/ 
                case 6:
                    d_result = (double)usrp->get_time_synchronized();
                    plhs[np]=mxCreateDoubleScalar(d_result);
                    break;

                /**************************************************************
                 * rx_rate
                 * Gets the RX sample rate.
                 * virtual double get_rx_rate (size_t chan=0)=0
                 *************************************************************/    
                case 7:
                    /* specific channel, given by first argument*/
                    if (num_args>0) {
                        d_result = usrp->get_rx_rate((size_t)atoi(args[0].c_str()));
                        plhs[np]=mxCreateDoubleScalar(d_result);
                    }

                    /* all channels, returns struct */
                    else {

                        num_structs=num_rx_channels;
                        dims[0]=1; dims[1]=num_structs;

                        plhs[np]=mxCreateStructArray(2, dims, 2 , r_channel_fields);

                        num_structs=0;

                        /* loop for all channels */
                        for (int num_channel=0;num_channel<num_rx_channels;num_channel++) {

                            d_result = usrp->get_rx_rate((size_t)num_channel);

                            mxSetField(plhs[np],num_structs,"channel",mxCreateDoubleScalar(num_channel));
                            mxSetField(plhs[np],num_structs,"rate",mxCreateDoubleScalar(d_result));

                            num_structs++;
                        }                  
                    }
                    break;

                /**************************************************************
                 * rx_freq
                 * Get the RX center frequency.
                 * virtual double get_rx_freq (size_t chan=0)=0
                 *************************************************************/ 
                case 8:
                    /* specific channel, given by first argument*/
                    if (num_args>0) {

                        try {                     
                            d_result = usrp->get_rx_freq((size_t)atoi(args[0].c_str()));
                        }
                        catch (uhd::runtime_error) {
                            d_result = -1;
                        }

                        plhs[np]=mxCreateDoubleScalar(d_result);

                    }

                    /* all channels, returns struct */
                    else {

                        num_structs=num_rx_channels;
                        dims[0]=1; dims[1]=num_structs;

                        plhs[np]=mxCreateStructArray(2, dims, 2 , frequency_fields);

                        num_structs=0;

                        /* loop for all channels */
                        for (int num_channel=0;num_channel<num_rx_channels;num_channel++) {

                            try {                     
                                d_result = usrp->get_rx_freq((size_t)num_channel);
                            }
                            catch (uhd::runtime_error) {
                                d_result = -1;
                            }

                            mxSetField(plhs[np],num_structs,"channel",mxCreateDoubleScalar(num_channel));
                            mxSetField(plhs[np],num_structs,"frequency",mxCreateDoubleScalar(d_result));

                            num_structs++;
                        }                  
                    }
                    break;

                /**************************************************************
                 * rx_freq_range
                 * Get the RX center frequency range.
                 * virtual freq_range_t get_rx_freq_range (size_t chan=0)=0
                 *************************************************************/ 
                case 9:
                    /* specific channel, given by first argument*/
                    if (num_args>0) {
                        r_result = usrp->get_rx_freq_range((size_t)atoi(args[0].c_str()));

                        dims[0]=1; dims[1]=1;

                        plhs[np]=mxCreateStructArray(2, dims, 3 , range_fields);
                        mxSetField(plhs[np],0,"start",mxCreateDoubleScalar(r_result.start()));
                        mxSetField(plhs[np],0,"stop",mxCreateDoubleScalar(r_result.stop()));
                        mxSetField(plhs[np],0,"step",mxCreateDoubleScalar(r_result.step()));
                    }

                    /* all channels, returns struct */
                    else {

                        num_structs=num_rx_channels;
                        dims[0]=1; dims[1]=num_structs;

                        plhs[np]=mxCreateStructArray(2, dims, 3 , range_fields);

                        num_structs=0;

                        /* loop for all channels */
                        for (int num_channel=0;num_channel<num_rx_channels;num_channel++) {

                            r_result = usrp->get_rx_freq_range((size_t)num_channel);

                            mxSetField(plhs[np],num_structs,"start",mxCreateDoubleScalar(r_result.start()));
                            mxSetField(plhs[np],num_structs,"stop",mxCreateDoubleScalar(r_result.stop()));
                            mxSetField(plhs[np],num_structs,"step",mxCreateDoubleScalar(r_result.step()));

                            num_structs++;
                        }                  
                    }
                    break;                

                /**************************************************************
                 * rx_gain
                 * Get the RX gain and ranges
                 * virtual double get_rx_gain (const std::string &name, size_t chan=0)=0
                 *************************************************************/
                case 10:

                    /* get rx gain information only for a specific channel */
                    if (num_args==1) {
                        d_result = usrp->get_rx_gain((size_t)atoi(args[0].c_str()));                    
                        plhs[np]=mxCreateDoubleScalar(d_result);
                    }

                    /* get rx gain information for a specific channel and gain element (name) */
                    else if (num_args>1) {
                        d_result = usrp->get_rx_gain(args[0].c_str(),(size_t)atoi(args[1].c_str()));
                        plhs[np]=mxCreateDoubleScalar(d_result);
                    }

                    /* get all rx gain information */
                    else {

                        num_structs=0;

                        /* get number of structures */
                        for (int num_channel=0;num_channel<num_rx_channels;num_channel++)
                            num_structs += (int)(usrp->get_rx_gain_names(num_channel)).size();

                        dims[0]=1; dims[1]=num_structs;
                        plhs[np]=mxCreateStructArray(2, dims, 6 , gain_fields);
                        num_structs=0;

                        /* loop for all channels */
                        for (int num_channel=0;num_channel<num_rx_channels;num_channel++) {

                            vs_result = usrp->get_rx_gain_names(num_channel);

                            /* loop for all gain elements (names) at specific channel */
                            for (iter=vs_result.begin(); iter!=vs_result.end();++iter) {

                                r_result = usrp->get_rx_gain_range(*iter,(size_t)num_channel);
                                d_result = usrp->get_rx_gain(*iter,(size_t)num_channel);

                                /* fill output */
                                mxSetField(plhs[np],num_structs,"channel",mxCreateDoubleScalar(num_channel));
                                mxSetField(plhs[np],num_structs,"name",mxCreateString((*iter).c_str()));
                                mxSetField(plhs[np],num_structs,"gain",mxCreateDoubleScalar(d_result));                    
                                mxSetField(plhs[np],num_structs,"start",mxCreateDoubleScalar(r_result.start())); 
                                mxSetField(plhs[np],num_structs,"stop",mxCreateDoubleScalar(r_result.stop()));
                                mxSetField(plhs[np],num_structs,"step",mxCreateDoubleScalar(r_result.step()));

                                num_structs++;
                            }

                        }
                    }
                    break;

                /**************************************************************
                 * sensor_info
                 * A convenience wrapper for getting overall sensor information
                 *************************************************************/
                case 12:

                    num_structs=0;

                    /* get number of structures */
                    for (int num_mboard=0;num_mboard<num_mboards;num_mboard++)                    
                        num_structs += (int)(usrp->get_mboard_sensor_names((size_t)num_mboard)).size();
                    for (int num_channel=0;num_channel<num_tx_channels;num_channel++)                    
                        num_structs += (int)(usrp->get_tx_sensor_names((size_t)num_channel)).size();
                    for (int num_channel=0;num_channel<num_rx_channels;num_channel++)                    
                        num_structs += (int)(usrp->get_rx_sensor_names((size_t)num_channel)).size();

                    dims[0]=1; dims[1]=num_structs;                
                    plhs[np]=mxCreateStructArray(2, dims, 5 , sensor_fields);

                    num_structs=0;

                    /* for all mboard sensors do.. */
                    for (int num_mboard=0;num_mboard<num_mboards;num_mboard++) {

                        vs_result = usrp->get_mboard_sensor_names((size_t)num_mboard);

                        /* for all parameters of ONE mboard sensor do.. */
                        for (iter=vs_result.begin(); iter!=vs_result.end();++iter) {

                            uhd::sensor_value_t sensor_value = usrp->get_mboard_sensor(*iter,(size_t)num_mboard);

                            mxSetField(plhs[np],num_structs,"identifier",mxCreateString(identifier.c_str()));
                            mxSetField(plhs[np],num_structs,"channel",mxCreateDoubleScalar(num_mboard));
                            mxSetField(plhs[np],num_structs,"class",mxCreateDoubleScalar(0));                    
                            mxSetField(plhs[np],num_structs,"name",mxCreateString((*iter).c_str())); 

                            if (datatype_map.find(sensor_value.type) != datatype_map.end())
                                mxSetField(plhs[np],num_structs,"dTypeId",mxCreateDoubleScalar( (datatype_map.find(sensor_value.type)->seco nd) ));
                            else
                                mxSetField(plhs[np],num_structs,"dTypeId",mxCreateDoubleScalar( 0 ));

                            num_structs++;
                        }
                    }

                    /* for all tx sensors do.. */
                    for (int num_channel=0;num_channel<num_tx_channels;num_channel++) {

                        vs_result = usrp->get_tx_sensor_names((size_t)num_channel);

                        /* for all parameters of ONE tx sensor do.. */
                        for (iter=vs_result.begin(); iter!=vs_result.end();++iter) {

                            uhd::sensor_value_t sensor_value = usrp->get_tx_sensor(*iter,(size_t)num_channel);

                            mxSetField(plhs[np],num_structs,"identifier",mxCreateString(identifier.c_str()));
                            mxSetField(plhs[np],num_structs,"channel",mxCreateDoubleScalar(num_channel));
                            mxSetField(plhs[np],num_structs,"class",mxCreateDoubleScalar(1));                    
                            mxSetField(plhs[np],num_structs,"name",mxCreateString((*iter).c_str())); 

                            if (datatype_map.find(sensor_value.type) != datatype_map.end())
                                mxSetField(plhs[np],num_structs,"dTypeId",mxCreateDoubleScalar( (datatype_map.find(sensor_value.type)->second) ));
                            else
                                mxSetField(plhs[np],num_structs,"dTypeId",mxCreateDoubleScalar( 0 ));

                            num_structs++;
                        }
                    }

                    /* for all rx sensors do.. */
                    for (int num_channel=0;num_channel<num_rx_channels;num_channel++) {

                        vs_result = usrp->get_rx_sensor_names((size_t)num_channel);

                        /* for all parameters of ONE rx sensor do.. */
                        for (iter=vs_result.begin(); iter!=vs_result.end();++iter) {

                            uhd::sensor_value_t sensor_value = usrp->get_rx_sensor(*iter,(size_t)num_channel);

                            mxSetField(plhs[np],num_structs,"identifier",mxCreateString(identifier.c_str()));
                            mxSetField(plhs[np],num_structs,"channel",mxCreateDoubleScalar(num_channel));
                            mxSetField(plhs[np],num_structs,"class",mxCreateDoubleScalar(2));                    
                            mxSetField(plhs[np],num_structs,"name",mxCreateString((*iter).c_str())); 

                            if (datatype_map.find(sensor_value.type) != datatype_map.end())
                                mxSetField(plhs[np],num_structs,"dTypeId",mxCreateDoubleScalar( (datatype_map.find(sensor_value.type)->second) ));
                            else
                                mxSetField(plhs[np],num_structs,"dTypeId",mxCreateDoubleScalar( 0 ));

                            num_structs++;
                        }
                    }                
                    break;

                /**************************************************************
                 * rx_antenna
                 * Get the selected RX antenna on the subdevice.
                 * virtual std::string get_rx_antenna (size_t chan=0)=0
                 *************************************************************/
                case 13:                
                    /* specific channel, given by first argument*/
                    if (num_args==1) {
                        s_result = usrp->get_rx_antenna((size_t)atoi(args[0].c_str()));                    
                        plhs[np]=mxCreateString(s_result.c_str());
                    }

                    /* all channels, returns struct */
                    else {

                        num_structs=num_rx_channels;
                        dims[0]=1; dims[1]=num_structs;

                        plhs[np]=mxCreateStructArray(2, dims, 2 , antenna_fields);

                        num_structs=0;

                        /* loop for all channels */
                        for (int num_channel=0;num_channel<num_rx_channels;num_channel++) {

                            s_result = usrp->get_rx_antenna((size_t)num_channel);

                            mxSetField(plhs[np],num_structs,"channel",mxCreateDoubleScalar(num_channel));
                            mxSetField(plhs[np],num_structs,"name",mxCreateString(s_result.c_str()));

                            num_structs++;
                        }                  
                    }                
                    break;

                /**************************************************************
                 * rx_antennas
                 * Get a list of possible RX antennas on the subdevice. 
                 * virtual std::vector< std::string > get_rx_antennas (size_t chan=0)=0
                 *************************************************************/
                case 14:                
                    /* specific channel, given by first argument*/
                    if (num_args>1) {

                        vs_result = usrp->get_rx_antennas((size_t)atoi(args[0].c_str()));

                        num_structs=(int)vs_result.size();
                        dims[0]=1; dims[1]=num_structs;

                        plhs[np]=mxCreateStructArray(2, dims, 2 , antenna_fields);

                        num_structs=0;

                        /* loop for all available antennas on specific channel */
                        for (iter=vs_result.begin(); iter!=vs_result.end();++iter) {

                            mxSetField(plhs[np],num_structs,"channel",mxCreateDoubleScalar(atoi(args[0].c_str())));
                            mxSetField(plhs[np],num_structs,"name",mxCreateString((*iter).c_str()));

                            num_structs++;
                        }

                    }

                    /* all channels, returns struct */
                    else {

                        num_structs=0;

                        /* get number of structures */
                        for (int num_channel=0;num_channel<num_rx_channels;num_channel++)
                            num_structs += (int)(usrp->get_rx_antennas((size_t)num_channel)).size();

                        dims[0]=1; dims[1]=num_structs;
                        plhs[np]=mxCreateStructArray(2, dims, 2 , antenna_fields);

                        num_structs=0;

                        /* loop for all channels */
                        for (int num_channel=0;num_channel<num_rx_channels;num_channel++) {

                            vs_result = usrp->get_rx_antennas((size_t)num_channel);

                            /* loop for all available antennas on specific channel */
                            for (iter=vs_result.begin(); iter!=vs_result.end();++iter) {

                                mxSetField(plhs[np],num_structs,"channel",mxCreateDoubleScalar(num_channel));
                                mxSetField(plhs[np],num_structs,"name",mxCreateString((*iter).c_str()));

                                num_structs++;
                            }
                        }
                    }                
                    break;

                /**************************************************************
                 * rx_subdev_spec
                 * Get the RX subdevice specification.
                 * virtual uhd::usrp::subdev_spec_t get_rx_subdev_spec (size_t mboard=0)=0
                 *************************************************************/
                case 15:
                    /* specific mboard, given by first argument*/
                    if (num_args>0) {

                        ss_result = usrp->get_rx_subdev_spec((size_t)atoi(args[0].c_str()));
                        plhs[np]=mxCreateString((ss_result.to_string()).c_str());
                    }

                    /* all mboards, returns struct */
                    else {

                        num_structs=num_mboards;
                        dims[0]=1; dims[1]=num_structs;

                        plhs[np]=mxCreateStructArray(2, dims, 2 , subdev_mboard_fields);

                        num_structs=0;

                        /* loop for all mboards */
                        for (int num_mboard=0;num_mboard<num_mboards;num_mboard++) {

                            ss_result = usrp->get_rx_subdev_spec((size_t)num_mboard);

                            mxSetField(plhs[np],num_structs,"mboard",mxCreateDoubleScalar(num_mboard));
                            mxSetField(plhs[np],num_structs,"name",mxCreateString((ss_result.to_string()).c_str()));

                            num_structs++;          
                        }
                    }
                    break;

                /**************************************************************
                 * rx_subdev_name
                 * Get the name of the RX subdevice.
                 * virtual std::string get_rx_subdev_name (size_t chan=0)=0
                 *************************************************************/
                case 16:
                    /* specific channel, given by first argument*/
                    if (num_args>0) {

                        s_result = usrp->get_rx_subdev_name((size_t)atoi(args[0].c_str()));
                        plhs[np]=mxCreateString(s_result.c_str());
                    }

                    /* all channels, returns struct */
                    else {

                        num_structs=num_rx_channels;
                        dims[0]=1; dims[1]=num_structs;

                        plhs[np]=mxCreateStructArray(2, dims, 2 , subdev_channel_fields);

                        num_structs=0;

                        /* loop for all channels */
                        for (int num_channel=0;num_channel<num_rx_channels;num_channel++) {

                            s_result = usrp->get_rx_subdev_name((size_t)num_channel);

                            mxSetField(plhs[np],num_structs,"channel",mxCreateDoubleScalar(num_channel));
                            mxSetField(plhs[np],num_structs,"name",mxCreateString(s_result.c_str()));

                            num_structs++;          
                        }
                    }
                    break;

                /**************************************************************
                 * rx_bandwidth
                 * Get the RX bandwidth on the subdevice.
                 * virtual double get_rx_bandwidth (size_t chan=0)=0
                 *************************************************************/
                case 17:
                    /* specific channel, given by first argument*/
                    if (num_args>0) {

                        d_result = usrp->get_rx_bandwidth((size_t)atoi(args[0].c_str()));
                        plhs[np]=mxCreateDoubleScalar(d_result);
                    }

                    /* all channels, returns struct */
                    else {

                        num_structs=num_rx_channels;
                        dims[0]=1; dims[1]=num_structs;

                        plhs[np]=mxCreateStructArray(2, dims, 2 , bandwidth_fields);

                        num_structs=0;

                        /* loop for all channels */
                        for (int num_channel=0;num_channel<num_rx_channels;num_channel++) {

                            d_result = usrp->get_rx_bandwidth((size_t)num_channel);

                            mxSetField(plhs[np],num_structs,"channel",mxCreateDoubleScalar(num_channel));
                            mxSetField(plhs[np],num_structs,"bandwidth",mxCreateDoubleScalar(d_result));

                            num_structs++;          
                        }
                    }
                    break;

                /**************************************************************
                 * tx_num_channels
                 * Get the number of TX channels in this configuration. This is
                 * the number of USRPs times the number of TX channels per
                 * board, where the number of TX channels per board is
                 * homogeneous among all USRPs.  
                 * virtual size_t get_tx_num_channels (void)=0
                 *************************************************************/     
                case 25:
                    d_result = (double)usrp->get_tx_num_channels();
                    plhs[np]=mxCreateDoubleScalar(d_result);
                    break;

                /**************************************************************
                 * tx_rate
                 * Gets the TX sample rate.
                 * virtual double get_tx_rate (size_t chan=0)
                 *************************************************************/    
                case 26:
                    /* specific channel, given by first argument*/
                    if (num_args>0) {
                        d_result = usrp->get_tx_rate((size_t)atoi(args[0].c_str()));
                        plhs[np]=mxCreateDoubleScalar(d_result);
                    }

                    /* all channels, returns struct */
                    else {

                        num_structs=num_tx_channels;
                        dims[0]=1; dims[1]=num_structs;

                        plhs[np]=mxCreateStructArray(2, dims, 2 , r_channel_fields);

                        num_structs=0;

                        /* loop for all channels */
                        for (int num_channel=0;num_channel<num_tx_channels;num_channel++) {

                            d_result = usrp->get_tx_rate((size_t)num_channel);

                            mxSetField(plhs[np],num_structs,"channel",mxCreateDoubleScalar(num_channel));
                            mxSetField(plhs[np],num_structs,"rate",mxCreateDoubleScalar(d_result));

                            num_structs++;
                        }                  
                    }
                    break;

                /**************************************************************
                 * tx_freq
                 * Get the TX center frequency.
                 * virtual double get_tx_freq (size_t chan=0)=0
                 *************************************************************/ 
                case 27:
                    /* specific channel, given by first argument*/
                    if (num_args>0) {

                        try {                     
                            d_result = usrp->get_rx_freq((size_t)atoi(args[0].c_str()));
                        }
                        catch (uhd::runtime_error) {
                            d_result = -1;
                        }                    

                        plhs[np]=mxCreateDoubleScalar(d_result);
                    }

                    /* all channels, returns struct */
                    else {

                        num_structs=num_tx_channels;
                        dims[0]=1; dims[1]=num_structs;

                        plhs[np]=mxCreateStructArray(2, dims, 2 , frequency_fields);

                        num_structs=0;

                        /* loop for all channels */
                        for (int num_channel=0;num_channel<num_tx_channels;num_channel++) {

                            try {                     
                                d_result = usrp->get_tx_freq((size_t)num_channel);
                            }
                            catch (uhd::runtime_error) {
                                d_result = -1;
                            }

                            mxSetField(plhs[np],num_structs,"channel",mxCreateDoubleScalar(num_channel));
                            mxSetField(plhs[np],num_structs,"frequency",mxCreateDoubleScalar(d_result));

                            num_structs++;
                        }                  
                    }
                    break;

                /**************************************************************
                 * tx_freq_range
                 * Get the TX center frequency range.
                 * virtual freq_range_t get_tx_freq_range (size_t chan=0)=0
                 *************************************************************/ 
                case 28:
                    /* specific channel, given by first argument*/
                    if (num_args>0) {
                        r_result = usrp->get_tx_freq_range((size_t)atoi(args[0].c_str()));

                        dims[0]=1; dims[1]=1;

                        plhs[np]=mxCreateStructArray(2, dims, 3 , range_fields);
                        mxSetField(plhs[np],0,"start",mxCreateDoubleScalar(r_result.start()));
                        mxSetField(plhs[np],0,"stop",mxCreateDoubleScalar(r_result.stop()));
                        mxSetField(plhs[np],0,"step",mxCreateDoubleScalar(r_result.step()));
                    }

                    /* all channels, returns struct */
                    else {

                        num_structs=num_tx_channels;
                        dims[0]=1; dims[1]=num_structs;

                        plhs[np]=mxCreateStructArray(2, dims, 3 , range_fields);

                        num_structs=0;

                        /* loop for all channels */
                        for (int num_channel=0;num_channel<num_tx_channels;num_channel++) {

                            r_result = usrp->get_tx_freq_range((size_t)num_channel);

                            mxSetField(plhs[np],num_structs,"start",mxCreateDoubleScalar(r_result.start()));
                            mxSetField(plhs[np],num_structs,"stop",mxCreateDoubleScalar(r_result.stop()));
                            mxSetField(plhs[np],num_structs,"step",mxCreateDoubleScalar(r_result.step()));

                            num_structs++;
                        }                  
                    }
                    break;

                /**************************************************************
                 * tx_gain
                 * Get the TX gain and ranges
                 * virtual double get_tx_gain (const std::string &name, size_t chan=0)=0
                 *************************************************************/
                case 29:

                    /* get tx gain information only for a specific channel */
                    if (num_args==1) {
                        d_result = usrp->get_tx_gain((size_t)atoi(args[0].c_str()));                    
                        plhs[np]=mxCreateDoubleScalar(d_result);
                    }

                    /* get tx gain information for a specific channel and gain element (name) */
                    else if (num_args>1) {
                        d_result = usrp->get_tx_gain(args[0].c_str(),(size_t)atoi(args[1].c_str()));
                        plhs[np]=mxCreateDoubleScalar(d_result);
                    }

                    /* get all tx gain information */
                    else {

                        num_structs=0;

                        /* get number of structures */
                        for (int num_channel=0;num_channel<num_tx_channels;num_channel++)
                            num_structs += (int)(usrp->get_tx_gain_names(num_channel)).size();

                        dims[0]=1; dims[1]=num_structs;
                        plhs[np]=mxCreateStructArray(2, dims, 6 , gain_fields);
                        num_structs=0;

                        /* loop for all channels */
                        for (int num_channel=0;num_channel<num_tx_channels;num_channel++) {

                            vs_result = usrp->get_tx_gain_names(num_channel);

                            /* loop for all gain elements (names) at specific channel */
                            for (iter=vs_result.begin(); iter!=vs_result.end();++iter) {

                                r_result = usrp->get_tx_gain_range(*iter,(size_t)num_channel);
                                d_result = usrp->get_tx_gain(*iter,(size_t)num_channel);

                                /* fill output */
                                mxSetField(plhs[np],num_structs,"channel",mxCreateDoubleScalar(num_channel));
                                mxSetField(plhs[np],num_structs,"name",mxCreateString((*iter).c_str()));
                                mxSetField(plhs[np],num_structs,"gain",mxCreateDoubleScalar(d_result));                    
                                mxSetField(plhs[np],num_structs,"start",mxCreateDoubleScalar(r_result.start())); 
                                mxSetField(plhs[np],num_structs,"stop",mxCreateDoubleScalar(r_result.stop()));
                                mxSetField(plhs[np],num_structs,"step",mxCreateDoubleScalar(r_result.step()));

                                num_structs++;
                            }

                        }
                    }
                    break;

                /**************************************************************
                 * tx_antenna
                 * Get the selected TX antenna on the subdevice.
                 * virtual std::string get_tx_antenna (size_t chan=0)=0
                 *************************************************************/
                case 30:                
                    /* specific channel, given by first argument*/
                    if (num_args==1) {
                        s_result = usrp->get_tx_antenna((size_t)atoi(args[0].c_str()));                    
                        plhs[np]=mxCreateString(s_result.c_str());
                    }

                    /* all channels, returns struct */
                    else {

                        num_structs=num_tx_channels;
                        dims[0]=1; dims[1]=num_structs;

                        plhs[np]=mxCreateStructArray(2, dims, 2 , antenna_fields);

                        num_structs=0;

                        /* loop for all channels */
                        for (int num_channel=0;num_channel<num_tx_channels;num_channel++) {

                            s_result = usrp->get_tx_antenna((size_t)num_channel);

                            mxSetField(plhs[np],num_structs,"channel",mxCreateDoubleScalar(num_channel));
                            mxSetField(plhs[np],num_structs,"name",mxCreateString(s_result.c_str()));

                            num_structs++;
                        }                  
                    }                
                    break;

                /**************************************************************
                 * tx_antennas
                 * Get a list of possible TX antennas on the subdevice. 
                 * virtual std::vector< std::string > get_tx_antennas (size_t chan=0)=0
                 *************************************************************/
                case 31:                
                    /* specific channel, given by first argument*/
                    if (num_args>1) {

                        vs_result = usrp->get_tx_antennas((size_t)atoi(args[0].c_str()));

                        num_structs=(int)vs_result.size();
                        dims[0]=1; dims[1]=num_structs;

                        plhs[np]=mxCreateStructArray(2, dims, 2 , antenna_fields);

                        num_structs=0;

                        /* loop for all available antennas on specific channel */
                        for (iter=vs_result.begin(); iter!=vs_result.end();++iter) {

                            mxSetField(plhs[np],num_structs,"channel",mxCreateDoubleScalar(atoi(args[0].c_str())));
                            mxSetField(plhs[np],num_structs,"name",mxCreateString((*iter).c_str()));

                            num_structs++;
                        }

                    }

                    /* all channels, returns struct */
                    else {

                        num_structs=0;

                        /* get number of structures */
                        for (int num_channel=0;num_channel<num_tx_channels;num_channel++)
                            num_structs += (int)(usrp->get_tx_antennas((size_t)num_channel)).size();

                        dims[0]=1; dims[1]=num_structs;
                        plhs[np]=mxCreateStructArray(2, dims, 2 , antenna_fields);

                        num_structs=0;

                        /* loop for all channels */
                        for (int num_channel=0;num_channel<num_tx_channels;num_channel++) {

                            vs_result = usrp->get_tx_antennas((size_t)num_channel);

                            /* loop for all available antennas on specific channel */
                            for (iter=vs_result.begin(); iter!=vs_result.end();++iter) {

                                mxSetField(plhs[np],num_structs,"channel",mxCreateDoubleScalar(num_channel));
                                mxSetField(plhs[np],num_structs,"name",mxCreateString((*iter).c_str()));

                                num_structs++;
                            }
                        }
                    }                
                    break;

                /**************************************************************
                 * tx_subdev_spec
                 * Get the TX subdevice specification.
                 * virtual uhd::usrp::subdev_spec_t get_tx_subdev_spec (size_t mboard=0)=0
                 *************************************************************/
                case 32:
                    /* specific mboard, given by first argument*/
                    if (num_args>0) {

                        ss_result = usrp->get_tx_subdev_spec((size_t)atoi(args[0].c_str()));
                        plhs[np]=mxCreateString((ss_result.to_string()).c_str());
                    }

                    /* all mboards, returns struct */
                    else {

                        num_structs=(int)num_mboards;
                        dims[0]=1; dims[1]=num_structs;

                        plhs[np]=mxCreateStructArray(2, dims, 2 , subdev_mboard_fields);

                        num_structs=0;

                        /* loop for all mboards */
                        for (int num_mboard=0;num_mboard<num_mboards;num_mboard++) {

                            ss_result = usrp->get_tx_subdev_spec((size_t)num_mboard);

                            mxSetField(plhs[np],num_structs,"mboard",mxCreateDoubleScalar(num_mboard));
                            mxSetField(plhs[np],num_structs,"name",mxCreateString((ss_result.to_string()).c_str()));

                            num_structs++;          
                        }
                    }
                    break;

                /**************************************************************
                 * tx_subdev_name
                 * Get the name of the TX subdevice.
                 * virtual std::string get_tx_subdev_name (size_t chan=0)=0
                 *************************************************************/
                case 33:
                    /* specific channel, given by first argument*/
                    if (num_args>0) {

                        s_result = usrp->get_tx_subdev_name((size_t)atoi(args[0].c_str()));
                        plhs[np]=mxCreateString(s_result.c_str());
                    }

                    /* all channels, returns struct */
                    else {

                        num_structs=num_tx_channels;
                        dims[0]=1; dims[1]=num_structs;

                        plhs[np]=mxCreateStructArray(2, dims, 2 , subdev_channel_fields);

                        num_structs=0;

                        /* loop for all channels */
                        for (int num_channel=0;num_channel<num_tx_channels;num_channel++) {

                            s_result = usrp->get_tx_subdev_name((size_t)num_channel);

                            mxSetField(plhs[np],num_structs,"channel",mxCreateDoubleScalar(num_channel));
                            mxSetField(plhs[np],num_structs,"name",mxCreateString(s_result.c_str()));

                            num_structs++;          
                        }
                    }
                    break;

                /**************************************************************
                 * tx_bandwidth
                 * Get the TX bandwidth on the subdevice.
                 * virtual double get_tx_bandwidth (size_t chan=0)
                 *************************************************************/
                case 34:
                    /* specific channel, given by first argument*/
                    if (num_args>0) {

                        d_result = usrp->get_tx_bandwidth((size_t)atoi(args[0].c_str()));
                        plhs[np]=mxCreateDoubleScalar(d_result);
                    }

                    /* all channels, returns struct */
                    else {

                        num_structs=num_tx_channels;
                        dims[0]=1; dims[1]=num_structs;

                        plhs[np]=mxCreateStructArray(2, dims, 2 , bandwidth_fields);

                        num_structs=0;

                        /* loop for all channels */
                        for (int num_channel=0;num_channel<num_tx_channels;num_channel++) {

                            d_result = usrp->get_tx_bandwidth((size_t)num_channel);

                            mxSetField(plhs[np],num_structs,"channel",mxCreateDoubleScalar(num_channel));
                            mxSetField(plhs[np],num_structs,"bandwidth",mxCreateDoubleScalar(d_result));

                            num_structs++;          
                        }
                    }
                    break;

                default:
                    mexPrintf("not found\n");
            }      
        }

    }
    catch (uhd::exception &e) {

        mexPrintf("%s\n",sstr_msg.str().c_str());
        mexErrMsgTxt(e.what());
    }
    
}

/**************************************************************************
 **************************************************************************
 *  Utility functions - String operations
 **************************************************************************
 *************************************************************************/
int param_has_args(string param)
{
    /* Check if ( and ) exist */
    if (param.find("(") == -1 || param.find(")") == -1)
        return 0;
    /* Check if arg is given between ( and )*/
    else if ((param.find(")") - param.find("(")) ==1)
        return 0;
    /* Check if ( and ) are twisted */
    else if (((int)param.find(")") - (int)param.find("(")) <0)
        return 0;
    else
        return 1;
}

vector<string> param_get_args(string param,int num_args)
{
    int start = (int)param.find("(")+1;
    int length = (int)param.find(")")-start;
    int k=(int)0;
    
    if (num_args == 0)
        mexErrMsgTxt("param_get_args --> No arguments");    
    
    vector<string> arg_list;
    
    string args = param.substr(start,length);
    
    stringstream stream(args);
    string arg;

    while( getline(stream, arg, ',') ) {
        arg_list.push_back(arg);        
        k++;
    }
    
    return arg_list;
}

int param_get_num_args(string param)
{
    int start = (int)param.find("(")+1;
    int length = (int)param.find(")")-start;
    int num_args = (int)0;
    
    string args = param.substr(start,length);
    
    stringstream stream(args);
    string arg;

    while( getline(stream, arg, ',') ) {
        num_args++;
    }
    
    if (num_args == 0)
        mexErrMsgTxt("param_get_num_args --> No arguments");
    
    return num_args;
}

string param_get_fcn_call(string param)
{
    string fcn_call;
    
    if (param_has_args(param)) {
        
        int length = (int)param.find("(");
        fcn_call = param.substr(0,length);
    }
    else
        fcn_call = param;
    
    return fcn_call;
}