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

/* Get property tree of UHD device */

/* C++ Includes */
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
#include <uhd/property_tree.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/subdev_spec.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/exception.hpp>
#include <uhd/device.hpp>
#include <uhd/usrp/dboard_eeprom.hpp>
#include <uhd/usrp/mboard_eeprom.hpp>

/* #define DEBUG */

using namespace std;
using namespace uhd;

static stringstream sstr_msg;

/* ================================================================= */
/* helper functions
/* ================================================================= */

static string vector_to_string(const vector<string> &prop_names);
static string get_freq_range(const uhd::fs_path &path, uhd::property_tree::sptr tree);
static string get_gain_range(const uhd::fs_path &path, uhd::property_tree::sptr tree);
/* --- only for debug purposes ---
void print_tree(const uhd::fs_path &path, uhd::property_tree::sptr tree); */


void print2Matlab(uhd::msg::type_t t, const std::string &msg)
{
    sstr_msg << msg;
};

/* Entry point to C/C++ */
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    string              identifier;
    uhd::device_addr_t  device_addr;
    device::sptr        usrp;
    uhd::device_addr_t  hint;
    vector<string>      txrx; txrx.push_back("tx"); txrx.push_back("rx");
    
    /* data type translation map*/
    typedef map<const uhd::sensor_value_t::data_type_t, const string> DataType;
    DataType datatype_map;
    datatype_map.insert(std::pair<const uhd::sensor_value_t::data_type_t, const string>(uhd::sensor_value_t::BOOLEAN,string("BOOLEAN")));
    datatype_map.insert(std::pair<const uhd::sensor_value_t::data_type_t, const string>(uhd::sensor_value_t::INTEGER,string("INT32")));
    datatype_map.insert(std::pair<const uhd::sensor_value_t::data_type_t, const string>(uhd::sensor_value_t::REALNUM,string("DOUBLE")));
    datatype_map.insert(std::pair<const uhd::sensor_value_t::data_type_t, const string>(uhd::sensor_value_t::STRING,string("DOUBLE")));
    
    /* ================================================================= */
    /* check input and ouput
    /* ================================================================= */
    
    /* Register message handler */
    uhd::msg::register_handler(&print2Matlab);
    
    #ifdef DEBUG
        mexPrintf("mexFunction\n");
    #endif
            
    if (nrhs == 1 && mxIsChar(prhs[0])) {
        /* Get identifier */
        identifier = mxArrayToString(prhs[0]);
    }
    else {
		mexErrMsgTxt(   "\nUsage: []=uhd_get_tree(identifier) \n\n"
                        "       This function will query the UHD device parameters and provide them as a list of key/value pairs.\n\n"
                        "       identifier: A string that specifies the unique identifier of the UHD device.\n\n"
                        "       Possible identifiers are:\n\n"
                        "\t     IP address (addr), e.g. addr=192.168.55.3\n"
                        "\t     Serial number (serial), e.g. serial=EGR18WFRP\n"
                        "\t     Device Type (type), e.g. type=usrp2\n"
                        "\t     Device Name (name), e.g. name=myUSRP\n\n"
                        "       In the case of an IP Address, the hint addr= can be omitted.\n\n"
                        "Example: a=uhd_get_tree('serial=EGR18WFRP')\n\n"
                );
    }
    
    #ifdef DEBUG
        mexPrintf("identifier: %s\n",identifier.c_str());
	#endif
    
    
    /* check if identifier contains "=" */
    if ((int)identifier.find("=") > 0)
        hint=identifier;
    /* .. if not, assume identifier is an IP address */
    else
        hint=string("addr=") + identifier;
    
    
    try {

        /* ============================================================= */
        /* get property tree
        /* ============================================================= */
        
        /* make usrp */
        usrp = device::make(hint);
        
        /* get property tree from usrp */
        property_tree::sptr tree = usrp->get_tree();
        
        /* ============================================================= */
        /* read out properties
        /* ============================================================= */
        
        /* create string vector pair key and value that captures the
         * path (key) and the value (val) */
        vector<string>      key;
        vector<string>      val;
        
        /* name of the usrp */
        key.push_back("/name");
        val.push_back(tree->access<string>(fs_path("/name")).get());
        
        string cur_path;
        
        /* query mboards */
        BOOST_FOREACH(const string &mboard, tree->list("/mboards")) {
            
            string root_path = "/mboards/" + mboard;
            
            /* name of the mboard */
            cur_path = root_path + "/name";
            key.push_back(cur_path);
            val.push_back(tree->access<string>(fs_path(cur_path)).get());
            
            /* firmware version */
            cur_path = root_path + "/fw_version";
            key.push_back(cur_path);
            val.push_back(tree->access<string>(fs_path(cur_path)).get());
            
            /* fpga version */
            cur_path = root_path + "/fpga_version";
            key.push_back(cur_path);
            val.push_back(tree->access<string>(fs_path(cur_path)).get());
            
            /* query eeprom on mboard */
            cur_path = root_path + "/eeprom";
            usrp::mboard_eeprom_t mb_eeprom = tree->access<usrp::mboard_eeprom_t>(cur_path).get();
            BOOST_FOREACH(const string &prop, mb_eeprom.keys()){
                if (not mb_eeprom[prop].empty())  {
                    key.push_back(cur_path + "/" + prop);
                    val.push_back(mb_eeprom[prop]);
                }
            }
            
            /* available time sources */
            cur_path = root_path + "/time_source/options";
            key.push_back(cur_path);
            val.push_back(vector_to_string(tree->access<vector<string> >(fs_path(cur_path)).get()));
            
            /* available clock sources */
            cur_path = root_path + "/clock_source/options";
            key.push_back(cur_path);
            val.push_back(vector_to_string(tree->access<vector<string> >(fs_path(cur_path)).get()));
            
            /* query sensors */
            cur_path = root_path + "/sensors";
            BOOST_FOREACH(const string &sensor, tree->list(cur_path)){
                
                sensor_value_t sensor_value = tree->access<sensor_value_t>(fs_path(cur_path + "/" + sensor)).get();
                
                key.push_back(cur_path + "/" + sensor + "/name");
                val.push_back(sensor_value.name);
                
                key.push_back(cur_path + "/" + sensor + "/value");
                val.push_back(sensor_value.value);
                
                key.push_back(cur_path + "/" + sensor + "/unit");
                val.push_back(sensor_value.unit);
                
                key.push_back(cur_path + "/" + sensor + "/type");
                val.push_back(datatype_map.find(sensor_value.type)->second);
            }
            
            /* query dsps */
            BOOST_FOREACH(const string &dir, txrx){
                
                cur_path = root_path + "/" + dir + "_dsps";
                BOOST_FOREACH(const string &dsp, tree->list(cur_path)){
                    
                    key.push_back(cur_path + "/" + dsp + "/freq/range");
                    val.push_back(get_freq_range(cur_path + "/" + dsp + "/freq/range",tree));
                }
            }
            
            /* query codecs */
            BOOST_FOREACH(const string &dir, txrx){
                
                cur_path = root_path + "/" + dir + "_codecs";
                BOOST_FOREACH(const string &cod, tree->list(cur_path)){

                    key.push_back(cur_path + "/" + cod + "/name");
                    val.push_back(tree->access<string>(fs_path(cur_path + "/" + cod + "/name")).get());
                    
                    BOOST_FOREACH(const string &gain, tree->list(cur_path + "/" + cod + "/gains")){
                    
                        key.push_back(cur_path + "/" + cod + "/gains/" + gain + "/range");
                        val.push_back(get_gain_range(cur_path + "/" + cod + "/gains/" + gain + "/range",tree));
                    }
                }
            }
            
            /* query dboards */
            root_path = root_path + "/dboards";
            BOOST_FOREACH(const string &dboard, tree->list(root_path)) {
                
                cur_path = root_path + "/" + dboard;            
                    
                /* query rx eeprom */
                usrp::dboard_eeprom_t db_eeprom = tree->access<usrp::dboard_eeprom_t>(fs_path(cur_path + "/rx_eeprom")).get();
                if (db_eeprom.id != usrp::dboard_id_t::none())  {
                    key.push_back(cur_path + "/rx_eeprom/id");
                    val.push_back(db_eeprom.id.to_pp_string());
                }
                if (not db_eeprom.serial.empty()) {
                    key.push_back(cur_path + "/rx_eeprom/serial");
                    val.push_back(db_eeprom.serial);
                }
                
                /* query tx eeprom */
                db_eeprom = tree->access<usrp::dboard_eeprom_t>(fs_path(cur_path + "/tx_eeprom")).get();
                if (db_eeprom.id != usrp::dboard_id_t::none())  {
                    key.push_back(cur_path + "/tx_eeprom/id");
                    val.push_back(db_eeprom.id.to_pp_string());
                }
                if (not db_eeprom.serial.empty()) {
                    key.push_back(cur_path + "/tx_eeprom/serial");
                    val.push_back(db_eeprom.serial);
                }
                
                /* query gdb eeprom on dboard */
                db_eeprom = tree->access<usrp::dboard_eeprom_t>(fs_path(cur_path + "/gdb_eeprom")).get();
                if (db_eeprom.id != usrp::dboard_id_t::none())  {
                    key.push_back(cur_path + "/gdb_eeprom/id");
                    val.push_back(db_eeprom.id.to_pp_string());
                }
                if (not db_eeprom.serial.empty()) {
                    key.push_back(cur_path + "/gdb_eeprom/serial");
                    val.push_back(db_eeprom.serial);
                }
                                
                /* query frontends (subdev) on dboard */
                BOOST_FOREACH(const string &dir, txrx){
                     
                    root_path = "/mboards/" + mboard + "/dboards/" + dboard + "/" + dir + "_frontends";
                    BOOST_FOREACH(const string &frontend, tree->list(root_path)) {

                        /* name of the subdev */
                        cur_path = root_path + "/" + frontend + "/name";
                        key.push_back(cur_path);
                        val.push_back(tree->access<string>(fs_path(cur_path)).get());

                        /* query sensors on subdev*/
                        cur_path = root_path + "/" + frontend + "/sensors";
                        BOOST_FOREACH(const string &sensor, tree->list(cur_path)){

                            sensor_value_t sensor_value = tree->access<sensor_value_t>(fs_path(cur_path + "/" + sensor)).get();
                
                            key.push_back(cur_path + "/" + sensor + "/name");
                            val.push_back(sensor_value.name);

                            key.push_back(cur_path + "/" + sensor + "/value");
                            val.push_back(sensor_value.value);

                            key.push_back(cur_path + "/" + sensor + "/unit");
                            val.push_back(sensor_value.unit);

                            key.push_back(cur_path + "/" + sensor + "/type");
                            val.push_back(datatype_map.find(sensor_value.type)->second);                            
                        }

                        /* query gains on subdev */
                        cur_path = root_path + "/" + frontend + "/gains";
                        BOOST_FOREACH(const string &gain, tree->list(cur_path)){

                            key.push_back(cur_path + "/" + gain + "/range");
                            val.push_back(get_gain_range(cur_path + "/" + gain + "/range",tree));
                        }

                        /* frequency range of subdev */
                        cur_path = root_path + "/" + frontend + "/freq";
                        key.push_back(cur_path + "/range");
                        val.push_back(get_freq_range(cur_path + "/range",tree));

                        /* available sensors */
                        cur_path = root_path + "/" + frontend + "/antenna/options";
                        key.push_back(cur_path);
                        val.push_back(vector_to_string(tree->access<vector<string> >(fs_path(cur_path)).get()));

                        /* connection type */
                        cur_path = root_path + "/" + frontend + "/connection";
                        key.push_back(cur_path);
                        val.push_back(tree->access<string>(fs_path(cur_path)).get());

                        /* use local oscillator offset */
                        cur_path = root_path + "/" + frontend + "/use_lo_offset";
                        key.push_back(cur_path);
                        val.push_back(tree->access<bool>(fs_path(cur_path)).get() ? "Yes" : "No");

                        /* subdev enabled? */
                        cur_path = root_path + "/" + frontend + "/enabled";
                        key.push_back(cur_path);
                        val.push_back(tree->access<bool>(fs_path(cur_path)).get() ? "Yes" : "No");

                        /* bandwidth of subdev */
                        cur_path = root_path + "/" + frontend + "/bandwidth/value";
                        key.push_back(cur_path);
                        val.push_back( boost::str(boost::format("%.3f MHz") % ( tree->access<double>(fs_path(cur_path)).get()/1e6 )));
                    }
                }               
            }
        }
        
        /* ============================================================= */
        /* export key vaulue pairs
        /* ============================================================= */
                
        int num_keys=key.size();
        mxArray *output = mxCreateCellMatrix(num_keys, 2);
        for (int l=0; l<num_keys; l++) {
            
            mxSetCell(output, l, mxCreateString(key[l].c_str()));
            mxSetCell(output, num_keys+l, mxCreateString(val[l].c_str()));
        }
        
        plhs[0]=output;

    }
    catch (uhd::exception &e) {

        mexPrintf("%s\n",sstr_msg.str().c_str());
        mexErrMsgTxt(e.what());
    }
    
}

/* ===================================================================== */
/* helper functions, definition section
/* ===================================================================== */

/* get printable gain range from uhd meta data type */
static string get_gain_range(const uhd::fs_path &path, uhd::property_tree::sptr tree)
{
    
    if (tree->exists(path)) {
        
        meta_range_t range=tree->access<meta_range_t>(path).get();
        return boost::str(boost::format("%.1f dB to %.1f dB, step %.1f dB") % range.start() % range.stop() % range.step());
    }
    else 
        return string("None");
}

/* get printable frequency range from uhd meta data type */
static string get_freq_range(const uhd::fs_path &path, uhd::property_tree::sptr tree)
{

    if (tree->exists(path)) {
        
        meta_range_t range=tree->access<meta_range_t>(path).get();
        return boost::str(boost::format("%.3f MHz to %.3f MHz") % (range.start()/1e6) % (range.stop()/1e6));
    }
    else 
        return string("None");
}

/* concatenate a vector of strings to a single string delimeted by , */
static string vector_to_string(const vector<string> &prop_names)
{
    stringstream ss; size_t count=0;
    BOOST_FOREACH(const string &prop_name, prop_names) {
        
        ss << ((count++)? ", " : "") << prop_name;
    }
    return ss.str();
}

/* --- only for debug purposes ---
void print_tree(const uhd::fs_path &path, uhd::property_tree::sptr tree)
{
 
    mexPrintf("%s\n",((string)path).c_str());
    BOOST_FOREACH(const string &name, tree->list(path)){
        print_tree(path / name, tree);
    }
}*/