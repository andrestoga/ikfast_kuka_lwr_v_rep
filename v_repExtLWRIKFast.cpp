#include "v_repExtLWRIKFast.h"
#include "luaFunctionData.h"
#include "v_repLib.h"
#include <iostream>

#include <fstream>
#include <stdio.h>
#include <stdlib.h>

#include "IKFastKinematicsPlugin.h"

#ifdef _WIN32
    #ifdef QT_COMPIL
        #include <direct.h>
    #else
        #include <shlwapi.h>
        #pragma comment(lib, "Shlwapi.lib")
    #endif
#endif /* _WIN32 */

#if defined (__linux) || defined (__APPLE__)
        #include <unistd.h>
#endif /* __linux || __APPLE__ */

#ifdef __APPLE__
    #define _stricmp strcmp
#endif

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)    CONCAT(x,y,z)

#define PLUGIN_VERSION 2 // 2 since version 3.2.1

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

#define LUA_LWRIKFAST_COMMAND "simExtIWRIKFast" // the name of the new Lua command

const int inArgs_LWR_IKFAST_INPUT[]={ // Decide what kind of arguments we need
1, // we want 1 input arguments
sim_lua_arg_string,0, // first argument is a string
/*sim_lua_arg_float|sim_lua_arg_table,3,*/ // second argument should be a table of at least 3 float values (use 0 instead of 3 for a table of random size)
/*sim_lua_arg_int|sim_lua_arg_table,2,*/ // third argument should be a table of at least 2 integer values (use 0 instead of 2 for a table of random size)
};

int ReadInputIKFast(const char * path, Eigen::Affine3d &pose)
{
    double trans[3];
    // double quaternion[4];
    std::ifstream infile;
    infile.open(path);

    if (!infile.is_open())
    {
        return -1;
    }

    double a, b, c;
    // int i = 0;

    Eigen::Matrix3d m;

    infile >> a >> b >> c;

    trans[0] = a;
    trans[1] = b;
    trans[2] = c;

    for (int i = 0; i < 3; ++i)
    {
        infile >> a >> b >> c;

        m(i, 0) = a;
        m(i, 1) = b;
        m(i, 2) = c;
    }

    Eigen::Quaterniond q(m);

    pose = Eigen::Translation3d(trans[0], trans[1], trans[2])* q;

    infile.close();

    return 0;
}

void ikToFile( const char * path, std::vector< double > &solution)
{
    std::ofstream ikToFile(path, std::ofstream::out);

    // vector< vector<double> >::iterator row;
    // vector<double>::iterator col;

    // for (row = ikSolutions.begin(); row != ikSolutions.end(); row++)
    // {
    //     for (col = row->begin(); col != row->end(); col++)
    //     {
    //         ikToFile << *col << " ";
    //     }

    //     ikToFile << "\n";
    // }

    for (int i = 0; i < solution.size(); ++i)
    {
        ikToFile << solution[i] << "\n";
    }

    ikToFile.close();
}

void LUA_LWR_IKFAST_INPUT_CALLBACK(SLuaCallBack* p)
{ 
        // the callback function of the new Lua command ("simExtSkeleton_getSensorData")
    p->outputArgCount=0;
    CLuaFunctionData D;
        // If successful the command will return an interger (result), a float table of size 3 (data), and a float (distance). If the command is not successful, it will not return anything
        // bool commandWasSuccessful=false;
        // int returnResult;
        // std::vector<float> returnData;
        // float returnDistance;

    if (D.readDataFromLua(p,inArgs_LWR_IKFAST_INPUT,inArgs_LWR_IKFAST_INPUT[0],LUA_LWRIKFAST_COMMAND))
    { 
        std::vector<std::vector<double> > ik_solutions;
        std::vector<double> solution;
        Eigen::Affine3d target_pose;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
        const char * path = inData->at(0).stringData[0].c_str();

        ReadInputIKFast(path, target_pose);  

        IKFastKinematicsPlugin temp;

        temp.ComputeIkKukaLWR(target_pose, ik_solutions, solution);

        std::cout << "Number of ik solutions: " << ik_solutions.size() << std::endl;
        std::cout << "The best solution:" << std::endl;

        for (int i = 0; i < solution.size(); ++i)
        {
            std::cout << solution[i] << std::endl;
        }

        if (solution.size())
        {
            ikToFile(path, solution);
        }

            /*int sensorIndex=inData->at(0).intData[0]; // the first argument
            std::vector<float>& floatParameters=inData->at(1).floatData; // the second argument
            std::vector<int>& intParameters=inData->at(2).intData; // the third argument

            // Now you can do something with above's arguments. For example:
            if ((sensorIndex>=0)&&(sensorIndex<10))
            {
                commandWasSuccessful=true;
                returnResult=1;
                returnData.push_back(1.0f);
                returnData.push_back(2.0f);
                returnData.push_back(3.0f);
                returnDistance=59.0f;
            }
            else
                simSetLastError(LUA_GETSENSORDATA_COMMAND,"Invalid sensor index."); // output an error message to the simulator's status bar*/
    }

        // if (commandWasSuccessful)
        // { // prepare the return values:
        //     D.pushOutData(CLuaFunctionDataItem(returnResult));
        //     D.pushOutData(CLuaFunctionDataItem(returnData));
        //     D.pushOutData(CLuaFunctionDataItem(returnDistance));
        // }

        // D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// This is the plugin start routine (called just once, just after the plugin was loaded):
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{
        // Dynamically load and bind V-REP functions:
        // ******************************************
        // 1. Figure out this plugin's directory:
    char curDirAndFile[1024];
        #ifdef _WIN32
            #ifdef QT_COMPIL
    _getcwd(curDirAndFile, sizeof(curDirAndFile));
            #else
    GetModuleFileName(NULL,curDirAndFile,1023);
    PathRemoveFileSpec(curDirAndFile);
            #endif
        #elif defined (__linux) || defined (__APPLE__)
    getcwd(curDirAndFile, sizeof(curDirAndFile));
        #endif

    std::string currentDirAndPath(curDirAndFile);
        // 2. Append the V-REP library's name:
    std::string temp(currentDirAndPath);
        #ifdef _WIN32
    temp+="\\v_rep.dll";
        #elif defined (__linux)
    temp+="/libv_rep.so";
        #elif defined (__APPLE__)
    temp+="/libv_rep.dylib";
        #endif /* __linux || __APPLE__ */
            // 3. Load the V-REP library:
    vrepLib=loadVrepLibrary(temp.c_str());
    if (vrepLib==NULL)
    {
        std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
            return(0); // Means error, V-REP will unload this plugin
        }
        if (getVrepProcAddresses(vrepLib)==0)
        {
            std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
            unloadVrepLibrary(vrepLib);
            return(0); // Means error, V-REP will unload this plugin
        }
        // ******************************************

        // Check the version of V-REP:
        // ******************************************
        int vrepVer;
        simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
        if (vrepVer<30200) // if V-REP version is smaller than 3.02.00
        {
            std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'PluginSkeleton' plugin.\n";
            unloadVrepLibrary(vrepLib);
            return(0); // Means error, V-REP will unload this plugin
        }
        // ******************************************

        std::vector<int> inArgs;

        // Register the new Lua command "simExtSkeleton_getSensorData":
        CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_LWR_IKFAST_INPUT,inArgs);
        simRegisterCustomLuaFunction(LUA_LWRIKFAST_COMMAND,strConCat("number result,table data,number distance=",LUA_LWRIKFAST_COMMAND,"(number sensorIndex,table_3 floatParameters,table_2 intParameters)"),&inArgs[0],LUA_LWR_IKFAST_INPUT_CALLBACK);

        return(PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when V-REP is ending, i.e. releasing this plugin):
VREP_DLLEXPORT void v_repEnd()
{
    // Here you could handle various clean-up tasks

    unloadVrepLibrary(vrepLib); // release the library
}

// This is the plugin messaging routine (i.e. V-REP calls this function very often, with various messages):
VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ 
    // This is called quite often. Just watch out for messages/events you want to handle
    // Keep following 5 lines at the beginning and unchanged:
    static bool refreshDlgFlag=true;
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
    void* retVal=NULL;

    // Here we can intercept many messages from V-REP (actually callbacks). Only the most important messages are listed here.
    // For a complete list of messages that you can intercept/react with, search for "sim_message_eventcallback"-type constants
    // in the V-REP user manual.

    if (message==sim_message_eventcallback_refreshdialogs)
        refreshDlgFlag=true; // V-REP dialogs were refreshed. Maybe a good idea to refresh this plugin's dialog too

    if (message==sim_message_eventcallback_menuitemselected)
    { // A custom menu bar entry was selected..
        // here you could make a plugin's main dialog visible/invisible
    }

    if (message==sim_message_eventcallback_instancepass)
    {   // This message is sent each time the scene was rendered (well, shortly after) (very often)
        // It is important to always correctly react to events in V-REP. This message is the most convenient way to do so:

        int flags=auxiliaryData[0];
        bool sceneContentChanged=((flags&(1+2+4+8+16+32+64+256))!=0); // object erased, created, model or scene loaded, und/redo called, instance switched, or object scaled since last sim_message_eventcallback_instancepass message 
        bool instanceSwitched=((flags&64)!=0);

        if (instanceSwitched)
        {
            // React to an instance switch here!!
        }

        if (sceneContentChanged)
        { // we actualize plugin objects for changes in the scene

            //...

            refreshDlgFlag=true; // always a good idea to trigger a refresh of this plugin's dialog here
        }
    }

    if (message==sim_message_eventcallback_mainscriptabouttobecalled)
    { // The main script is about to be run (only called while a simulation is running (and not paused!))

    }

    if (message==sim_message_eventcallback_simulationabouttostart)
    { // Simulation is about to start

    }

    if (message==sim_message_eventcallback_simulationended)
    { // Simulation just ended

    }

    if (message==sim_message_eventcallback_moduleopen)
    { // A script called simOpenModule (by default the main script). Is only called during simulation.
        // if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
        // {
        //  // we arrive here only at the beginning of a simulation
        // }
    }

    if (message==sim_message_eventcallback_modulehandle)
    { // A script called simHandleModule (by default the main script). Is only called during simulation.
        // if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
        // {
        //  // we arrive here only while a simulation is running
        // }
    }

    if (message==sim_message_eventcallback_moduleclose)
    { // A script called simCloseModule (by default the main script). Is only called during simulation.
        // if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
        // {
        //  // we arrive here only at the end of a simulation
        // }
    }

    if (message==sim_message_eventcallback_instanceswitch)
    { // We switched to a different scene. Such a switch can only happen while simulation is not running

    }

    if (message==sim_message_eventcallback_broadcast)
    { // Here we have a plugin that is broadcasting data (the broadcaster will also receive this data!)

    }

    if (message==sim_message_eventcallback_scenesave)
    { // The scene is about to be saved. If required do some processing here (e.g. add custom scene data to be serialized with the scene)

    }

    // You can add many more messages to handle here

    if ((message==sim_message_eventcallback_guipass)&&refreshDlgFlag)
    { // handle refresh of the plugin's dialogs
        // ...
refreshDlgFlag=false;
}

    // Keep following unchanged:
    simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
    return(retVal);
}