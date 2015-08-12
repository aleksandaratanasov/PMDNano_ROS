/*****************************************************************************
 * PMDSDK 2
 * 
 * Copyright (c) 2006-2012 PMD Technologies GmbH
 * All Rights Reserved.
 *
 * Author: Martin Profittlich
 *
 *****************************************************************************/
#pragma once
#include "pmddatadescription.h"

extern "C" {

typedef unsigned PMDHandle;
int pmdOpen (PMDHandle * hnd, const char * rplugin, const char * rparam, const char * pplugin, const char * pparam);int pmdOpenSourcePlugin (PMDHandle * hnd, const char * rplugin, const char * rparam);int pmdClose (PMDHandle hnd);int pmdCloseAll ();int pmdGetLastError (PMDHandle hnd, char * error, size_t maxLen);int pmdUpdate (PMDHandle hnd);int pmdSetIntegrationTime (PMDHandle hnd, unsigned idx, unsigned t);int pmdGetIntegrationTime (PMDHandle hnd, unsigned * t, unsigned idx);int pmdGetValidIntegrationTime (PMDHandle hnd, unsigned * result, unsigned idx, Proximity w, unsigned t);int pmdSetModulationFrequency (PMDHandle hnd, unsigned idx, unsigned f);int pmdGetModulationFrequency (PMDHandle hnd, unsigned * f, unsigned idx);int pmdGetValidModulationFrequency (PMDHandle hnd, unsigned * result, unsigned idx, Proximity w, unsigned f);int pmdGetSourceData (PMDHandle hnd, void * data, size_t maxLen);int pmdGetSourceDataSize (PMDHandle hnd, size_t * size);int pmdGetSourceDataDescription (PMDHandle hnd, struct PMDDataDescription * dd);int pmdGetDistances (PMDHandle hnd, float * data, size_t maxLen);int pmdGet3DCoordinates (PMDHandle hnd, float * data, size_t maxLen);int pmdGetAmplitudes (PMDHandle hnd, float * data, size_t maxLen);int pmdGetIntensities (PMDHandle hnd, float * data, size_t maxLen);int pmdGetFlags (PMDHandle hnd, unsigned * data, size_t maxLen);int pmdSourceCommand (PMDHandle hnd, char * result, size_t maxLen, const char * cmd);int pmdProcessingCommand (PMDHandle hnd, char * result, size_t maxLen, const char * cmd);int pmdOpenProcessingPlugin (PMDHandle * hnd, const char * pplugin, const char * pparam);int pmdProcess (PMDHandle hnd, 
        unsigned numOut, struct PMDDataDescription * ddOut, void ** output, size_t * maxLen, unsigned * type, 
        unsigned numIn, struct PMDDataDescription * ddIn, void ** input);int pmdGetProcessedData (PMDHandle hnd, void * dataPtr, size_t maxLen, unsigned type);int pmdCanProcess (PMDHandle hnd, unsigned * memNeeded, unsigned type, unsigned numFmt, struct PMDDataDescription * fmt);int pmdCalcDistances (PMDHandle hnd, float * data, size_t maxLen, struct PMDDataDescription sourceDD, void * sourceData);int pmdCalcAmplitudes (PMDHandle hnd, float * data, size_t maxLen, struct PMDDataDescription sourceDD, void * sourceData);int pmdCalcIntensities (PMDHandle hnd, float * data, size_t maxLen, struct PMDDataDescription sourceDD, void * sourceData);int pmdCalc3DCoordinates (PMDHandle hnd, float * data, size_t maxLen, struct PMDDataDescription sourceDD, void * sourceData);int pmdCalcFlags (PMDHandle hnd, unsigned * data, size_t maxLen, struct PMDDataDescription sourceDD, void * sourceData);int pmdOpenOutputPlugin (PMDHandle * hnd, const char * oplugin, const char * oparam);int pmdOutput (PMDHandle hnd, unsigned numIn, struct PMDDataDescription * ddIn, void ** input);int pmdCanOutput (PMDHandle hnd, unsigned numFmt, struct PMDDataDescription * fmt);int pmdOutputCommand (PMDHandle hnd, char * result, size_t maxLen, const char * cmd);int pmdConnectFireWire (PMDHandle * hnd, unsigned index);int pmdConnectASample (PMDHandle * hnd, const char * address);

#ifndef PMD_NO_DEPRECATED
int pmdConnect (PMDHandle * hnd, const char * rplugin, const char * rparam, const char * pplugin, const char * pparam);int pmdConnectOnlyRaw (PMDHandle * hnd, const char * rplugin, const char * rparam);int pmdDisconnect (PMDHandle hnd);int pmdOpenAccessPlugin (PMDHandle * hnd, const char * rplugin, const char * rparam);int pmdGetRawData (PMDHandle hnd, void * data, size_t maxLen);int pmdGetRawDataSize (PMDHandle hnd, size_t * size);int pmdGetRawDataDescription (PMDHandle hnd, struct PMDDataDescription * dd);int pmdPlatformCommand (PMDHandle hnd, char * result, size_t maxLen, const char * cmd);int pmdConfigureProcess (PMDHandle hnd, char * result, size_t maxLen, const char * cmd);int pmdGetInfo (PMDHandle hnd, char * result, size_t maxLen, const char * key);

#endif 

}
