/*
 A template for a basic MSP external object with more explicit documentation
 
 See Readme of repository for build instructions.
 Create an Issue on the repository if anything is amiss or you have any suggestion
 - mhamilt Mar 2020
 
  For the variable delay, we need to speed up ordown the rate of delay if it is greater or less than the amount of delay
 delay steps should be floating numbers and we need to interpolate between samples.
 */
#include "ext.h"
#include "ext_obex.h"
#include "z_dsp.h"

double getInterpOut(double floatIndex, double* audioBuffer, unsigned int buffersize)
{
    unsigned int bufferIndex = (int)floatIndex;
    unsigned int nextBufferIndex = bufferIndex+1;
    
    if(nextBufferIndex >= buffersize)
        nextBufferIndex = 0;
    
    double alpha = floatIndex - floor(floatIndex);
    return (audioBuffer[bufferIndex] * (1.0 - alpha)) + (alpha * audioBuffer[nextBufferIndex]);
}
//------------------------------------------------------------------------------

/// void* to the complete new Max External class so that it can be used in the class methods
/// This will be set to t_class* in the main function and a global pointer to our class definition
/// @code t_class* c = class_new(...);
/// myExternClass = c;
static t_class* myExternClass;
const unsigned int numDelayLineSamples = 44100;
const unsigned int numReadHeads = 4;

//------------------------------------------------------------------------------
/// @struct MSPExternalObject
/// @abstract The MaxMSP object
/// @field externalMspObject Header for the MSP object
typedef struct _MSPExternalObject
{
    ///
    t_pxobject externalMspObject;
    ///
    double* delayLine;
    ///
    unsigned int delayTimeMs[numReadHeads];
    ///
    unsigned int currentDelayTimeSamples[numReadHeads];
    ///
    double readHead[numReadHeads];
    ///
    unsigned int targetReadHead[numReadHeads];
    ///
    unsigned int targetDelayTimeSamples[numReadHeads];
    ///
    double slewTime;
    ///
    unsigned int writeHead;
    ///
    double passThruGain;
    ///
    double gain[numReadHeads];
    ///
    double sampRate;
    ///
    void *inlet[numReadHeads];
    ///
    long inletNumber;
} MSPExternalObject;

//------------------------------------------------------------------------------
/// External Object Constructor: use this to setup any variables / properties of your DSP Struct or MSPExternalObject
/// Arguement list should be as long as the list of type arguments passed in the class_new call below.
/// @param s argument symbol
/// @param argc number of arguments
/// @param argv argument vector
/// @returns a void* to an instance of the MSPExternalObject
void* myExternalConstructor(t_symbol *s, long argc, t_atom *argv)
{
    //--------------------------------------------------------------------------
    MSPExternalObject* maxObjectPtr = (MSPExternalObject*)object_alloc(myExternClass);
    dsp_setup((t_pxobject*)maxObjectPtr, 1);
    //--------------------------------------------------------------------------
    outlet_new((t_object*)maxObjectPtr, "signal");
    //--------------------------------------------------------------------------
    maxObjectPtr->delayLine = malloc(sizeof(double) * numDelayLineSamples);
    memset(maxObjectPtr->delayLine, 0, sizeof(double) * numDelayLineSamples);
    //--------------------------------------------------------------------------
    maxObjectPtr->passThruGain = 1.0;
    
    maxObjectPtr->writeHead = 0;
    
    for (int i = 0; i < numReadHeads; i++)
    {
        maxObjectPtr->readHead[i] = ((int)numDelayLineSamples / numReadHeads) * i;
        maxObjectPtr->gain[i] = 0.707;
        maxObjectPtr->inlet[i] = proxy_new((t_object *)maxObjectPtr, (i+1), &maxObjectPtr->inletNumber);
        
        maxObjectPtr->currentDelayTimeSamples[i] = (maxObjectPtr->writeHead) - maxObjectPtr->readHead[i] - 1;
        if (maxObjectPtr->currentDelayTimeSamples[i] < 0)
            maxObjectPtr->currentDelayTimeSamples[i] += numDelayLineSamples;
    }
        
    //--------------------------------------------------------------------------
    return maxObjectPtr;
}

//------------------------------------------------------------------------------
/// @brief what happens when the object is deleted
void myExternDestructor(MSPExternalObject* maxObjectPtr)
{
    dsp_free((t_pxobject*)maxObjectPtr);
}



//------------------------------------------------------------------------------
#pragma mark DSP Loop
/// Main DSP process block, do your DSP here
/// @param maxObjectPtr
/// @param dsp64 can be used to interrogate the number of channels in each of your object’s inlets via the getnuminputchannels method. If your object has two inlets, here is how you can find out how many input channels each inlet has:
/// @param ins double pointer array to sample inlets
/// @param numins number of inputs
/// @param outs double pointer array to sample outlets
/// @param numouts number of outputs
/// @param sampleframes samples per channel
/// @param flags
/// @param userparam no idea
void mspExternalProcessBlock(MSPExternalObject* maxObjectPtr, t_object* dsp64,
                             double** ins, long numins, double** outs, long numouts,
                             long sampleframes, long flags, void* userparam)

{
    double stepSize[4] = {1.0, 1.0, 1.0, 1.0};
    
    if (maxObjectPtr->targetDelayTimeSamples[0] != maxObjectPtr->currentDelayTimeSamples[0])
    {
        if (maxObjectPtr->targetDelayTimeSamples[0] < maxObjectPtr->currentDelayTimeSamples[0])
            stepSize[0] = 5.0;
        else
            stepSize[0] = 0.2;
    }

    for (int i = 0; i < sampleframes; i++)
    {
        double inputSample = ins[0][i];
        
        outs[0][i] = inputSample * maxObjectPtr->passThruGain;

        for (int j = 0; j < numReadHeads; j++)
        {
            outs[0][i] += getInterpOut(maxObjectPtr->readHead[j], maxObjectPtr->delayLine, numDelayLineSamples) * maxObjectPtr->gain[j];
            
            maxObjectPtr->readHead[j] += stepSize[j];
            if(maxObjectPtr->readHead[j] >= (float)numDelayLineSamples)
                maxObjectPtr->readHead[j] -= (float)numDelayLineSamples;            
        }
        maxObjectPtr->currentDelayTimeSamples[0] = (maxObjectPtr->writeHead) - maxObjectPtr->readHead[0] - 1;
        if (maxObjectPtr->currentDelayTimeSamples[0] < 0)
            maxObjectPtr->currentDelayTimeSamples[0] += numDelayLineSamples;
        
        maxObjectPtr->delayLine[maxObjectPtr->writeHead] = inputSample;
        if(++maxObjectPtr->writeHead == numDelayLineSamples)
                    maxObjectPtr->writeHead = 0;
    }
}

//------------------------------------------------------------------------------

/// Audio DSP setup
/// @param maxObjectPtr object pointer
/// @param dsp64 can be used to interrogate the number of channels in each of your object’s inlets via the getnuminputchannels method. If your object has two inlets, here is how you can find out how many input channels each inlet has:
/// @param count array containing number of connections to an inlet with index [i]
/// @param samplerate
/// @param vectorsize
/// @param flags
void prepareToPlay(MSPExternalObject* maxObjectPtr, t_object* dsp64, short* count,
                   double samplerate, long vectorsize, long flags)
{
    object_method(dsp64,
                  gensym("dsp_add64"),
                  maxObjectPtr,
                  mspExternalProcessBlock,
                  0,
                  NULL);
}
//------------------------------------------------------------------------------
void onFloat(MSPExternalObject* maxObjectPtr, double floatIn)
{
    long inletNumber = proxy_getinlet((t_object *)maxObjectPtr);
    
    if(inletNumber)
        maxObjectPtr->gain[inletNumber - 1] = floatIn;
    else
        maxObjectPtr->passThruGain = floatIn;
    
}

void onInt(MSPExternalObject* maxObjectPtr, long intIn)
{
    long inletNumber = proxy_getinlet((t_object *)maxObjectPtr);
    
    if(inletNumber)
    {
        long absDelayMs = labs(intIn);
        long delaySamples = (absDelayMs*numDelayLineSamples) / 1000;
        if (delaySamples >= numDelayLineSamples)
            delaySamples =numDelayLineSamples - 1;
        maxObjectPtr->targetDelayTimeSamples[inletNumber - 1] = (unsigned int)delaySamples;                
    }
    
}

void onPrint(MSPExternalObject* maxObjectPtr)
{
    post("C: %ld T: %ld",maxObjectPtr->currentDelayTimeSamples[0], maxObjectPtr->targetDelayTimeSamples[0]);
}
//------------------------------------------------------------------------------
/// Bundle all class_addmethod calls into one function.
/// @param c max external class pointer
void coupleMethodsToExternal( t_class* c)
{
    class_addmethod(c, (method)prepareToPlay, "dsp64", A_CANT, 0);
    class_addmethod(c, (method)onFloat, "float", A_FLOAT, 0);
    class_addmethod(c, (method)onInt, "int", A_FLOAT, 0);
    class_addmethod(c, (method)onPrint, "print", 0);
}

//------------------------------------------------------------------------------
int C74_EXPORT main(void)
{
    t_class* c = class_new("max-external~",
                           (method)myExternalConstructor,
                           (method)myExternDestructor,
                           (short)sizeof(MSPExternalObject),
                           0L,
                           A_GIMME,
                           0);
    
    coupleMethodsToExternal(c);
    
    class_dspinit(c);
    class_register(CLASS_BOX, c);
    
    myExternClass = c;
    
    return 0;
}
//------------------------------------------------------------------------------

