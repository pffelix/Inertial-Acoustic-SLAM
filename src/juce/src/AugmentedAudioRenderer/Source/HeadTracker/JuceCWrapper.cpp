#include <iostream>
#include "JuceHeader.h"
#include "JuceCWrapper.h"
//#include "timing.h"
// FFT
int maximalFftOrder = 20; // maximal used FFT order
juce::dsp::FFT ** dspObject;
//juce::dsp::FFT * dspObject;

// WAV
File *wavFile_default;
AudioBuffer<float> wavBuffer_default;
WavAudioFormat *wavFormat_default;
//ScopedPointer<AudioFormatWriter> wavWriter; std::unique_ptr<AudioFormatWriter> writer;
//std::unique_ptr<AudioFormatWriter> wavWriter;
double wavSamplingRate;

void JuceCWrapperInit(int maximalFftOrder_dynamic, long Nmax, int Channelmax, double samplingRate) {
	// Init FFT objects up to maximal used FFT Order
	if (maximalFftOrder_dynamic) {
		maximalFftOrder = maximalFftOrder_dynamic;
		dspObject = (juce::dsp::FFT**)malloc((maximalFftOrder * sizeof(juce::dsp::FFT*)));
		for (int fftOrder = 0; fftOrder <= maximalFftOrder; fftOrder++) {
			dspObject[fftOrder] = new juce::dsp::FFT(fftOrder);
			//dspObject = new juce::dsp::FFT(fftOrder);
		}
	}
	// Init WAV objects
	wavFormat_default = new WavAudioFormat();
	wavBuffer_default.setSize(Channelmax, Nmax, false, false, true);
	wavSamplingRate = (double)samplingRate;
}

void JuceCWrapperFFTForward(float* x, long x_N, bool ignore_negative_freqs) {
	int fftOrder = (int)ceil(log(x_N)/log(2)); 
	dspObject[fftOrder]->performRealOnlyForwardTransform(x, ignore_negative_freqs);
}

void JuceCWrapperFFTInverse(float* x, long x_N) {
	int fftOrder = (int)ceil(log(x_N)/log(2)); 
	dspObject[fftOrder]->performRealOnlyInverseTransform(x);
}

void JuceCWrapperWAVRead(const char *filepath, float ** output, float* sample_rate) {
    File file (filepath);
    AudioFormatManager formatManager;
	bool aa = file.existsAsFile();
	formatManager.registerBasicFormats();
	std::unique_ptr<AudioFormatReader> reader (formatManager.createReaderFor(file));
    if (reader.get() != nullptr) { 
        AudioSampleBuffer buffer(reader->numChannels, (long)reader->lengthInSamples);
		reader->read (&buffer,                                          
                0,                                                          
                (long) reader->lengthInSamples,                             
                0,                                                          
                true,                                                       
                true);   
		const float** buffer_float = buffer.getArrayOfReadPointers();
		for (int ch = 0; ch < (int)reader->numChannels; ch++) {
			memcpy(output[ch], buffer_float[ch], reader->lengthInSamples * sizeof(float));
		}
		*sample_rate = (float)reader->sampleRate;
    }
}

void JuceCWrapperWAVWrite(float **x, int Nchannels, long Nsamples, const char *filepath, const char* metadata, double samplingRate) {
	// convert metadata
	//StringPairArray meta = WavAudioFormat::createBWAVMetadata("test","test","test",Time::getCurrentTime(),0,"test"); // StringPairArray();
	//if (!(metadata[0] == '\0')) {
		//meta.set("INAM ", String(metadata));
		//
	//}

	// multithreading
	File* wavFile;
	AudioBuffer<float> wavBuffer;
	WavAudioFormat* wavFormat;
	wavFormat= new WavAudioFormat();

	// write wav
	wavFile = new File(File::getCurrentWorkingDirectory().getChildFile (filepath)); // wavFile_default
	wavFile->deleteFile(); // wavFile_default
	wavBuffer.setDataToReferTo(x, Nchannels, Nsamples); // wavBuffer_default
	FileOutputStream *wavOutputstream = new FileOutputStream(*wavFile);
	AudioFormatWriter *wavWriter = (wavFormat->createWriterFor(wavOutputstream, // wavFormat_default
		(double)samplingRate,
		wavBuffer.getNumChannels(),
		16,
		{}, //{}
		0));//wavWriter.reset(, new FileOutputStream(*wavFile); //ScopedPointer<AudioFormatWriter> wavWriter; 
	if (wavWriter != nullptr)
		wavWriter->writeFromAudioSampleBuffer(wavBuffer, 0, wavBuffer.getNumSamples());
	//wavWriter->~AudioFormatWriter();
	delete wavWriter;
	delete wavFile;

	// multithreading
	delete wavFormat;
}

void JuceCWrapperClose() {
   //delete dspObject;

	for (int n = 0; n < maximalFftOrder; n++) {
		delete dspObject[n];
	}
	free (dspObject);

	// delte WAV Object
	//free(buffer);
	free(wavFormat_default);
	//free(wavBuffer);
	free(wavFile_default);

   return;
}
