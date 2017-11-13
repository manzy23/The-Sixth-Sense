/*
    Capstone Project Number: A17-252
	Project Name: The Sixth Sense
    Author: Manujaya Kankanige
    Student Number: 11840954
    Copyright (c) <2017>, Intel Corporation
*/
 

// Constant definitions
#define NUM_OF_SEGMENTS 9
#define THRESHOLD 1500
#define RANGE_MIN 510
#define RANGE_MAX 4000
#define DANGER_ZONE 600
#define DANGER_FRAME_LIMIT 15

// Included Cinder libraries
#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/params/Params.h"

// Included OpenCV libraries
#include "opencv2/imgproc/imgproc.hpp"

// Included RealSense SDK libraries
#include "pxcsensemanager.h"
#include "pxcprojection.h"
#include "pxcspeechsynthesis.h"  
#include "pxcsession.h" 

// Included Windows libraries
#include <Windows.h>  
#include <iostream>
#include <memory>
#include "mmeapi.h"

// Included namespaces
using namespace ci;
using namespace ci::app;
using namespace std;
using namespace cv;


/*
_________________________
|       |       |       |
|	0	|   1   |   2   |
|_______|_______|_______|
|		|       |       |
|	3	|   4   |   5   |
|_______|_______|_______|
|		|       |       |
|	6	|   7   |   8   |
|_______|_______|_______|

*/


// Class for handling algorithms and graphics
class TheSixthSense : public AppNative 
{
    public:
	  
		// Prototypes
	    void setup();
	    void update();
	    void draw();
		float map(float value, float inputMin, float inputMax, float outputMin, float outputMax);
		cinder::Vec2f position(float x, float y);
		void closestDepthSearch();
	    void voiceManager();
		void speak(pxcCHAR* sentence);
		void shutdown();

		// Variables for RealSense camera data acquisition
		PXCSenseManager *senseManager;
		PXCProjection *coordinateMapper;
   		PXCCapture::Sample *currentSample;
		pxcStatus status;

		// Variables for RealSense camera speech synthesis
		PXCSpeechSynthesis *tts;  
	    PXCSession *session; 
		PXCSpeechSynthesis::ProfileInfo pinfo;
	    
		// Variables for displaying depth buffer onscreen
	    Channel8u depthChannel;
	    gl::Texture	depthTexture;

		// Variables to store height and width of depth buffer
		int	depthWidth;
		int depthHeight; 

		// Success flag
       	bool depthFlag;

		// Pointer to depth image frame
		uint16_t * depth;

		// Variables for OpenCV operations
		Mat src;
        Mat element;

		// Variable for Cinder visualtisation
		ci::Vec2i depthSize;

		// Arrays to store the closest, current and previous pixel depths for each segment
		float closestDepth[NUM_OF_SEGMENTS];
		float currentDepth[NUM_OF_SEGMENTS]; 
		float previousDepths[NUM_OF_SEGMENTS][DANGER_FRAME_LIMIT];

		// Pointer responsible depth history buffer activities
		int historyPtr;

		// Variables to specify segment boundaries  
		float topHeight;    // Height of top segments
		float middleHeight; // Height of middle segments
		float bottomHeight; // Height of bottom segments

		float leftWidth;	// Width of left segments
		float middleWidth;  // Width of middle segments
		float rightWidth;	// Width of right segments

		// Array of variables to identify onscreen segments on cinder
		Rectf segments[NUM_OF_SEGMENTS];

		// Array of variables to display the closest depth value for each segment
		stringstream segStrings[NUM_OF_SEGMENTS];
};


// Class for handling audio and voice
class VoiceOut 
{  
	protected:  
        
		enum {buffering = 3};  
   
		// List of variables for working with the speech synthesis module
		WAVEHDR waveHeader[buffering];  
		PXCAudio::AudioData audioData[buffering];  
		PXCAudio* audioSample[buffering];   
		HWAVEOUT waveOut;  
	    WAVEFORMATEX waveFormat;
		int numSamples;
 
	public:  
	  
	  // List of variables that communicate with the output channel
	  VoiceOut(PXCSpeechSynthesis::ProfileInfo *pinfo) 
	  {  
	      numSamples= 0;  
          waveOut= 0;  
   
          memset(&waveFormat,0, sizeof(waveFormat));  
          waveFormat.wFormatTag= WAVE_FORMAT_PCM;  
          waveFormat.nSamplesPerSec= pinfo->outputs.sampleRate;  
          waveFormat.wBitsPerSample= 16;  
          waveFormat.nChannels= pinfo->outputs.nchannels;  
          waveFormat.nBlockAlign= (waveFormat.wBitsPerSample / 8)*waveFormat.nChannels;  
          waveFormat.nAvgBytesPerSec= waveFormat.nBlockAlign*waveFormat.nSamplesPerSec;  
          waveOutOpen(&waveOut,WAVE_MAPPER, &waveFormat, 0, 0, CALLBACK_NULL);  
      }  
   
	  // Audio Rendering
      void RenderAudio(PXCAudio *audio) 
	  {  
          int k = (numSamples%buffering);  
          if (numSamples++ >=buffering)
		  {  
              while (waveOutUnprepareHeader(waveOut,&waveHeader[k], 
				     sizeof(WAVEHDR)) == WAVERR_STILLPLAYING);  

              audioSample[k]->ReleaseAccess(&audioData[k]);  
              audioSample[k]->Release();  
          }  
          audio->AddRef();  
          audioSample[k]= audio;  
          
		  if(audioSample[k]->AcquireAccess(PXCAudio::ACCESS_READ, 
			                               PXCAudio::AUDIO_FORMAT_PCM, 
										   &audioData[k]) >= PXC_STATUS_NO_ERROR) 
		  {  
              memset(&waveHeader[k],0, sizeof(WAVEHDR));  
              waveHeader[k].dwBufferLength= audioData[k].dataSize * 2;  
              waveHeader[k].lpData= (LPSTR)audioData[k].dataPtr;  
              waveOutPrepareHeader(waveOut,&waveHeader[k], sizeof(WAVEHDR));  
              waveOutWrite(waveOut,&waveHeader[k], sizeof(WAVEHDR));  
          }  
      }  
   
      // Audio output command
      ~VoiceOut(void) 
	  {  
          if (!waveOut || numSamples <= 0) 
		   	  return;  

          for (int i = numSamples - buffering + 1; i<numSamples; i++) 
		  {  
              if (i < 0) 
			   	  i++;  

              int k = (i%buffering);  
			  
			  // Do not interrupt halfway through an output
              while (waveOutUnprepareHeader(waveOut, &waveHeader[k], sizeof(WAVEHDR)) == WAVERR_STILLPLAYING);  
              audioSample[k]->ReleaseAccess(&audioData[k]);  
              audioSample[k]->Release();  
          }  
          waveOutClose(waveOut);  
      } 
}; 
 

// Function for initialising program tools
void TheSixthSense::setup()
{	
    // Initialising of vital variables to zero
	historyPtr = 0;
	tts = 0;  

	// Initialise Realsense camera to obtain the depth data
	senseManager = PXCSenseManager::CreateInstance();

	if (senseManager)
	{
		// Resolution of the depth image set to 480 X 360
		depthSize = ci::Vec2i(480, 360);
		depthWidth = depthSize.x; // Extract width of depth buffer
	    depthHeight = depthSize.y; // Extract height of depth buffer

		// Enable the depth stream and initialise to 480 X 360 resolution, 60 fps
		status = senseManager->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, depthWidth, depthHeight, 60); 
		if (status >= PXC_STATUS_NO_ERROR)
		{
			depthFlag = true;
			console() << "Depth Camera Initialisation Successful" << std::endl;
		}
		else
			console() << "Error: Depth Camera Initialisation Failed" << std::endl;
	}

    // Obtain camera stream
	pxcStatus status = senseManager->Init();
	if (status >= PXC_STATUS_NO_ERROR)
	{
		coordinateMapper = senseManager->QueryCaptureManager()->QueryDevice()->CreateProjection();
		console() << "Sense Manager Initialisation Successful" << std::endl;
	}
	else
		console() << "Error: Sense Manager Initialisation Failed" << std::endl;

	session = PXCSession::CreateInstance();  
	session->CreateImpl<PXCSpeechSynthesis>(&tts);  
     
	tts->QueryProfile(0, &pinfo);  
	pinfo.language = PXCSpeechSynthesis::LANGUAGE_US_ENGLISH;  
	tts->SetProfile(&pinfo);  

	// Onscreen segment boundary values
	topHeight = float(depthHeight/3);
	middleHeight = float((depthHeight*2)/3);
	bottomHeight = float(depthHeight);
	rightWidth = float(depthWidth/3);
	middleWidth = float((depthWidth*2)/3);
	leftWidth = float(depthWidth);

	// Display visualiser segments onscreen
	segments[0] = Rectf(0, 0, rightWidth, topHeight);
	segments[1] = Rectf(rightWidth, 0, middleWidth, topHeight);
	segments[2] = Rectf(middleWidth, 0, leftWidth, topHeight);
	segments[3] = Rectf(0, topHeight, rightWidth, middleHeight);
	segments[4] = Rectf(rightWidth, topHeight, middleWidth, middleHeight);
	segments[5] = Rectf(middleWidth, topHeight, leftWidth, middleHeight);
	segments[6] = Rectf(0, middleHeight, rightWidth, bottomHeight);
	segments[7] = Rectf(rightWidth, middleHeight, middleWidth, bottomHeight);
	segments[8] = Rectf(middleWidth, middleHeight, leftWidth, bottomHeight);

    // Stream display setup 
	depthChannel = Channel8u(depthWidth, depthHeight);
	setWindowSize(depthWidth, depthHeight);
}


// Function carried out to achieve realtime recognition
void TheSixthSense::update()
{
	int dangerFrameCount[NUM_OF_SEGMENTS] = {0};

	pxcStatus status;
	if (senseManager)
	{
		// Acquire frame based on given stream parameters
		status = senseManager->AcquireFrame(); 
		if (status < PXC_STATUS_NO_ERROR)
			console() << "Error: Frame Acquisition Failed" << std::endl;

		PXCCapture::Sample *currentSample = senseManager->QuerySample();

		if (!currentSample)
			console() << "Error: Sampling Query Failed" << std::endl;
		
		if (depthFlag)
		{
			if (!currentSample->depth)
				console() << "Error: Sampling Depth Failed" << std::endl;

			PXCImage *cDepthImage = currentSample->depth;
			PXCImage::ImageData cDepthData;
			
			// Read depth frame in cDepthData
			status = cDepthImage->AcquireAccess(PXCImage::ACCESS_READ, 
				                                PXCImage::PIXEL_FORMAT_DEPTH, 
												&cDepthData); 
			
			if (status < PXC_STATUS_NO_ERROR)
			{
				cDepthImage->ReleaseAccess(&cDepthData);
				console() << "Error: Accessing Depth Data Failed" << std::endl;
			}

			// Store depth map in the "depth" variable 
			depth = reinterpret_cast<uint16_t *>(cDepthData.planes[0]); 

			// Resample depth data to fit 500mm - 4000mm into 0-255 grayscale for visualising
			int q = 0;
			auto it = depthChannel.getIter();
			while (it.line())
			{
				while (it.pixel())
				{
					uint16_t val = *(depth + q);
					if (val < RANGE_MIN)
						it.v() = 0;
					else
						it.v() = uint8_t(map(val, RANGE_MIN, RANGE_MAX, 0, 255));
					q++;
				}
			}
			depthTexture = gl::Texture(depthChannel);
	

			// Assign a Mat header to depth data array
			src = Mat(cvSize(480, 360), CV_16UC1, depth);

			// Initialisation of the strength of erosion
			element = getStructuringElement(MORPH_RECT, Size(4, 4), Point(-1, -1));

			// Erode function for noise removal
			erode(src, src, element);

			closestDepthSearch(); // Function that finds the closest depth in each segment
			cDepthImage->ReleaseAccess(&cDepthData); // Release depth access for current frame
		}
		senseManager->ReleaseFrame(); // Release current frame


		// Update history buffer 
		for (int i = 0; i < NUM_OF_SEGMENTS; i++)
	    {
			previousDepths[i][historyPtr] = closestDepth[i];
		}

		// Check history of depth values 
		for (int i = 0; i < NUM_OF_SEGMENTS; i++)
	    {
		    for (int j = 0; j < DANGER_FRAME_LIMIT; j++)
		    {
				if ((previousDepths[i][j] > RANGE_MIN) && (previousDepths[i][j] < DANGER_ZONE))
					dangerFrameCount[i]++;
		    }
	    }

		// Overwrite history buffer (if full)
		if (historyPtr < DANGER_FRAME_LIMIT-1)
			historyPtr++;
		else 
		{
			historyPtr = 0;
			for (int i = 0; i < NUM_OF_SEGMENTS; i++)
	        {
		        for (int j = 0; j < DANGER_FRAME_LIMIT; j++)
		        {
				    previousDepths[i][j] = 0;
		        }
	        }
		}

		// Generate voice command if 15 continuous frames have depth values in the danger area
		for (int i = 0; i < NUM_OF_SEGMENTS; i++)
	    {
			if (dangerFrameCount[i] == DANGER_FRAME_LIMIT)
			{
				voiceManager();
			    break;
			}
		}
	}
	else
        quit();
}


// Function for setting up the onscreen visualiser
void TheSixthSense::draw()
{
	gl::enableAlphaBlending();

	// Initialise window colour to black
	gl::clear(Color(0, 0, 0)); 

    // Display depth camera stream onscreen
	gl::color(ColorA(1, 1, 1, 1));
	gl::draw(depthTexture);

	// Update onscreen visualiser segments 
	if (closestDepth[0] == 0)
        gl::color(ColorA(0, 0, 0, 0)); // Black if segment 0 detects too close
	else if (closestDepth[0] < DANGER_ZONE)
        gl::color(ColorA(1, 0, 0, 1)); // Red if segment 0 detects danger zone
	else // Shade in between red and green if segment 0 within detection threshold
	    gl::color(ColorA(1 - map(closestDepth[0], RANGE_MIN, THRESHOLD, 0, 1), 
		                 map(closestDepth[0], RANGE_MIN, THRESHOLD, 0, 1), 
		                 0, 
					     0.5));
	gl::drawSolidRect(segments[0]);

	if (closestDepth[1] == 0)
        gl::color(ColorA(0, 0, 0, 0)); // Black if segment 1 detects too close
	else if (closestDepth[1] < DANGER_ZONE)
        gl::color(ColorA(1, 0, 0, 1)); // Red if segment 1 detects danger zone
	else // Shade in between red and green if segment 1 within detection threshold
	    gl::color(ColorA(1 - map(closestDepth[1], RANGE_MIN, THRESHOLD, 0, 1), 
		                 map(closestDepth[1], RANGE_MIN, THRESHOLD, 0, 1), 
		                 0, 
				    	 0.5));
	gl::drawSolidRect(segments[1]);

	if (closestDepth[2] == 0)
        gl::color(ColorA(0, 0, 0, 0)); // Black if segment 2 detects too close
	else if (closestDepth[2] < DANGER_ZONE)
        gl::color(ColorA(1, 0, 0, 1)); // Red if segment 2 detects danger zone
	else // Shade in between red and green if segment 2 within detection threshold
	    gl::color(ColorA(1 - map(closestDepth[2], RANGE_MIN, THRESHOLD, 0, 1), 
		                 map(closestDepth[2], RANGE_MIN, THRESHOLD, 0, 1), 
		                 0, 
			    		 0.5));
	gl::drawSolidRect(segments[2]);

	if (closestDepth[3] == 0)
        gl::color(ColorA(0, 0, 0, 0)); // Black if segment 3 detects too close
    else if (closestDepth[3] < DANGER_ZONE)
        gl::color(ColorA(1, 0, 0, 1)); // Red if segment 3 detects danger zone
	else // Shade in between red and green if segment 3 within detection threshold
	    gl::color(ColorA(1 - map(closestDepth[3], RANGE_MIN, THRESHOLD, 0, 1), 
		                 map(closestDepth[3], RANGE_MIN, THRESHOLD, 0, 1), 
		                 0, 
				    	 0.5));
	gl::drawSolidRect(segments[3]);

	if (closestDepth[4] == 0)
        gl::color(ColorA(0, 0, 0, 0)); // Black if segment 4 detects too close
	else if (closestDepth[4] < DANGER_ZONE)
        gl::color(ColorA(1, 0, 0, 1)); // Red if segment 4 detects danger zone
	else // Shade in between red and green if segment 4 within detection threshold
	    gl::color(ColorA(1 - map(closestDepth[4], RANGE_MIN, THRESHOLD, 0, 1), 
		                 map(closestDepth[4], RANGE_MIN, THRESHOLD, 0, 1), 
		                 0, 
				    	 0.5));
	gl::drawSolidRect(segments[4]);

	if (closestDepth[5] == 0)
        gl::color(ColorA(0, 0, 0, 0)); // Black if segment 5 detects too close
	else if (closestDepth[5] < DANGER_ZONE)
        gl::color(ColorA(1, 0, 0, 1)); // Red if segment 5 detects danger zone
	else // Shade in between red and green if segment 5 within detection threshold
	    gl::color(ColorA(1 - map(closestDepth[5], RANGE_MIN, THRESHOLD, 0, 1), 
		                 map(closestDepth[5], RANGE_MIN, THRESHOLD, 0, 1), 
		                 0, 
				    	 0.5));
	gl::drawSolidRect(segments[5]);

	if (closestDepth[6] == 0)
        gl::color(ColorA(0, 0, 0, 0)); // Black if segment 6 detects too close
	else if (closestDepth[6] < DANGER_ZONE)
        gl::color(ColorA(1, 0, 0, 1)); // Red if segment 6 detects danger zone
	else // Shade in between red and green if segment 6 within detection threshold
	    gl::color(ColorA(1 - map(closestDepth[6], RANGE_MIN, THRESHOLD, 0, 1), 
		                 map(closestDepth[6], RANGE_MIN, THRESHOLD, 0, 1), 
		                 0, 
				    	 0.5));
	gl::drawSolidRect(segments[6]);

	if (closestDepth[7] == 0)
        gl::color(ColorA(0, 0, 0, 0)); // Black if segment 7 detects too close
	else if (closestDepth[7] < DANGER_ZONE)
        gl::color(ColorA(1, 0, 0, 1)); // Red if segment 7 detects danger zone
	else // Shade in between red and green if segment 7 within detection threshold
	    gl::color(ColorA(1 - map(closestDepth[7], RANGE_MIN, THRESHOLD, 0, 1), 
		                 map(closestDepth[7], RANGE_MIN, THRESHOLD, 0, 1), 
		                 0, 
				    	 0.5)); 
	gl::drawSolidRect(segments[7]);
	
	if (closestDepth[8] == 0)
        gl::color(ColorA(0, 0, 0, 0)); // Black if segment 8 detects too close
	else if (closestDepth[8] < DANGER_ZONE)
        gl::color(ColorA(1, 0, 0, 1)); // Red if segment 8 detects danger zone
	else // Shade in between red and green if segment 8 within detection threshold
	    gl::color(ColorA(1.0f - map(closestDepth[8], RANGE_MIN, THRESHOLD, 0, 1), 
		                 map(closestDepth[8], RANGE_MIN, THRESHOLD, 0, 1), 
		                 0, 
				    	 0.5f)); 
	gl::drawSolidRect(segments[8]);

	// Draw segment outlines onscreen in white
	gl::color(ColorA(1, 1, 1, 1));
	gl::drawStrokedRect(segments[0]);
	gl::drawStrokedRect(segments[1]);
	gl::drawStrokedRect(segments[2]);
	gl::drawStrokedRect(segments[3]);
	gl::drawStrokedRect(segments[4]);
	gl::drawStrokedRect(segments[5]);
	gl::drawStrokedRect(segments[6]);
	gl::drawStrokedRect(segments[7]);
	gl::drawStrokedRect(segments[8]);

	// Update strings with latest closest depth value
	for (int i = 0; i < NUM_OF_SEGMENTS; i++)
	{
		if (closestDepth[i] == 0)
            segStrings[i].str("Too Close");
		else
		{
			segStrings[i].str("");
            segStrings[i] << closestDepth[i];
		}
	}

	// Display closest depth value onscreen
	gl::drawString(segStrings[0].str(), position(0.0f,0.0f), ColorA(1,1,1), Font("Calibri", 30));
	gl::drawString(segStrings[1].str(), position(0.33f,0.0f), ColorA(1,1,1), Font("Calibri", 30));
	gl::drawString(segStrings[2].str(), position(0.66f,0.0f), ColorA(1,1,1), Font("Calibri", 30));

	gl::drawString(segStrings[3].str(), position(0.0f,0.33f), ColorA(1,1,1), Font("Calibri", 30));
	gl::drawString(segStrings[4].str(), position(0.33f,0.33f), ColorA(1,1,1), Font("Calibri", 30));
	gl::drawString(segStrings[5].str(), position(0.66f,0.33f), ColorA(1,1,1), Font("Calibri", 30));

	gl::drawString(segStrings[6].str(), position(0.0f,0.66f), ColorA(1,1,1), Font("Calibri", 30));
	gl::drawString(segStrings[7].str(), position(0.33f,0.66f), ColorA(1,1,1), Font("Calibri", 30));
	gl::drawString(segStrings[8].str(), position(0.66f,0.66f), ColorA(1,1,1), Font("Calibri", 30));

	gl::disableAlphaBlending();
}


// Function for mapping values from one range to another
float TheSixthSense::map(float value, float inputMin, float inputMax, float outputMin, float outputMax) 
{
      return ((value - inputMin) / (inputMax - inputMin) * (outputMax - outputMin) + outputMin);
}


// Function for obtaining positioning of the depth value text in vector form
cinder::Vec2f TheSixthSense::position(float x, float y)
{
	return cinder::Vec2f(getWindowWidth()*x, getWindowHeight()*y);
}


// Function to determine the closest depth pixel in each segment
void TheSixthSense::closestDepthSearch()
{
	int cVer; // Counter for looping over vertical pixels
	int cHor; // Counter for looping over horizontal pixels

    for (int i = 0; i < NUM_OF_SEGMENTS; i++)
	{
	   currentDepth[i] = 0;
	   closestDepth[i] = 10000;   
	} 

	// Find closest pixels in each segment
	// Segment 0 - Top left
	for (cVer = 0; cVer < int(topHeight); cVer++)
	{
		for (cHor = 0; cHor < int(rightWidth); cHor++)
		{
			currentDepth[0] = depth[(cVer * depthWidth) + cHor];
			if ((currentDepth[0] > RANGE_MIN) && (currentDepth[0] < closestDepth[0]))
				closestDepth[0] = currentDepth[0];
		}
	}

	// Segment 1 - Top middle
	for (cVer = 0; cVer < int(topHeight); cVer++)
	{
		for (cHor = int(rightWidth); cHor < int(middleWidth); cHor++)
		{
			currentDepth[1] = depth[(cVer * depthWidth) + cHor];
			if ((currentDepth[1] > RANGE_MIN) && (currentDepth[1] < closestDepth[1]))
				closestDepth[1] = currentDepth[1];
		}
	}

	// Segment 2 - Top right
	for (cVer = 0; cVer < int(topHeight); cVer++)
	{
		for (cHor = int(middleWidth); cHor < int(leftWidth); cHor++)
		{
			currentDepth[2] = depth[(cVer * depthWidth) + cHor];
			if ((currentDepth[2] > RANGE_MIN) && (currentDepth[2] < closestDepth[2]))
				closestDepth[2] = currentDepth[2];
		}
	}

	// Segment 3 - Centre left
	for (cVer = int(topHeight); cVer < int(middleHeight); cVer++)
	{
		for (cHor = 0; cHor < int(rightWidth); cHor++)
		{
			currentDepth[3] = depth[(cVer * depthWidth) + cHor];
			if ((currentDepth[3] > RANGE_MIN) && (currentDepth[3] < closestDepth[3]))
				closestDepth[3] = currentDepth[3];
		}
	}

	// Segment 4 - Centre middle
	for (cVer = int(topHeight); cVer < int(middleHeight); cVer++)
	{
		for (cHor = int(rightWidth); cHor < int(middleWidth); cHor++)
		{
			currentDepth[4] = depth[(cVer * depthWidth) + cHor];
			if ((currentDepth[4] > RANGE_MIN) && (currentDepth[4] < closestDepth[4]))
				closestDepth[4] = currentDepth[4];
		}
	}

	// Segment 5 - Centre right
	for (cVer = int(topHeight); cVer < int(middleHeight); cVer++)
	{
		for (cHor = int(middleWidth); cHor < int(leftWidth); cHor++)
		{
			currentDepth[5] = depth[(cVer * depthWidth) + cHor];
			if ((currentDepth[5] > RANGE_MIN) && (currentDepth[5] < closestDepth[5]))
				closestDepth[5] = currentDepth[5];
		}
	}

	// Segment 6 - Bottom left
	for (cVer = int(middleHeight); cVer < int(bottomHeight); cVer++)
	{
		for (cHor = 0; cHor < int(rightWidth); cHor++)
		{
			currentDepth[6] = depth[(cVer * depthWidth) + cHor];
			if ((currentDepth[6] > RANGE_MIN) && (currentDepth[6] < closestDepth[6]))
				closestDepth[6] = currentDepth[6];
		}
	}

	// Segment 7 - Bottom middle
	for (cVer = int(middleHeight); cVer < int(bottomHeight); cVer++)
	{
		for (cHor = int(rightWidth); cHor < int(middleWidth); cHor++)
		{
			currentDepth[7] = depth[(cVer * depthWidth) + cHor];
			if ((currentDepth[7] > RANGE_MIN) && (currentDepth[7] < closestDepth[7]))
				closestDepth[7] = currentDepth[7];
		}
	}

	// Segment 8 - Bottom right
	for (cVer = int(middleHeight); cVer < int(bottomHeight); cVer++)
	{
		for (cHor = int(middleWidth); cHor < int(leftWidth); cHor++)
		{
			currentDepth[8] = depth[(cVer * depthWidth) + cHor];
			if ((currentDepth[8] > RANGE_MIN) && (currentDepth[8] < closestDepth[8]))
				closestDepth[8] = currentDepth[8];
		}
	}

	// Pixel too close to camera
    for (int i = 0; i < NUM_OF_SEGMENTS; i++)
    {
	   if (closestDepth[i] == 10000)
	       closestDepth[i] = 0;   
    } 

	// Display distance of closest pixel in each segment
	console() << "Seg1:" << closestDepth[0] << " " 
		      << "Seg2:" << closestDepth[1] << " " 
			  << "Seg3:" << closestDepth[2] << " " 
			  << "Seg4:" << closestDepth[3] << " " 
			  << "Seg5:" << closestDepth[4] << " " 
		      << "Seg6:" << closestDepth[5] << " " 
			  << "Seg7:" << closestDepth[6] << " "
			  << "Seg8:" << closestDepth[7] << " "
			  << "Seg9:" << closestDepth[8] << " "
			  << std::endl;
}


// Function for managing voice commands
void TheSixthSense::voiceManager()
{
	pxcCHAR* line = NULL;
	
	// Stop command
	if (((closestDepth[0] < DANGER_ZONE) || (closestDepth[1] < DANGER_ZONE) || (closestDepth[2] < DANGER_ZONE) ||
		 (closestDepth[3] < DANGER_ZONE) || (closestDepth[4] < DANGER_ZONE) || (closestDepth[5] < DANGER_ZONE)) && 
	   (((closestDepth[3] < DANGER_ZONE) && (closestDepth[4] < DANGER_ZONE) && (closestDepth[5] < DANGER_ZONE)) ||
	    ((closestDepth[6] < DANGER_ZONE) && (closestDepth[7] < DANGER_ZONE) && (closestDepth[8] < DANGER_ZONE))) ||
		
		((closestDepth[0] < DANGER_ZONE) && (closestDepth[4] < DANGER_ZONE) && (closestDepth[5] < DANGER_ZONE) ||
		 (closestDepth[0] < DANGER_ZONE) && (closestDepth[7] < DANGER_ZONE) && (closestDepth[8] < DANGER_ZONE) ||
		 (closestDepth[2] < DANGER_ZONE) && (closestDepth[3] < DANGER_ZONE) && (closestDepth[4] < DANGER_ZONE) ||
		 (closestDepth[2] < DANGER_ZONE) && (closestDepth[6] < DANGER_ZONE) && (closestDepth[7] < DANGER_ZONE) ||
		 (closestDepth[1] < DANGER_ZONE) && (closestDepth[2] < DANGER_ZONE) && (closestDepth[6] < DANGER_ZONE) ||
		 (closestDepth[4] < DANGER_ZONE) && (closestDepth[5] < DANGER_ZONE) && (closestDepth[6] < DANGER_ZONE) ||
		 (closestDepth[0] < DANGER_ZONE) && (closestDepth[1] < DANGER_ZONE) && (closestDepth[8] < DANGER_ZONE) ||
		 (closestDepth[3] < DANGER_ZONE) && (closestDepth[4] < DANGER_ZONE) && (closestDepth[8] < DANGER_ZONE) ||
		 (closestDepth[1] < DANGER_ZONE) && (closestDepth[2] < DANGER_ZONE) && (closestDepth[3] < DANGER_ZONE) ||
		 (closestDepth[5] < DANGER_ZONE) && (closestDepth[6] < DANGER_ZONE) && (closestDepth[7] < DANGER_ZONE) ||
		 (closestDepth[0] < DANGER_ZONE) && (closestDepth[1] < DANGER_ZONE) && (closestDepth[5] < DANGER_ZONE) ||
		 (closestDepth[3] < DANGER_ZONE) && (closestDepth[7] < DANGER_ZONE) && (closestDepth[8] < DANGER_ZONE)))
	    line = L"Stop";

	// Duck Command
	else if ((closestDepth[0] < DANGER_ZONE) && (closestDepth[1] < DANGER_ZONE) && (closestDepth[2] < DANGER_ZONE))
	    line = L"Duck";

	// Step over command
	else if ((closestDepth[6] < DANGER_ZONE) && (closestDepth[7] < DANGER_ZONE) && (closestDepth[8] < DANGER_ZONE))
	    line = L"Step Over";

	// Turn left command
	else if ((closestDepth[1] < DANGER_ZONE) && (closestDepth[2] < DANGER_ZONE) ||
		     (closestDepth[1] < DANGER_ZONE) && (closestDepth[5] < DANGER_ZONE) ||
			 (closestDepth[1] < DANGER_ZONE) && (closestDepth[8] < DANGER_ZONE) ||
			 (closestDepth[4] < DANGER_ZONE) && (closestDepth[2] < DANGER_ZONE) ||
			 (closestDepth[4] < DANGER_ZONE) && (closestDepth[5] < DANGER_ZONE) ||
		     (closestDepth[4] < DANGER_ZONE) && (closestDepth[8] < DANGER_ZONE) ||
			 (closestDepth[7] < DANGER_ZONE) && (closestDepth[2] < DANGER_ZONE) ||
			 (closestDepth[7] < DANGER_ZONE) && (closestDepth[5] < DANGER_ZONE) ||
		     (closestDepth[7] < DANGER_ZONE) && (closestDepth[8] < DANGER_ZONE))
	   	line = L"Turn Left";
    
	// Turn right command
	else if ((closestDepth[0] < DANGER_ZONE) && (closestDepth[1] < DANGER_ZONE) ||
		     (closestDepth[0] < DANGER_ZONE) && (closestDepth[4] < DANGER_ZONE) ||
			 (closestDepth[0] < DANGER_ZONE) && (closestDepth[7] < DANGER_ZONE) ||
			 (closestDepth[3] < DANGER_ZONE) && (closestDepth[1] < DANGER_ZONE) ||
			 (closestDepth[3] < DANGER_ZONE) && (closestDepth[4] < DANGER_ZONE) ||
	       	 (closestDepth[3] < DANGER_ZONE) && (closestDepth[7] < DANGER_ZONE) ||
			 (closestDepth[6] < DANGER_ZONE) && (closestDepth[1] < DANGER_ZONE) ||
			 (closestDepth[6] < DANGER_ZONE) && (closestDepth[4] < DANGER_ZONE) ||
		     (closestDepth[6] < DANGER_ZONE) && (closestDepth[7] < DANGER_ZONE))
	   	line = L"Turn Right";

	// Narrow path ahead command
	else if ((closestDepth[0] < DANGER_ZONE) && (closestDepth[2] < DANGER_ZONE) ||
		     (closestDepth[0] < DANGER_ZONE) && (closestDepth[5] < DANGER_ZONE) ||
			 (closestDepth[0] < DANGER_ZONE) && (closestDepth[8] < DANGER_ZONE) ||
		     (closestDepth[3] < DANGER_ZONE) && (closestDepth[2] < DANGER_ZONE) ||
			 (closestDepth[3] < DANGER_ZONE) && (closestDepth[5] < DANGER_ZONE) ||
			 (closestDepth[3] < DANGER_ZONE) && (closestDepth[8] < DANGER_ZONE) ||
			 (closestDepth[6] < DANGER_ZONE) && (closestDepth[2] < DANGER_ZONE) ||
			 (closestDepth[6] < DANGER_ZONE) && (closestDepth[5] < DANGER_ZONE) ||
		     (closestDepth[6] < DANGER_ZONE) && (closestDepth[8] < DANGER_ZONE))
	   	line = L"Narrow Path Ahead";
	
	// Slight left command
	else if ((closestDepth[2] < DANGER_ZONE) || (closestDepth[5] < DANGER_ZONE) || (closestDepth[8] < DANGER_ZONE)) 
	   	line = L"Slight Left";

	// Slight right command
	else if ((closestDepth[0] < DANGER_ZONE) || (closestDepth[3] < DANGER_ZONE) || (closestDepth[6] < DANGER_ZONE)) 
	   	line = L"Slight Right";

	// Step left or right command
	else if ((closestDepth[1] < DANGER_ZONE) || (closestDepth[4] < DANGER_ZONE) || (closestDepth[7] < DANGER_ZONE))
	    line = L"Step Left or Right";

	// Link to audio
	if (line != NULL)
	    speak(line);
}


// Function to control the camera speech module
void TheSixthSense::speak(pxcCHAR* sentence)
{  
	tts->BuildSentence(1, sentence); // Synthesise the text string  
	int nbuffers = tts->QueryBufferNum(1); // Retrieve synthesised speech  
	VoiceOut vo(&pinfo);  
   
	for (int i = 0; i<nbuffers; i++) 
	{  
		PXCAudio *audio = tts->QueryBuffer(1, i);  
		vo.RenderAudio(audio); // Send audio sample to audio output device        
    }  
    tts->ReleaseSentence(1); // Release current sentence
} 


// Function to turn of camera modules safely on shutdown
void TheSixthSense::shutdown()
{
	if (senseManager)
	{
		coordinateMapper->Release();
		senseManager->Close();
	}
}

CINDER_APP_NATIVE(TheSixthSense, RendererGl)
