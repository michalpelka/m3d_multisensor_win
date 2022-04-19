//=============================================================================
// Copyright (c) 2001-2018 FLIR Systems, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================

//=============================================================================
// This example illustrates the post processing pipeline API on LD5 and newer.
//
// It is similar to the SimpleGrab example but sets various image processing
// options that are available on the LadyBug5 camera.  The major function
// of interest here is setPostProcessingOptions() which sets the post
// processing options.
//
//=============================================================================

#include <stdio.h>
#include <string.h>
#include <string>
#include <stdlib.h>
#include "ladybug.h"
#include "ladybugImageAdjustment.h"
#include "ladybuggeom.h"
#include "ladybugrenderer.h"

#ifdef _WIN32

#include <windows.h>
#include <shlobj.h>

#else

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#define _MAX_PATH 4096

#endif

namespace
{
	LadybugContext context{NULL};
	std::string getWriteableDirectory()
	{
		std::string writeableDirectory;
#ifdef _WIN32
		char buf[_MAX_PATH];
		HRESULT res = SHGetFolderPath(NULL, CSIDL_PERSONAL, NULL, 0, buf);
		writeableDirectory.append(buf);
		if (res == S_OK)
		{
			writeableDirectory.append("\\");
		}
#else
		const char* homedir;

		if ((homedir = getenv("HOME")) == NULL)
		{
			uid_t uid = getuid();
			struct passwd* pw = getpwuid(uid);

			if (pw == NULL)
			{
				homedir = NULL;
			}
			else
			{
				homedir = pw->pw_dir;
			}
		}
		if (homedir != NULL)
		{
			writeableDirectory.append(homedir).append("/");
		}
#endif

		return writeableDirectory;
	}
}

void handleError(LadybugError error, const char* message = NULL);
void setPostProcessingOptions(LadybugContext context);

bool captureLadybugImage(const std::string& directory)
{

	// Initialize context.
	if (context) {
		handleError(ladybugDestroyContext(&context), "ladybugDestroyContext()");
	}
	handleError(ladybugCreateContext(&context));

	// Initialize the first ladybug on the bus.
	printf("Initializing...\n");
	handleError(ladybugInitializeFromIndex(context, 0), "ladybugInitializeFromIndex()");

	// Get camera info
	LadybugCameraInfo caminfo;
	handleError(ladybugGetCameraInfo(context, &caminfo), "ladybugGetCameraInfo()");

	// Load the calibration
	handleError(ladybugLoadConfig(context, NULL), "ladybugLoadConfig()");

	// Start up the camera according to device type and data format
	printf("Starting %s (%u)...\n", caminfo.pszModelName, caminfo.serialHead);
	handleError(ladybugStart(context, LADYBUG_DATAFORMAT_RAW12), "ladybugStart()");
	LadybugImage image;
	for (int i = 0; i < 5; i++)
	{
		// Grab a single image for image size

		handleError(ladybugGrabImage(context, &image), "ladybugGrabImage()");
		printf("pregrabing ...\n");
	}
	// Set alpha masks
	printf("Enabling alpha masks...\n");
	handleError(ladybugInitializeAlphaMasks(context, image.uiCols, image.uiRows), "ladybugInitializeAlphaMasks()");

	// Set output image
	printf("Enabling panoramic images...\n");
	handleError(ladybugConfigureOutputImages(context, LADYBUG_PANORAMIC), "ladybugConfigureOutputImages");

	// Set rendered image size
	printf("Enabling rendering image size...\n");
	handleError(ladybugSetOffScreenImageSize(context, LADYBUG_PANORAMIC, 2 * 2048, 2048), "ladybugSetOffScreenImageSize");

	// Set color processing method
	printf("Setting debayering method...\n");
	handleError(ladybugSetColorProcessingMethod(context, LADYBUG_NEAREST_NEIGHBOR_FAST), "ladybugSetColorProcessingMethod()");

	// Allocate memory for the 6 processed images
	unsigned char* arpBuffers[LADYBUG_NUM_CAMERAS] = { 0 };
	for (unsigned int uiCamera = 0; uiCamera < LADYBUG_NUM_CAMERAS; uiCamera++)
	{
		arpBuffers[uiCamera] = new unsigned char[image.uiRows * image.uiCols * 4 * sizeof(unsigned short)];

		// Initialize the entire buffer so that the alpha channel has a valid (maximum) value.
		memset(arpBuffers[uiCamera], 0xff, image.uiRows * image.uiCols * 4 * sizeof(unsigned short));
	}

	// Set conversion properties for post-procesing
	setPostProcessingOptions(context);

	// Grab images and process them
	LadybugProcessedImage processedImage;
	for (int i = 0; i < 1; i++)
	{
		// Grab the next image
		printf("Grabbing image...\n");
		handleError(ladybugGrabImage(context, &image), "ladybugGrabImage()");

		// Convert and post-process the image
		// If the result is not desired, then ladybugConvertImage() can be used too
		printf("Converting image...\n");

		bool outputDesired = true;
		if (outputDesired)
		{
			ConvertImageOutput convertedImageResults;
			handleError(ladybugConvertImageEx(context, &image, arpBuffers, LADYBUG_BGRU16, convertedImageResults), "ladybugConvertImage()");

			printf("Grabbed image %d - target mean was %s\n", i, convertedImageResults.targetMeanReach ? "reached" : "not reached");
		}
		else
		{
			handleError(ladybugConvertImage(context, &image, arpBuffers, LADYBUG_BGRU16), "ladybugConvertImage()");
		}

		// Update textures
		printf("Updating textures...\n");
		handleError(ladybugUpdateTextures(context, LADYBUG_NUM_CAMERAS, (const unsigned char**)arpBuffers, LADYBUG_BGRU16), "ladybugUpdateTextures");
		char filename[_MAX_PATH] = { 0 };
		for (const double radius : { 0.5, 1.0,2.0,5.0,10.0,50.0,100.0, 1000.0})
		{
			handleError(ladybugSet3dMapSphereSize(context, radius));
			// Render panorama
			printf("Rendering panorama for radius %f \n", radius);
			handleError(ladybugRenderOffScreenImage(context, LADYBUG_PANORAMIC, LADYBUG_BGR16, &processedImage), "ladybugRenderOffScreenImage");

			// Save image
			sprintf(filename, "ladybugPostProcessing-panoramic-%d-radius_%f.jpg", i,radius);
			const std::string outputPath = directory + std::string(filename);
			handleError(ladybugSaveImage(context, &processedImage, outputPath.c_str(), LADYBUG_FILEFORMAT_JPG), "ladybugSaveImage");
		}


		
		printf("Enabling rendering image size...\n");


		const LadybugOutputImage images_to_render[] = { LADYBUG_RECTIFIED_CAM0 ,LADYBUG_RECTIFIED_CAM1,LADYBUG_RECTIFIED_CAM2,LADYBUG_RECTIFIED_CAM3,LADYBUG_RECTIFIED_CAM4,LADYBUG_RECTIFIED_CAM5 };
		for (auto cam = 0; cam < 6; cam++)
		{
			printf("Rendering rectified %d ...\n", cam);
			handleError(ladybugSetOffScreenImageSize(context, images_to_render[cam], 2448, 2048), "ladybugSetOffScreenImageSize");
			handleError(ladybugRenderOffScreenImage(context, images_to_render[cam], LADYBUG_BGR16, &processedImage), "ladybugRenderOffScreenImage");
			sprintf(filename, "ladybugPostProcess-rectified%d-%d.jpg", cam, i);
			const std::string outputPath = directory + std::string(filename);
			handleError(ladybugSaveImage(context, &processedImage, outputPath.c_str(), LADYBUG_FILEFORMAT_JPG), "ladybugSaveImage");
			printf("Conversion successful - saved %s\n", outputPath.c_str());
		}

	}

	printf("Stopping camera\n");
	handleError(ladybugStop(context), "ladybugStop()");

	// Destroy the context
	printf("Destroying context...\n");
	handleError(ladybugDestroyContext(&context), "ladybugDestroyContext()");

	// Clean up the buffers
	for (unsigned int uiCamera = 0; uiCamera < LADYBUG_NUM_CAMERAS; uiCamera++)
	{
		delete[] arpBuffers[uiCamera];
	}

	printf("Done.\n");

	return 0;
}

void setPostProcessingOptions(LadybugContext context)
{
	// Grab the current parameters
	LadybugAdjustmentParameters parameters;
	handleError(ladybugGetAdjustmentParameters(context, parameters));

	// Modify values
	//
	// There are other values that are not set here that may be valuable for you.
	// Please take a look at the LadybugAdjustmentParameters member variables in
	// ladybugImageAdjustment.h

	// Needs to be true to enable any post processing.
	parameters.doAdjustment = true;

	// Don't perform black level adjustment.
	parameters.blackLevelAdjustmentType = false;

	// Set automatic gain compensation.
	parameters.gainAdjustmentType = GAIN_AUTOMATIC_COMPENSATION;
	parameters.considerGammaInGainAdjustment = true;
	parameters.exposureCompensation = 0;

	// Set gamma.
	parameters.gammaAdjustmentType = true;
	parameters.gammaManualValue = 2.2f; // 1.8 is often used for Apple monitors

	// Set white balance.
	// Consider the following settings for (Red, Green):
	//   Sunny        = (-2.2, 1.0)
	//   Cloudy       = (-2.1, 0.3)
	//   Fluorescent  = (-4.3, 8.3)
	//   Incandescent = (-9.4, 9.7)
	parameters.whiteBalanceAdjustmentType = AUTOMATIC;
	parameters.gainRed_ManualValue = -2.2f;
	parameters.gainBlue_ManualValue = 1.0f;

	// Set the updated parameters (future calls to ladybugConvertImage() will take them into account).
	handleError(ladybugSetAdjustmentParameters(context, parameters));
}

void handleError(LadybugError error, const char* message)
{
	if (error != LADYBUG_OK)
	{

		if (message == NULL)
		{
			printf("Error: Ladybug library reported - %s\n", ::ladybugErrorToString(error));
		}
		else
		{
			printf("Error: Ladybug library reported in %s - %s\n", message, ::ladybugErrorToString(error));
		}
		throw(std::exception());
	}
}
