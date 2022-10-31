/*
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "image_converter.h"

#include <jetson-utils/cudaColorspace.h>
#include <jetson-utils/cudaWarp.h>
#include <jetson-utils/cudaMappedMemory.h>

#include <cv_bridge/cv_bridge.h>

#include <cuda_runtime_api.h>


static imageFormat imageFormatFromEncoding( const std::string& encoding )
{
	if( encoding == sensor_msgs::image_encodings::BGR8 )
		return IMAGE_BGR8;
	else if( encoding == sensor_msgs::image_encodings::BGRA8 )
		return IMAGE_BGRA8;
	else if( encoding == sensor_msgs::image_encodings::RGB8 )
		return IMAGE_RGB8;
	else if( encoding == sensor_msgs::image_encodings::RGBA8 )
		return IMAGE_RGBA8;
	else if( encoding == sensor_msgs::image_encodings::MONO8 )
		return IMAGE_GRAY8;
	else if( encoding == sensor_msgs::image_encodings::YUV422 )
		return IMAGE_UYVY;
	else if( encoding == sensor_msgs::image_encodings::BAYER_RGGB8 )
		return IMAGE_BAYER_RGGB;
	else if( encoding == sensor_msgs::image_encodings::BAYER_BGGR8 )
		return IMAGE_BAYER_BGGR;
	else if( encoding == sensor_msgs::image_encodings::BAYER_GBRG8 )
		return IMAGE_BAYER_GBRG;
	else if( encoding == sensor_msgs::image_encodings::BAYER_GRBG8 )
		return IMAGE_BAYER_GRBG;

	return IMAGE_UNKNOWN;
}

static std::string imageFormatToEncoding( imageFormat fmt )
{
	switch(fmt)
	{
		case IMAGE_BGR8:		return sensor_msgs::image_encodings::BGR8;
		case IMAGE_BGRA8:		return sensor_msgs::image_encodings::BGRA8;
		case IMAGE_RGB8:		return sensor_msgs::image_encodings::RGB8;
		case IMAGE_RGBA8:		return sensor_msgs::image_encodings::RGBA8;
		case IMAGE_GRAY8:		return sensor_msgs::image_encodings::MONO8;
		case IMAGE_UYVY:		return sensor_msgs::image_encodings::YUV422;
		case IMAGE_BAYER_RGGB:	return sensor_msgs::image_encodings::BAYER_RGGB8;
		case IMAGE_BAYER_BGGR:	return sensor_msgs::image_encodings::BAYER_BGGR8;
		case IMAGE_BAYER_GBRG:	return sensor_msgs::image_encodings::BAYER_GBRG8;
		case IMAGE_BAYER_GRBG:	return sensor_msgs::image_encodings::BAYER_GRBG8;
	}

	return "invalid";
}


// constructor
imageConverter::imageConverter(bool preflip_, bool undistort_) :
	preflip(preflip_), undistort(undistort_)
{
	mWidth  	  = 0;
	mHeight 	  = 0;
	mSizeInput  = 0;
	mSizeOutput = 0;

	mInputCPU = NULL;
	mInputGPU = NULL;

	mOutputCPU = NULL;
	mOutputGPU = NULL;
}


// destructor
imageConverter::~imageConverter()
{
	Free();	
}


// Free
void imageConverter::Free()
{
	if( mInputCPU != NULL )
	{
		CUDA(cudaFreeHost(mInputCPU));

		mInputCPU = NULL;
		mInputGPU = NULL;
	}

	if( mOutputCPU != NULL )
	{
		CUDA(cudaFreeHost(mOutputCPU));

		mOutputCPU = NULL;
		mOutputGPU = NULL;
	}
}

// Convert
bool imageConverter::Convert( const sensor_msgs::ImageConstPtr& input )
{
	ROS_DEBUG("converting %ux%u %s image", input->width, input->height, input->encoding.c_str());

	// parse the input format
	const imageFormat input_format = imageFormatFromEncoding(input->encoding);

	if( input_format == IMAGE_UNKNOWN )
	{
		ROS_ERROR("image encoding %s is not a compatible format to use with ros_deep_learning", input->encoding.c_str());
		return false;
	}

	// assure memory allocation
	if( !Resize(input->width, input->height, input_format) )
		return false;
	
	// copy input to shared memory
	memcpy(mInputCPU, input->data.data(), imageFormatSize(input_format, input->width, input->height));			
	
	// convert image format
	if( CUDA_FAILED(cudaConvertColor(mInputGPU, input_format, mOutputGPU, InternalFormat, input->width, input->height)) )
	{
		ROS_ERROR("failed to convert %ux%u image (from %s to %s) with CUDA", mWidth, mHeight, imageFormatToStr(input_format), imageFormatToStr(InternalFormat));
		return false;
	}

	return true;
}


// Convert
bool imageConverter::Convert( sensor_msgs::Image& msg, imageFormat format )
{
	return Convert(msg, format, ImageGPU(), sensor_msgs::CameraInfo());
}


// Convert
bool imageConverter::Convert( sensor_msgs::Image& msg, imageFormat format, PixelType* imageGPU, const sensor_msgs::CameraInfo& camera_info_msg )
{
	if( !mInputCPU || !imageGPU || mWidth == 0 || mHeight == 0 || mSizeInput == 0 || mSizeOutput == 0 )
		return false;
	
	void* primaryGPU = mInputGPU;
	void* primaryCPU = mInputCPU;
	void* secondaryGPU = mInputGPU2;
	void* secondaryCPU = mInputCPU2;

	cudaMemcpy(primaryGPU, static_cast<void*>(imageGPU), imageFormatSize(InternalFormat, mWidth, mHeight), cudaMemcpyDeviceToDevice);

	if( preflip )
	{
		const float transform[2][3]{
			{-1, 0, mWidth},
			{0, -1, mHeight}
		};

		if( !CUDA_FAILED(cudaWarpAffine(static_cast<PixelType*>(primaryGPU), static_cast<PixelType*>(secondaryGPU), mWidth, mHeight, transform)) )
		{
			void* tmp = primaryGPU;
			primaryGPU = secondaryGPU;
			secondaryGPU = tmp;
			tmp = primaryCPU;
			primaryCPU = secondaryCPU;
			secondaryCPU = tmp;
		}
		else
		{
			ROS_ERROR("failed preflip input image with CUDA");
			return false;
		}
	}

	cudaStreamSynchronize(cudaStreamLegacy);

	// perform image rectification: undistortion and changing camera intrinsics
	if( undistort )
	{
		// extract camera parameters from message
		const float fx = static_cast<float>(camera_info_msg.K[0]);
		const float fy = static_cast<float>(camera_info_msg.K[4]);
		const float cx = static_cast<float>(camera_info_msg.K[2]);
		const float cy = static_cast<float>(camera_info_msg.K[5]);

		const float k1 = static_cast<float>(camera_info_msg.D[0]);
		const float k2 = static_cast<float>(camera_info_msg.D[1]);
		const float p1 = static_cast<float>(camera_info_msg.D[2]);
		const float p2 = static_cast<float>(camera_info_msg.D[3]);

		if( !CUDA_FAILED(cudaWarpIntrinsic(static_cast<PixelType*>(primaryGPU), static_cast<PixelType*>(secondaryGPU), mWidth, mHeight, {fx, fy}, {cx, cy}, {k1, k2, p1, p2} )) )
		{
			void* tmp = primaryGPU;
			primaryGPU = secondaryGPU;
			secondaryGPU = tmp;
			tmp = primaryCPU;
			primaryCPU = secondaryCPU;
			secondaryCPU = tmp;
		}
		else
		{
			ROS_ERROR("failed to undistort input image with CUDA");
			return false;
		}
	}

	cudaStreamSynchronize(cudaStreamLegacy);

	// perform colorspace conversion into the desired encoding
	// in this direction, we reverse use of input/output pointers
	if( !CUDA_FAILED(cudaConvertColor(primaryGPU, InternalFormat, secondaryGPU, format, mWidth, mHeight)) )
	{
		void* tmp = primaryGPU;
		primaryGPU = secondaryGPU;
		secondaryGPU = tmp;
		tmp = primaryCPU;
		primaryCPU = secondaryCPU;
		secondaryCPU = tmp;
	}
	else
	{
		ROS_ERROR("failed to convert %ux%u image (from %s to %s) with CUDA", mWidth, mHeight, imageFormatToStr(InternalFormat), imageFormatToStr(format));
		return false;
	}

	// calculate size of the msg
	const size_t msg_size = imageFormatSize(format, mWidth, mHeight);

	// allocate msg storage
	msg.data.resize(msg_size);

	// copy the converted image into the msg
	memcpy(msg.data.data(), primaryCPU, msg_size);

	// populate metadata
	msg.width  = mWidth;
	msg.height = mHeight;
	msg.step   = (mWidth * imageFormatDepth(format)) / 8;

	msg.encoding     = imageFormatToEncoding(format);
	msg.is_bigendian = false;

	return true;
}


// Resize
bool imageConverter::Resize( uint32_t width, uint32_t height, imageFormat inputFormat )
{
	const size_t input_size  = imageFormatSize(inputFormat, width, height);
	const size_t output_size = imageFormatSize(InternalFormat, width, height);

	if( input_size != mSizeInput || output_size != mSizeOutput || mWidth != width || mHeight != height )
	{
		Free();

		if( !cudaAllocMapped((void**)&mInputCPU, (void**)&mInputGPU, input_size) ||
			!cudaAllocMapped((void**)&mInputCPU2, (void**)&mInputGPU2, input_size) ||
		    !cudaAllocMapped((void**)&mOutputCPU, (void**)&mOutputGPU, output_size) )
		{
			ROS_ERROR("failed to allocate memory for %ux%u image conversion", width, height);
			return false;
		}

		ROS_INFO("allocated CUDA memory for %ux%u image conversion", width, height);

		mWidth      = width;
		mHeight     = height;
		mSizeInput  = input_size;
		mSizeOutput = output_size;		
	}

	return true;
}
