/**
* This file was once part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <opencv2/core/core.hpp>

#include "nlohmann/json.hpp"

#include "Camera.h"
#include "ImageSize.h"

#include <tinyxml2.h>

namespace libvideoio {

class Undistorter
{
public:
	virtual ~Undistorter() {}

	/**
	 * Undistorts the given image and returns the result image.
	 */
	virtual void undistort(const cv::Mat &image, cv::OutputArray result) const = 0;

	virtual void undistortDepth( const cv::Mat &depth, cv::OutputArray result) const { depth.copyTo( result ); }

	/**
	 * Returns the intrinsic parameter matrix of the undistorted images.
	 */
	virtual const cv::Mat getK() const = 0;

	virtual const Camera getCamera() const  { return Camera(getK()); }

	/**
	 * Returns the intrinsic parameter matrix of the original images,
	 */
	virtual const cv::Mat getOriginalK() const = 0;

	virtual ImageSize inputImageSize( void ) const = 0;
	virtual int getInputWidth() const { return inputImageSize().width; };
	virtual int getInputHeight() const { return inputImageSize().height; };

	virtual ImageSize outputImageSize( void ) const { return inputImageSize(); };
	virtual int getOutputWidth() const { return outputImageSize().width; };
	virtual int getOutputHeight() const { return outputImageSize().height; };

	/**
	 * Returns if the undistorter was initialized successfully.
	 */
	virtual bool isValid() const = 0;

};


// An OpenCVUndistorter uses the OpenCV map() function for
// undistortion and takes the standard 4- (or more) coefficient
// tangential distortion model
class OpenCVUndistorter : public Undistorter
{
public:

	OpenCVUndistorter( const cv::Mat &k,
											const cv::Mat &distCoeff,
											const ImageSize &origSize );

	virtual ~OpenCVUndistorter();

	/**
	 * Undistorts the given image and returns the result image.
	 */
	virtual void undistort(const cv::Mat &image, cv::OutputArray result) const;
	virtual void undistortDepth( const cv::Mat &depth, cv::OutputArray result) const { depth.copyTo( result ); }

	/**
	 * Returns the intrinsic parameter matrix of the undistorted images.
	 */
	const cv::Mat getK() const { return _K; }

	/**
	 * Returns the intrinsic parameter matrix of the original images,
	 */
	const cv::Mat getOriginalK() const { return _originalK; }

	virtual ImageSize inputImageSize() const  { return _inputSize; }
	virtual ImageSize outputImageSize() const  { return _outputSize; }

	/**
	 * Returns if the undistorter was initialized successfully.
	 */
	bool isValid() const   { return _valid; }

protected:

	cv::Mat _K, _originalK;
	cv::Mat _distCoeffs;

	ImageSize _inputSize, _outputSize;
	cv::Mat _map1, _map2;

	/// true if the undistorter object is valid (has been initialized with
	/// a valid configuration)
	bool _valid;

};


class UndistorterFactory {
public:
	/**
	 * This function attempts to auto-detect the calibration file type.
	 * Creates and returns an Undistorter of the type used by the given
	 * configuration file. If the format is not recognized, returns nullptr.
	 */
	static Undistorter* getUndistorterFromFile(const std::string &configFilename);

};

// Each of these input files can be mapped to the OpenCV distortion model
class PhotoscanXMLUndistorterFactory : public UndistorterFactory {
public:
	static OpenCVUndistorter *loadFromFile( const std::string &filename );
	static OpenCVUndistorter *loadFromXML( tinyxml2::XMLDocument &doc, const std::string&filename = "" );
};

class PTAMUndistorterFactory : public UndistorterFactory {
public:
	static OpenCVUndistorter *loadFromFile( const std::string &filename );
	static OpenCVUndistorter *loadFromJSON( const nlohmann::json &json );
};

class OpenCVUndistorterFactory : public UndistorterFactory {
public:
	static OpenCVUndistorter *loadFromFile( const std::string &filename );
};



//=== Legacy Undistorter classes ===



class PTAMUndistorter : public Undistorter
{
public:
	/**
	 * Creates an Undistorter by reading the distortion parameters from a file.
	 *
	 * The file format is as follows:
	 * d1 d2 d3 d4 d5
	 * inputWidth inputHeight
	 * crop / full / none
	 * outputWidth outputHeight
	 */
	PTAMUndistorter(const char* configFileName);

	/**
	 * Destructor.
	 */
	~PTAMUndistorter();

	PTAMUndistorter(const PTAMUndistorter&) = delete;
	PTAMUndistorter& operator=(const PTAMUndistorter&) = delete;

	/**
	 * Undistorts the given image and returns the result image.
	 */
	void undistort(const cv::Mat &image, cv::OutputArray result) const;

	/**
	 * Returns the intrinsic parameter matrix of the undistorted images.
	 */
	const cv::Mat getK() const;

	virtual const Camera getCamera() const;


	/**
	 * Returns the intrinsic parameter matrix of the original images,
	 */
	const cv::Mat getOriginalK() const;

	/**
	 * Returns the width of the undistorted images in pixels.
	 */
	int getOutputWidth() const;

	/**
	 * Returns the height of the undistorted images in pixels.
	 */
	int getOutputHeight() const;


	virtual ImageSize inputImageSize( void ) const
		{ return ImageSize( getInputWidth(), getInputHeight() ); }

	/**
	 * Returns the width of the input images in pixels.
	 */
	int getInputWidth() const;

	/**
	 * Returns the height of the input images in pixels.
	 */
	int getInputHeight() const;


	/**
	 * Returns if the undistorter was initialized successfully.
	 */
	bool isValid() const;

private:
	cv::Mat K_;
	cv::Mat originalK_;

	float inputCalibration[5];
	float outputCalibration[5];
	int out_width, out_height;
	int in_width, in_height;
	float* remapX;
	float* remapY;


	/// Is true if the undistorter object is valid (has been initialized with
	/// a valid configuration)
	bool valid;
};


// class UndistorterOpenCV : public Undistorter
// {
// public:
// 	/**
// 	 * Creates an Undistorter by reading the distortion parameters from a file.
// 	 *
// 	 * The file format is as follows:
// 	 * fx fy cx cy d1 d2 d3 d4 d5 d6
// 	 * inputWidth inputHeight
// 	 * crop / full / none
// 	 * outputWidth outputHeight
// 	 */
// 	UndistorterOpenCV(const char* configFileName);
//
// 	/**
// 	 * Destructor.
// 	 */
// 	~UndistorterOpenCV();
//
// 	UndistorterOpenCV(const UndistorterOpenCV&) = delete;
// 	UndistorterOpenCV& operator=(const UndistorterOpenCV&) = delete;
//
// 	/**
// 	 * Undistorts the given image and returns the result image.
// 	 */
// 	void undistort(const cv::Mat &image, cv::OutputArray result) const;
//
// 	/**
// 	 * Returns the intrinsic parameter matrix of the undistorted images.
// 	 */
// 	const cv::Mat getK() const;
//
// 	virtual const Camera getCamera() const;
//
// 	/**
// 	 * Returns the intrinsic parameter matrix of the original images,
// 	 */
// 	const cv::Mat getOriginalK() const;
//
// 	/**
// 	 * Returns the width of the undistorted images in pixels.
// 	 */
// 	int getOutputWidth() const;
//
// 	/**
// 	 * Returns the height of the undistorted images in pixels.
// 	 */
// 	int getOutputHeight() const;
//
//
// 	/**
// 	 * Returns the width of the input images in pixels.
// 	 */
// 	int getInputWidth() const;
//
// 	/**
// 	 * Returns the height of the input images in pixels.
// 	 */
// 	int getInputHeight() const;
//
// 	/**
// 	 * Returns if the undistorter was initialized successfully.
// 	 */
// 	bool isValid() const;
//
// private:
// 	cv::Mat K_;
// 	cv::Mat originalK_;
//
// 	float inputCalibration[10];
// 	float outputCalibration;
// 	int out_width, out_height;
// 	int in_width, in_height;
// 	cv::Mat map1, map2;
//
// 	/// Is true if the undistorter object is valid (has been initialized with
// 	/// a valid configuration)
// 	bool valid;
// };
//
//
// class UndistorterLogger : public Undistorter
// {
// public:
// 	/**
// 	 * Creates an Undistorter by reading the distortion parameters from a file.
// 	 *
// 	 * The file format is as follows:
// 	 * fx fy cx cy
// 	 * inputWidth inputHeight
// 	 * cropWidth cropHeight
// 	 * outputWidth outputHeight
// 	 */
// 	UndistorterLogger(const char* configFileName);
//
// 	/**
// 	 * Destructor.
// 	 */
// 	~UndistorterLogger();
//
// 	UndistorterLogger(const UndistorterLogger&) = delete;
// 	UndistorterLogger& operator=(const UndistorterLogger&) = delete;
//
// 	void undistort(const cv::Mat &image, cv::OutputArray result) const;
// 	virtual void undistortDepth( const cv::Mat &depth, cv::OutputArray result) const;
//
//
// 	const cv::Mat getK() const;
// 	virtual const Camera getCamera() const;
// 	virtual const Camera getOriginalCamera( void ) const { return _originalCamera; }
//
//
// 	const cv::Mat getOriginalK() const;
// 	int getOutputWidth() const						{ return _finalSize.width; }
// 	int getOutputHeight() const						{ return _finalSize.height; }
// 	int getInputWidth() const							{ return _inputSize.width; }
// 	int getInputHeight() const						{ return _inputSize.height; }
//
//
// 	bool isValid() const { return _valid; }
//
// protected:
//
// 	UndistorterLogger( const ImageSize &inputSize, const Camera &cam  );
//
// 	ImageSize _inputSize, _cropSize, _finalSize;
// 	Camera _originalCamera;
//
// 	bool _valid;
// };

}
