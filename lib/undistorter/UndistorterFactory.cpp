/**
* This file is part of LSD-SLAM.
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

#include "libvideoio/Undistorter.h"

#include "nlohmann/json.hpp"
#include <tinyxml2.h>

#include <fstream>

namespace libvideoio
{


Undistorter* UndistorterFactory::getUndistorterFromFile(const std::string &configFileName, const std::shared_ptr<Undistorter> & wrap )
{
	LOG(INFO) << "Attempting to determine type calibration from file ..." << configFileName;

	std::ifstream f(configFileName);

	try {
		// Try to parse it as JSON
		nlohmann::json j;
		f >> j;

		if( j.count("calib_type") > 0 ) {
			LOG(INFO) << "   ... believe file is PTAM JSON format";
			return PTAMUndistorterFactory::loadFromJSON(j, wrap );
		}
	} catch( nlohmann::json::parse_error e ) {
		LOG(INFO) << "Error parsing JSON, must not be a PTAM JSON model: " << e.what();
	}

	tinyxml2::XMLDocument doc;
	doc.LoadFile( configFileName.c_str() );
	auto topNode = doc.FirstChildElement( "calibration" );

	if( topNode ) {
		LOG(INFO) << "   ... believe file is Photoscan XML format";
		return PhotoscanXMLUndistorterFactory::loadFromXML( doc, configFileName, wrap );
	}

	// If all else fails...
	LOG(INFO) << "Hm, all else has failed, try the PTAM Undistorter";
	return new PTAMUndistorter(configFileName.c_str());



	// float ic[10];
	// if(std::sscanf(l1.c_str(), "%f %f %f %f %f %f %f %f",
	// 		&ic[0], &ic[1], &ic[2], &ic[3], &ic[4],
	// 		&ic[5], &ic[6], &ic[7]) == 8)
	// {
	// 	LOG(INFO) << "found OpenCV camera model, building rectifier.";
	//
	// 	return OpenCVUndistorterFactory::loadFromFile(configFileName);
	// }
	// else if(std::sscanf(l1.c_str(), "%f %f %f %f %f",
	// 			&ic[0], &ic[1], &ic[2], &ic[3], &ic[4]) == 5)
	// {
	// 	LOG(INFO) << "found PTAM camera model, building rectifier.";
    //
		// Undistorter* u = new UndistorterPTAM(configFileName.c_str());
		// if(!u->isValid()) return 0;
		// return u;
	// }
	// else if(std::sscanf(l1.c_str(), "%f %f %f %f",
	// 			&ic[0], &ic[1], &ic[2], &ic[3]) == 4)
	// {
	// 	LOG(INFO) << "Found Logger camera model, building rectifier.";
	// 	// Undistorter* u = new UndistorterLogger(configFileName.c_str());
	// 	// if(!u->isValid()) return 0;
	// 	// return u;
	// } else	{
	// 	LOG(INFO) << "Found ATAN camera model, building rectifier.";
	// 	// Undistorter* u = new UndistorterPTAM(configFileName.c_str());
	// 	// if(!u->isValid()) return 0;
	// 	// return u;
	// }


	// if (!f.good())
	// {
	// 	LOG(FATAL) << " ... not found. Cannot operate without calibration, shutting down.";
	// 	f.close();
	// 	return NULL;
	// }
	//
	// std::string l1;
	// std::getline(f,l1);
	// f.close();
	//
	//
	//


	LOG(INFO) << "Unable to figure out calibration file type, giving up";
	return nullptr;
}

}
