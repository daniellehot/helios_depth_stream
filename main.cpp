//https://support.thinklucid.com/app-note-helios-3d-point-cloud-with-rgb-color/#gs-opencv2
//https://support.thinklucid.com/using-opencv-with-arena-sdk-on-linux/
//#include "stdafx.h"
#include "ArenaApi.h"
#include "SaveApi.h"

#include "signal.h"
#include "unistd.h"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"

#define TAB1 "  "			
#define TAB2 "    "
#define TAB3 "      "

// image timeout
#define IMAGE_TIMEOUT 2000

struct PointData
{
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t intensity;
};

void signal_callback_handler(int signum)
{
   std::cout << "Caught signal " << signum << std::endl;
   // Terminate program
   exit(signum);
}

void set_node_values(Arena::IDevice* pDevice)
{
	//GenApi::INodeMap* pNodeMap = pDevice->GetNodeMap();
	//GenICam::gcstring operatingModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode");
    //std::cout<<"Operation mode "<<operatingModeInitial<<std::endl;

	// change operating mode from Far Mode to Near Mode
    Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dOperatingMode", "Distance1500mm");
	// enable stream auto negotiate packet size
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
	// enable stream packet resend
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
	
}

void stream_data(Arena::IDevice* pDevice)
{
	// get image
	pDevice->StartStream();
	while (true){
		Arena::IImage* pImage = pDevice->GetImage(IMAGE_TIMEOUT);
		std::cout<<pImage<<std::endl;
        //sleep(1.0);
	}
}


void show_gray_image(Arena::IDevice* pDevice)
{
	//set streaming parameters - pixel format, operating mode, and data transfer options
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", "Mono8");
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dOperatingMode", "Distance1500mm");
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
	//start stream and get an image
	pDevice->StartStream();
	Arena::IImage* pImage = pDevice->GetImage(IMAGE_TIMEOUT);
	//convert image to cv::Mat and visualise
	cv::Mat img_gray = cv::Mat((int)pImage->GetHeight(), (int)pImage->GetWidth(), CV_8UC1, (void *)pImage->GetData());
	GenICam::gcstring windowName = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "DeviceModelName");
	cv::imshow(windowName.c_str(), img_gray);
    cv::waitKey(0);
	//clean up
	cv::destroyAllWindows();
    pDevice->RequeueBuffer(pImage);
	pDevice->StopStream();
}


Arena::IImage* apply_heatmap(Arena::IImage* pImage, float scale)
{
	// prepare info from input buffer
	size_t width = pImage->GetWidth();
	size_t height = pImage->GetHeight();
	size_t size = width * height;
	size_t srcBpp = pImage->GetBitsPerPixel();
	size_t srcPixelSize = srcBpp / 8; // divide by the number of bits in a byte
	const uint8_t* pInput = pImage->GetData();

	// prepare memory output buffer 
	size_t dstBpp = Arena::GetBitsPerPixel(BGR8);
	size_t dstPixelSize = dstBpp / 8; // divide by the number of bits in a byte
	size_t dstDataSize = width * height * dstBpp / 8; // divide by the number of bits in a byte
	uint8_t* pOutput = new uint8_t[dstDataSize];
	memset(pOutput, 0, dstDataSize);

	// Prepare coloring buffer for ply image
	//    Saving ply with color takes RGB coloring compared to the BGR coloring
	//    the jpg image uses, therefore we need a separate buffer for this data.
	uint8_t* pColoring = new uint8_t[dstDataSize];
	memset(pColoring, 0, dstDataSize);
	uint8_t* pColor = pColoring;

	// manually convert to BGR image
	const uint8_t* pIn = pInput;
	uint8_t* pOut = pOutput;

	const double RGBmin = 0;
	const double RGBmax = 255;

	double redColorBorder = 0;
	double yellowColorBorder = 375;
	double greenColorBorder = 750; 
	double cyanColorBorder = 1125;
	double blueColorBorder = 1500;
	
	// iterate through each pixel and assign a color to it according to a distance
	for (size_t i = 0; i < size; i++)
	{
		// Isolate the z data
		//    The first channel is the x coordinate, second channel is the y
		//    coordinate, the third channel is the z coordinate (which is what we
		//    will use to determine the coloring) and the fourth channel is
		//    intensity.
		int16_t z = *reinterpret_cast<const int16_t*>((pIn + 4));

		// Convert z to millimeters
		//    The z data converts at a specified ratio to mm, so by multiplying
		//    it by the Scan3dCoordinateScale for CoordinateC, we are able to
		//    convert it to mm and can then compare it to the maximum distance of
		//    1500mm (in this case 3000mm for Helios2).
		z = int16_t(double(z) * scale);

		double coordinateColorBlue = 0.0;
		double coordinateColorGreen = 0.0;
		double coordinateColorRed = 0.0;

		// colors between red and yellow
		if ((z >= redColorBorder) && (z <= yellowColorBorder))
		{
				double yellowColorPercentage = z / yellowColorBorder;

				coordinateColorBlue = RGBmin;
				coordinateColorGreen = RGBmax * yellowColorPercentage;
				coordinateColorRed = RGBmax;
		}

		// colors between yellow and green
		else if ((z > yellowColorBorder) && (z <= greenColorBorder))
		{
				double greenColorPercentage = (z - yellowColorBorder) / yellowColorBorder;

				coordinateColorBlue = RGBmin;
				coordinateColorGreen = RGBmax;
				coordinateColorRed = RGBmax - RGBmax * greenColorPercentage;
		}

		// colors between green and cyan
		else if ((z > greenColorBorder) && (z <= cyanColorBorder))
		{
				double cyanColorPercentage = (z - greenColorBorder) / yellowColorBorder;

				coordinateColorBlue = RGBmax * cyanColorPercentage;
				coordinateColorGreen = RGBmax;
				coordinateColorRed = RGBmin;
		}

		// colors between cyan and blue
		else if ((z > cyanColorBorder) && (z <= blueColorBorder))
		{
				double blueColorPercentage = (z - cyanColorBorder) / yellowColorBorder;

				coordinateColorBlue = RGBmax;
				coordinateColorGreen = RGBmax - RGBmax * blueColorPercentage;
				coordinateColorRed = RGBmin;
		}
		else
		{
				coordinateColorBlue = RGBmin;
				coordinateColorGreen = RGBmin;
				coordinateColorRed = RGBmin;
		}

		// set pixel format values and move to next pixel
		*pOut = static_cast<int8_t>(coordinateColorBlue);
		*(pOut + 1) = static_cast<int8_t>(coordinateColorGreen);
		*(pOut + 2) = static_cast<int8_t>(coordinateColorRed);

		pIn += srcPixelSize;
		pOut += dstPixelSize;

		// set RGB pixel coloring for ply
		*pColor = static_cast<int8_t>(coordinateColorRed);
		*(pColor + 1) = static_cast<int8_t>(coordinateColorGreen);
		*(pColor + 2) = static_cast<int8_t>(coordinateColorBlue);
		pColor += dstPixelSize;
	}
	std::cout << TAB2 << "Create BGR heatmap using z data from 3D image\n";
	Arena::IImage* pHeatMapImage = Arena::ImageFactory::Create(pOutput, dstDataSize, width, height, BGR8);

	Save::ImageParams jpgParams(width, height, dstBpp);
	Save::ImageWriter jpgWriter(jpgParams, "heatmap.jpg");
	jpgWriter << pHeatMapImage->GetData();
	std::cout << TAB2 << "Save heatmap image as jpg to " << jpgWriter.GetLastFileName() << "\n";
	
	return pHeatMapImage;
}

void show_heatmap_image(Arena::IDevice* pDevice)
{
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", "Coord3D_ABCY16");
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dOperatingMode", "Distance1500mm");
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dCoordinateSelector", "CoordinateC");

	pDevice->StartStream();
	Arena::IImage* pImage = pDevice->GetImage(IMAGE_TIMEOUT);
	float scale = static_cast<float>(Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "Scan3dCoordinateScale"));
	Arena::IImage* pHeatMapImage = apply_heatmap(pImage, scale);
	cv::Mat img = cv::Mat((int)pHeatMapImage->GetHeight(), (int)pHeatMapImage->GetWidth(), CV_8UC3, (void *)pHeatMapImage->GetData());
	GenICam::gcstring windowName = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "DeviceModelName");
	cv::imshow(windowName.c_str(), img);
    cv::waitKey(0);
	//clean up
	cv::destroyAllWindows();
    pDevice->RequeueBuffer(pImage);
	pDevice->StopStream();
}

Arena::IDevice* get_device(Arena::ISystem* pSystem)
{
	pSystem->UpdateDevices(100);
	std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
	if (deviceInfos.size() == 0)
	{
		std::cout << "\nNo camera connected\nPress enter to complete\n";
		std::getchar();
		return 0;
	}
	Arena::IDevice* pDevice = pSystem->CreateDevice(deviceInfos[0]);
	return pDevice;
}

int main()
{
	// flag to track when an exception has been thrown
	bool exceptionThrown = false;
	signal(SIGINT, signal_callback_handler);
	
	try
	{
		// run example
		std::cout << "Commence example\n\n";

        // get a connected camera device
		Arena::ISystem* system = Arena::OpenSystem();
		Arena::IDevice* camera = get_device(system);

		// setup streaming parameters 
		//set_node_values(camera);

		// stream data
        //stream_data(camera);
		
		//show_gray_image(camera);
		show_heatmap_image(camera);

		// clean up
		system->DestroyDevice(camera);
		Arena::CloseSystem(system);
		std::cout << "\nExample complete\n";
	}
	catch (GenICam::GenericException& ge)
	{
		std::cout << "\nGenICam exception thrown: " << ge.what() << "\n";
		exceptionThrown = true;
	}
	catch (std::exception& ex)
	{
		std::cout << "Standard exception thrown: " << ex.what() << "\n";
		exceptionThrown = true;
	}
	catch (...)
	{
		std::cout << "Unexpected exception thrown\n";
		exceptionThrown = true;
	}

	//std::cout << "Press enter to complete\n";
	//std::getchar();

	if (exceptionThrown)
		return -1;
	else
		return 0;
}