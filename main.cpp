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
#define IMAGE_TIMEOUT 2000
#define PLY_FILE_NAME "pc.ply"
#define IMG_FILE_NAME "heatmap.jpg"


void signal_callback_handler(int signum)
{
   std::cout << "Caught signal " << signum << std::endl;
   // Terminate program
   exit(signum);
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
	Save::ImageWriter jpgWriter(jpgParams, IMG_FILE_NAME);
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


void save_image_as_ply(Arena::IDevice* pDevice)
{
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", "Coord3D_ABC16");
	//other pixel format options for depth data are Coord3D_ABC16s, Coord3D_ABCY16, Coord3D_ABCY16s
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dOperatingMode", "Distance1500mm");
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
	
	pDevice->StartStream();
	Arena::IImage* pImage = pDevice->GetImage(IMAGE_TIMEOUT);

	bool isSignedPixelFormat = false;

	if ((pImage->GetPixelFormat() == Coord3D_ABC16s) || (pImage->GetPixelFormat() == Coord3D_ABCY16s))
	{
		isSignedPixelFormat = true;
	}

	// Prepare image parameters
	//    An image's width, height, and bits per pixel are required to save to
	//    disk. Its size and stride (i.e. pitch) can be calculated from those 3
	//    inputs. Notice that an image's size and stride use bytes as a unit
	//    while the bits per pixel uses bits.

	Save::ImageParams params(pImage->GetWidth(), pImage->GetHeight(), pImage->GetBitsPerPixel());

	// Prepare image writer
	//    The image writer requires 3 arguments to save an image: the image's
	//    parameters, a specified file name or pattern, and the image data to
	//    save. Providing these should result in a successfully saved file on the
	//    disk. Because an image's parameters and file name pattern may repeat,
	//    they can be passed into the image writer's constructor.
	std::cout << TAB1 << "Prepare image writer\n";

	Save::ImageWriter writer(params, PLY_FILE_NAME);

	// set default parameters for SetPly()
	bool filterPoints = true;
	float scale = 0.25f;
	float offsetA = 0.0f;
	float offsetB = 0.0f;
	float offsetC = 0.0f;

	// set the output file format of the image writer to .ply
	writer.SetPly(".ply", filterPoints, isSignedPixelFormat, scale, offsetA, offsetB, offsetC);

	// Save image
	//    Passing image data into the image writer using the cascading I/O
	//    operator (<<) triggers a save. Notice that the << operator accepts the
	//    image data as a constant unsigned 8-bit integer pointer (const
	//    uint8_t*) and the file name as a character string (const char*).
	std::cout << TAB1 << "Save ply data\n";

	writer << pImage->GetData();
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

		// stream data
		//show_gray_image(camera);
		//show_heatmap_image(camera);
		//save_image_as_ply(camera);
		//stream_data(camera);

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