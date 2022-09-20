//https://support.thinklucid.com/app-note-helios-3d-point-cloud-with-rgb-color/#gs-opencv2
//https://support.thinklucid.com/using-opencv-with-arena-sdk-on-linux/
//#include "stdafx.h"
#include "ArenaApi.h"

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


void apply_heatmap(Arena::IImage* pImage)
{
	// prepare info from input buffer
	size_t width = pImage->GetWidth();
	size_t height = pImage->GetHeight();
	size_t size = width * height;
	size_t srcBpp = pImage->GetBitsPerPixel();
	size_t srcPixelSize = srcBpp / 8; // divide by the number of bits in a byte
	const uint8_t* pInput = pImage->GetData();

}

void show_heatmap_image(Arena::IDevice* pDevice)
{
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", "Coord3D_ABCY16");
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dOperatingMode", "Distance1500mm");
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

	pDevice->StartStream();
	Arena::IImage* pImage = pDevice->GetImage(IMAGE_TIMEOUT);
	apply_heatmap(pImage);

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
		show_gray_image(camera);
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