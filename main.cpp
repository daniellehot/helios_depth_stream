//https://support.thinklucid.com/app-note-helios-3d-point-cloud-with-rgb-color/
//https://gist.github.com/zhou-chao/7a7de79de47c652196f1

#include "ArenaApi.h"
#include "SaveApi.h"

#include "signal.h"
#include "unistd.h"
#include "fstream"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"


#define TAB1 "  "			
#define TAB2 "    "
#define IMAGE_TIMEOUT 2000
#define NUM_IMAGES 25
#define PLY_FILE_NAME "pc.ply"
#define IMG_FILE_NAME "heatmap.jpg"
int FLAG_KILL_STREAM = 0;
std::string operating_mode;


//This function is used to safely stop a camera stream
void signal_callback_handler(int signum)
{
   std::cout << "Caught signal " << signum << std::endl;
   // Terminate program
   FLAG_KILL_STREAM = 1;
}


//Get a connected Helios camera or exit the program if no camera stis connected
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


//Visualise intensity image as a gray image
void show_gray_image(Arena::IDevice* pDevice)
{
	//set streaming parameters - pixel format, operating mode, and data transfer options
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", "Mono8");
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dOperatingMode", operating_mode.c_str());
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

	//setup nodes for smooth results
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ExposureTimeSelector", "Exp1000Us");
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ConversionGain", "Low");
	Arena::SetNodeValue<int64_t>(pDevice->GetNodeMap(), "Scan3dImageAccumulation", 4);
	Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "Scan3dSpatialFilterEnable", true);
	Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "Scan3dConfidenceThresholdEnable", true);


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


//calculate heatmap coloring based on the depth data of the input image
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
	double redColorBorder, yellowColorBorder, greenColorBorder, cyanColorBorder, blueColorBorder;

	if (operating_mode == "Distance1500mm")
	{
		redColorBorder = 0;
		yellowColorBorder = 375;
		greenColorBorder = 750; 
		cyanColorBorder = 1125;
		blueColorBorder = 1500;
	} else 
	{
		redColorBorder = 0;
		yellowColorBorder = 1500;
		greenColorBorder = 3000; 
		cyanColorBorder = 4500;
		blueColorBorder = 6000;
	}
	
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

	/* This code saves the heatmap as a jpg image */
	//Save::ImageParams jpgParams(width, height, dstBpp);
	//Save::ImageWriter jpgWriter(jpgParams, IMG_FILE_NAME);
	//jpgWriter << pHeatMapImage->GetData();
	//std::cout << TAB2 << "Save heatmap image as jpg to " << jpgWriter.GetLastFileName() << "\n";
	
	return pHeatMapImage;
}


//Get the intensity image and calculate heatmap for it
void show_heatmap_image(Arena::IDevice* pDevice)
{
	//set operating and streaming nodes
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", "Coord3D_ABCY16");
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dOperatingMode", "Distance1500mm");
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dCoordinateSelector", "CoordinateC");

	//sets all relevant nodes targeted towards getting smooth results
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ExposureTimeSelector", "Exp1000Us");
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ConversionGain", "Low");
	Arena::SetNodeValue<int64_t>(pDevice->GetNodeMap(), "Scan3dImageAccumulation", 4);
	Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "Scan3dSpatialFilterEnable", true);
	Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "Scan3dConfidenceThresholdEnable", true);

	//start stream and get an image
	pDevice->StartStream();
	Arena::IImage* pImage = pDevice->GetImage(IMAGE_TIMEOUT);

	//apply heatmap 
	float scale = static_cast<float>(Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "Scan3dCoordinateScale"));
	Arena::IImage* pHeatMapImage = apply_heatmap(pImage, scale);

	//create opencv Mat from Arena::IImage and visualise it 
	cv::Mat img = cv::Mat((int)pHeatMapImage->GetHeight(), (int)pHeatMapImage->GetWidth(), CV_8UC3, (void *)pHeatMapImage->GetData());
	GenICam::gcstring windowName = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "DeviceModelName");
	cv::imshow(windowName.c_str(), img);
    cv::waitKey(0);

	//clean up
	cv::destroyAllWindows();
    pDevice->RequeueBuffer(pImage);
	pDevice->StopStream();
}


//Save depth data as a ply file
void save_image_as_ply(Arena::IDevice* pDevice)
{	
	//set operating and streaming nodes
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", "Coord3D_ABC16");
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dOperatingMode", operating_mode.c_str());
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

	//setup nodes for smooth results
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ExposureTimeSelector", "Exp1000Us");
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ConversionGain", "Low");
	Arena::SetNodeValue<int64_t>(pDevice->GetNodeMap(), "Scan3dImageAccumulation", 4);
	Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "Scan3dSpatialFilterEnable", true);
	Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "Scan3dConfidenceThresholdEnable", true);
	
	//start stream and get an image
	pDevice->StartStream();
	Arena::IImage* pImage = pDevice->GetImage(IMAGE_TIMEOUT);

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
	Save::ImageWriter writer(params, PLY_FILE_NAME);

	// set default parameters for SetPly()
	bool filterPoints = true;
	bool isSignedPixelFormat = false;
	if ((pImage->GetPixelFormat() == Coord3D_ABC16s) || (pImage->GetPixelFormat() == Coord3D_ABCY16s))
	{
		isSignedPixelFormat = true;
	}
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
	writer << pImage->GetData();
	pDevice->StopStream();
}


//Stream intensity data as grayscale image
void stream_data(Arena::IDevice* pDevice)
{
	//Setup nodes for streaming
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", "Mono8");
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "AcquisitionMode", "Continuous");
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dOperatingMode", "Distance6000mm");
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetTLStreamNodeMap(), "StreamBufferHandlingMode", "NewestOnly");
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
	
	//Smooth results
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ExposureTimeSelector", "Exp250Us");
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ConversionGain", "High");
	//Arena::SetNodeValue<int64_t>(pDevice->GetNodeMap(), "Scan3dImageAccumulation", 4);
	Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "Scan3dSpatialFilterEnable", true);
	Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "Scan3dConfidenceThresholdEnable", true);

	GenICam::gcstring windowName = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "DeviceModelName");
	pDevice->StartStream();

	while(1)
	{
		Arena::IImage* pImage = pDevice->GetImage(IMAGE_TIMEOUT);
		//size_t size = pImage->GetSizeFilled();
		//size_t width = pImage->GetWidth();
		//size_t height = pImage->GetHeight();
		//GenICam::gcstring pixelFormat = GetPixelFormatName(static_cast<PfncFormat>(pImage->GetPixelFormat()));
		//uint64_t timestampNs = pImage->GetTimestampNs();

		//std::cout << " (" << size << " bytes; " << width << "x" << height << "; " << pixelFormat << "; timestamp (ns): " << timestampNs << ")";
		cv::Mat img = cv::Mat((int)pImage->GetHeight(), (int)pImage->GetWidth(), CV_8UC1, (void *)pImage->GetData());
		cv::imshow(windowName.c_str(), img);
		cv::waitKey(1);
		// Requeue image buffer
		//    Requeue an image buffer when access to it is no longer needed.
		//    Notice that failing to requeue buffers may cause memory to leak and
		//    may also result in the stream engine being starved due to there
		//    being no available buffers.
		pDevice->RequeueBuffer(pImage);
		if (FLAG_KILL_STREAM) break;
	}

	// Stop stream
	//    Stop the stream after all images have been requeued. Failing to stop
	//    the stream will leak memory.
	cv::destroyAllWindows();
	pDevice->StopStream();
}


void get_depth_map(Arena::IDevice* pDevice)
{
	
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", "Coord3D_ABCY16");
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dOperatingMode", "Distance1500mm");
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

	//setup nodes for smooth results
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ExposureTimeSelector", "Exp250Us");
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ConversionGain", "High");
	Arena::SetNodeValue<int64_t>(pDevice->GetNodeMap(), "Scan3dImageAccumulation", 4);
	Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "Scan3dSpatialFilterEnable", true);
	Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "Scan3dConfidenceThresholdEnable", true);

	// Read the scale factor and offsets to convert from unsigned 16-bit values 
	// in the Coord3D_ABCY16 pixel format to coordinates in mm
	double xyz_scale_mm = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "Scan3dCoordinateScale");
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dCoordinateSelector", "CoordinateA");
	double x_offset_mm = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "Scan3dCoordinateOffset");
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dCoordinateSelector", "CoordinateB");
	double y_offset_mm = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "Scan3dCoordinateOffset");
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dCoordinateSelector", "CoordinateC");
	double z_offset_mm = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "Scan3dCoordinateOffset");

	// Start stream, and get image
	pDevice->StartStream();
	Arena::IImage* image = pDevice->GetImage(2000);

	// Create two empty OpenCV images
	// one to encode depth coordinates (CV_32FC3) - three channels (one channel for each coordinate XYZ) of float data type (depth must be float) 
	// one for intensity (CV_16UC1) - single channel, 16bit unsigned
	size_t height, width;
	height = image->GetHeight();
	width = image->GetWidth();
	cv::Mat xyz_mm = cv::Mat((int)height, (int)width, CV_32FC3);
	cv::Mat intensity_image = cv::Mat((int)height, (int)width, CV_16UC1);

	// Get Helios XYZ data bytes and intensity data:
	const uint16_t* input_data;
	input_data = (uint16_t*)image->GetData();
	

	// Copy the image data into the empty OpenCV images
	for (unsigned int ir = 0; ir < height; ++ir)
	{
		for (unsigned int ic = 0; ic < width; ++ic)
		{
			// Get unsigned 16 bit values for X,Y,Z coordinates
			ushort x_u16 = input_data[0];
			ushort y_u16 = input_data[1];
			ushort z_u16 = input_data[2];

			// Convert 16-bit X,Y,Z to float values in mm
			xyz_mm.at<cv::Vec3f>(ir, ic)[0] = (float)(x_u16 * xyz_scale_mm + x_offset_mm);
			xyz_mm.at<cv::Vec3f>(ir, ic)[1] = (float)(y_u16 * xyz_scale_mm + y_offset_mm);
			xyz_mm.at<cv::Vec3f>(ir, ic)[2] = (float)(z_u16 * xyz_scale_mm + z_offset_mm);
			//std::cout<<"Pixel ("<<ir<<","<<ic<<") Distance "<<xyz_mm.at<cv::Vec3f>(ir, ic)[2]<<std::endl;
			// Get intensity values, these dont need to be converted (ushort == uint16_t)
			intensity_image.at<ushort>(ir, ic) = input_data[3]; 
			input_data += 4;
		}
	}
	pDevice->StopStream();
	std::cout<<"Distance to the center point "<<xyz_mm.at<cv::Vec3f>(240, 320)[2]<<std::endl;
	// Visualise the intensity image and draw a circle around the center point
	cv::Point center(320, 240);//Declaring the center point
	int radius = 5; //Declaring the radius
	cv::Scalar line_Color(0, 0, 0);//Color of the circle
	int thickness = 5;//thickens of the line
	cv::circle(intensity_image, center,radius, line_Color, thickness);//Using circle()function to draw the line//
	cv::imshow("HLS Intensity", intensity_image);
	cv::waitKey(0);
	cv::destroyAllWindows();

	// Optional: save point coordinates into csv files
	/* 
	cv::Mat channels[3];
	cv::split(xyz_mm, channels);
	std::ofstream myfile;
	std::vector<std::string> filenames = {"x.csv", "y.csv", "z.csv"};
	for (size_t i = 0; i < 3; i++)
	{
		myfile.open(filenames[i].c_str());
		myfile<< cv::format(channels[i], cv::Formatter::FMT_CSV) << std::endl;
		myfile.close();
	}
	*/
}


void print(int i){
	std::cout<<"Debug "<<i<<std::endl;
}

int main(int argc, char* argv[])
{
	// flag to track when an exception has been thrown
	bool exceptionThrown = false;
	signal(SIGINT, signal_callback_handler);

	if(argc>3){
		std::cout<< "Please insert only two input arguments. For help, run ./main -help \n";
		exit(1);
	} 
	print(1);

	if(argc<3 && std::string(argv[1]) != "-help"){
		std::cout<< "Please insert two inputs argument. For help, run ./main -help \n";
		exit(1); 
	}
	print(2);

	if (std::string(argv[1]) != "-help" && std::string(argv[1]) != "-gray" &&
	std::string(argv[1]) != "-heatmap" && std::string(argv[1]) != "-ply" &&
	std::string(argv[1]) != "-stream" && std::string(argv[1]) != "-depth") 
	{
		std::cout<< "Unrecognized input argument. For help, run ./main -help \n";
		exit(1);
	}
	print(3);

	if (std::string(argv[1]) == "-help"){
		std::cout<<"First select functionality. Input arguments are:\n"
		<<TAB1<<"-gray"<<TAB1<<"Capture an intensity image, and visualise it as a grayscale image using OpenCV.\n"
		<<TAB1<<"-heatmap"<<TAB1<<"Capture depth data image, apply heatmat based on the measured depth, and visualise it as an rgb image using OpenCV.\n"
		<<TAB1<<"-ply"<<TAB1<<"Capture depth data, save it as a ply file.\n"
		<<TAB1<<"-stream"<<TAB1<<"Stream intensity data.\n"
		<<TAB1<<"-depth"<<TAB1<<"Capture depth map and visualise the intensity data as a grayscale image.\n"
		<<TAB1<<"Then select the operating mode. Input arguments are :\n"
		<<TAB1<<"-6000"<<TAB1<<"Operating mode is set to Distance6000mm (Far Mode). This mode operates at 15 FPS.\n"
		<<TAB1<<"-1500"<<TAB1<<"Operating mode is set to Distance1500mm (Near Mode). This mode operates at 30 FPS.\n"
		<<TAB1<<"For example, ./main -stream -6000 will launch stream in a Far Mode.\n";
		exit(1);
	}
	print(4);
	
	if (std::string(argv[2]) != "-6000" && std::string(argv[2]) != "-1500")
	{
		std::cout<< "Unrecognized input argument. For help, run ./main -help \n";
		exit(1);
	}
	print(5);

	if (std::string(argv[2]) == "-6000") operating_mode = "Distance6000mm";
	if (std::string(argv[2]) == "-1500") operating_mode = "Distance1500mm";
	print(6);
	
	try
	{
		// run example
		std::cout << "Commence example\n\n";

        // get a connected camera device
		Arena::ISystem* system = Arena::OpenSystem();
		Arena::IDevice* camera = get_device(system);
		
		if (std::string(argv[1]) == "-gray") show_gray_image(camera);
		if (std::string(argv[1]) == "-heatmap") show_heatmap_image(camera);
		if (std::string(argv[1]) == "-ply") save_image_as_ply(camera);
		if (std::string(argv[1]) == "-stream") stream_data(camera);
		if (std::string(argv[1]) == "-depth") get_depth_map(camera);
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

	if (exceptionThrown)
		return -1;
	else
		return 0;
}