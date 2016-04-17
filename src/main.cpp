////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////



#include <pxcsensemanager.h>
#include <pxcmetadata.h>
#include <pxccapture.h>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <algorithm>
#include <string>

void ConvertPXCImageToOpenCVMat(PXCImage *inImg, cv::Mat *outImg);


void enumerateDevices() {

	PXCSession *session = PXCSession::CreateInstance();
	
	PXCSession::ImplVersion ver = session->QueryVersion();
	std::cout << "SDK Version " << ver.major << "," << ver.minor << std::endl;;
	
	
	
	PXCSession::ImplDesc desc1 = {};
	desc1.group = PXCSession::IMPL_GROUP_SENSOR;
	desc1.subgroup = PXCSession::IMPL_SUBGROUP_VIDEO_CAPTURE;

	for (int m = 0;; m++) {
		PXCSession::ImplDesc desc2;
		if (session->QueryImpl(&desc1, m, &desc2)<PXC_STATUS_NO_ERROR) break;
		std::cout <<  "Module["<<m<<"]: " <<desc2.friendlyName << std::endl;

		PXCCapture *capture = 0;
		pxcStatus sts = session->CreateImpl<PXCCapture>(&desc2, &capture);
		if (sts<PXC_STATUS_NO_ERROR) continue;

		// print out all device information
		for (int d = 0;; d++) {
			PXCCapture::DeviceInfo dinfo;
			sts = capture->QueryDeviceInfo(d, &dinfo);
			if (sts<PXC_STATUS_NO_ERROR) break;

			wprintf_s(L"    Device[%d]: %s\n", d, dinfo.name);
		}

		capture->Release();
	}

	session->Release();
}

void choosingDevice() {
	PXCSession *session;
	
	// Create a session
	session = PXCSession::CreateInstance();

	// Release devices (not really needed now but conceptual step)
	//if (device) device->Release();
	//device = 0;
	
	// Init capture manager
	PXCCaptureManager *captureManager;
	captureManager = session->CreateCaptureManager();
	if (!captureManager) {
		// Error
	}

	PXCSession::ImplDesc mdesc = {};
	mdesc.group = PXCSession::IMPL_GROUP_SENSOR;
	mdesc.subgroup = PXCSession::IMPL_SUBGROUP_VIDEO_CAPTURE;
	PXCCapture::DeviceInfo depthDevInfo;

	if (session) {
		for (int m = 0, iitem = 0;; m++) {
			PXCSession::ImplDesc desc1;
			if (session->QueryImpl(&mdesc, m, &desc1) < PXC_STATUS_NO_ERROR) break;

			PXCCapture* capture = 0;
			if (session->CreateImpl<PXCCapture>(&desc1, &capture) < PXC_STATUS_NO_ERROR) continue;

			for (int j = 0;; j++) {
				PXCCapture::DeviceInfo dinfo;
				if (capture->QueryDeviceInfo(j, &dinfo) < PXC_STATUS_NO_ERROR) break;
				std::string devName((wchar_t*)dinfo.name, (wchar_t*)dinfo.name + 224);
				std::transform(devName.begin(), devName.end(), devName.begin(), tolower);
				//std::cout << devName << std::endl;
				if (devName.find("rgb") != std::string::npos) {
					std::cout << "FOUND depth camera!. " << devName << std::endl;
					depthDevInfo = dinfo;
				}
			}
			capture->Release();
		}
	}

	/////// STREAMING!
	bool sts = true;

	/* Create a PXCSenseManager instance */
	PXCSenseManager *sm = PXCSenseManager::CreateInstance();
	if (!sm) {
		return;
		//ERROR
	}

	sm->EnableStream(PXCCapture::STREAM_TYPE_COLOR, 0, 0, 0);
	//sm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, 0, 0, 0);

	/* Optional: Set file name for playback or recording */
	PXCCaptureManager *cm = sm->QueryCaptureManager();
	//if (record && file[0]) cm->SetFileName(file, true); RECORD TO A FILE
	//if (playback && file[0]) cm->SetFileName(file, false); READ FROM A FILE

	/* Optional: Set device source */
	//if (!playback) cm->FilterByDeviceInfo(&dinfo);	
	cm->FilterByDeviceInfo(&depthDevInfo);

	while (true) {

		/* Initialization */
		if (sm->Init(/*this*/) < PXC_STATUS_NO_ERROR) {
			//UpdateStatus(L"Init Failed");
			sts = false;
			return;	//	error
		}

		/* Optional but recommended: Reset all properties */
		PXCCapture::Device *device = cm->QueryDevice();
		device->ResetProperties(PXCCapture::STREAM_TYPE_ANY);

		// Search profiles
		static const int IDXM_DEVICE = 0;
		PXCCapture::Device::StreamProfileSet chosenProfiles = {};
		for (int s = 0, mi = IDXM_DEVICE + 1; s<PXCCapture::STREAM_LIMIT; s++) {
			PXCCapture::StreamType st = PXCCapture::StreamTypeFromIndex(s);
			if (!(depthDevInfo.streams & st)) continue;
		
			// Get num of profiles
			int nprofiles = device->QueryStreamProfileSetNum(st);

			// Print out profiles
			for (int p = 0; p<nprofiles; p++) {
				PXCCapture::Device::StreamProfileSet profiles = {};
				if (device->QueryStreamProfileSet(st, p, &profiles) < PXC_STATUS_NO_ERROR) break;

				PXCCapture::Device::StreamProfile &profile = profiles[st];
				std::cout << profile.imageInfo.width << "x" << profile.imageInfo.height << " - " << profile.frameRate.max << " fps." << std::endl;
				if (profile.imageInfo.width == 1920 && profile.frameRate.max > 20) {
					chosenProfiles = profiles;
					device->SetStreamProfileSet(&chosenProfiles);
				}
			}
			mi++;
		}

		/* Optional: Set mirror mode */
		//device->SetMirrorMode(mirror ? PXCCapture::Device::MirrorMode::MIRROR_MODE_HORIZONTAL : PXCCapture::Device::MirrorMode::MIRROR_MODE_DISABLED);

		bool stop = false;
		//UpdateStatus(L"Streaming...");
		pxcStatus sts2 = PXC_STATUS_NO_ERROR;
		for (int nframes = 0; !stop; nframes++) {
			/* Wait until a frame is ready, synchronized or asynchronously */
			sts2 = sm->AcquireFrame(/*false, 1000 /*synced == SYNC_OPTION_SW*/);
			if (sts2 < PXC_STATUS_NO_ERROR) {
				std::cout << "Error " << sts2 << std::endl;
				break;
			}

			/* If device is not lost */
			if (sts2 >= PXC_STATUS_NO_ERROR) {
				/* Retrieve the sample */
				PXCCapture::Sample *sample = sm->QuerySample();
				PXCImage* rgbImage = sample->color;
				cv::Mat cvMat;
				ConvertPXCImageToOpenCVMat(rgbImage, &cvMat);
				cv::imshow("frame", cvMat);
				cv::waitKey(3);

				/* Resume next frame processing */
				sm->ReleaseFrame();
			}
		}
		break;
	}

	/* Clean Up */
	sm->Release();
}

int main(int _argc, char ** _argv) {
	choosingDevice();

	system("PAUSE");
}




void ConvertPXCImageToOpenCVMat(PXCImage *inImg, cv::Mat *outImg) {
	int cvDataType;
	int cvDataWidth;


	PXCImage::ImageData data;
	inImg->AcquireAccess(PXCImage::ACCESS_READ, &data);
	PXCImage::ImageInfo imgInfo = inImg->QueryInfo();

	switch (data.format) {
		/* STREAM_TYPE_COLOR */
	case PXCImage::PIXEL_FORMAT_YUY2: /* YUY2 image  */
	{
		PXCImage::ImageData data;
		inImg->AcquireAccess(PXCImage::ACCESS_READ, &data);
		PXCImage::ImageInfo imgInfo = inImg->QueryInfo();
		outImg->create(imgInfo.height, imgInfo.width, CV_8UC3);
		auto dataPointer = data.planes[0];
		for (unsigned i = 0; i < outImg->rows; i++) {
			for (unsigned j = 0; j < outImg->cols; j += 2) {
				//Get first data (2 pixels)
				uchar y0 = dataPointer[0];
				uchar u0 = dataPointer[1];
				uchar y1 = dataPointer[2];
				uchar v0 = dataPointer[3];
				dataPointer += 4;
				//// Transform to intermedial format
				// First pixel
				int c0 = y0 - 16;
				int d01 = u0 - 128;
				int e01 = v0 - 128;
				// second pixel
				int c1 = y1 - 16;

				//// assign data
				// First pixel
				uchar b0 = ((298 * c0 + 516 * d01 + 128) >> 8); // blue
				uchar g0 = ((298 * c0 - 100 * d01 - 208 * e01 + 128) >> 8); // green
				uchar r0 = ((298 * c0 + 409 * e01 + 128) >> 8); // red
				cv::Vec3b p1 = { b0, g0, r0 };
				outImg->at<cv::Vec3b>(i, j) = p1;

				// second pixel
				uchar b1 = ((298 * c1 + 516 * d01 + 128) >> 8); // blue
				uchar g1 = ((298 * c1 - 100 * d01 - 208 * e01 + 128) >> 8); // green
				uchar r1 = ((298 * c1 + 409 * e01 + 128) >> 8); // red
				cv::Vec3b p2 = { b1, g1, r1 };
				outImg->at<cv::Vec3b>(i, j + 1) = p2;

			}
		}
		return;
	}
	case PXCImage::PIXEL_FORMAT_NV12: /* NV12 image */
		throw(0); // Not implemented
	case PXCImage::PIXEL_FORMAT_RGB32: /* BGRA layout on a little-endian machine */
		cvDataType = CV_8UC4;
		cvDataWidth = 4;
		break;
	case PXCImage::PIXEL_FORMAT_RGB24: /* BGR layout on a little-endian machine */
		cvDataType = CV_8UC3;
		cvDataWidth = 3;
		break;
	case PXCImage::PIXEL_FORMAT_Y8:  /* 8-Bit Gray Image, or IR 8-bit */
		cvDataType = CV_8U;
		cvDataWidth = 1;
		break;

		/* STREAM_TYPE_DEPTH */
	case PXCImage::PIXEL_FORMAT_DEPTH: /* 16-bit unsigned integer with precision mm. */
	case PXCImage::PIXEL_FORMAT_DEPTH_RAW: /* 16-bit unsigned integer with device specific precision (call device->QueryDepthUnit()) */
		cvDataType = CV_16U;
		cvDataWidth = 2;
		break;
	case PXCImage::PIXEL_FORMAT_DEPTH_F32: /* 32-bit float-point with precision mm. */
		cvDataType = CV_32F;
		cvDataWidth = 4;
		break;

		/* STREAM_TYPE_IR */
	case PXCImage::PIXEL_FORMAT_Y16:          /* 16-Bit Gray Image */
		cvDataType = CV_16U;
		cvDataWidth = 2;
		break;
	case PXCImage::PIXEL_FORMAT_Y8_IR_RELATIVE:    /* Relative IR Image */
		cvDataType = CV_8U;
		cvDataWidth = 1;
		break;
	}

	// suppose that no other planes
	if (data.planes[1] != NULL) throw(0); // not implemented
										  // suppose that no sub pixel padding needed
	if (data.pitches[0] % cvDataWidth != 0) throw(0); // not implemented

	outImg->create(imgInfo.height, data.pitches[0] / cvDataWidth, cvDataType);

	memcpy(outImg->data, data.planes[0], imgInfo.height*imgInfo.width*cvDataWidth*sizeof(pxcBYTE));

	inImg->ReleaseAccess(&data);
}