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

#include <iostream>
#include <algorithm>
#include <string>

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

				/* Display Main & PIP pictures */
				/*bool pip_update = false;
				if (pip) {
					PXCImage *image = (*sample)[pip];
					if (image) {
						DisplayPIPImage(image);
						pip_update = true;
					}
				}

				PXCImage::ImageInfo info = {};
				if (main) {
					PXCImage* image = (*sample)[main];
					if (image) {
						DisplayMainImage(image);
						pip_update = false;  // pip is updated here as well.
						info = image->QueryInfo();
					}
				}

				if (pip_update) DisplayMainImage(0);
				if (info.width > 0) Tick(&info);*/
				/* Resume next frame processing */
				sm->ReleaseFrame();
			}
		}
		break;
	}

	/* Clean Up */
	sm->Release();
}

void rawStreamDeviceSenseManager() {
	// Create a SenseManager instance
	PXCSenseManager *sm = PXCSenseManager::CreateInstance();

	// Enable the video stream
	sm->EnableStream(PXCCapture::STREAM_TYPE_COLOR, 0, 0, 0);

	// Initialization
	sm->Init();

	// Stream data
	while (sm->AcquireFrame(true) >= PXC_STATUS_NO_ERROR) {

		// Get the sample data
		PXCCapture::Sample *sample = sm->QuerySample();

		 // Resume next frame processing
		sm->ReleaseFrame();
	}

	// Clean up
	sm->Release();
}

int main(int _argc, char ** _argv) {
	choosingDevice();

	system("PAUSE");
}
