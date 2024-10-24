#include <iostream>
#include <stdio.h>
#include <unordered_map>
#include <openvr.h>
#include <chrono>
#include <thread>

int main()
{
	vr::EVRInitError error;
	vr::IVRSystem* sys = vr::VR_Init(&error, vr::VRApplication_Overlay);
	if (error != 0) {
		printf("Error initializng: %s\n", VR_GetVRInitErrorAsSymbol(error));
	}

	vr::VROverlayHandle_t handle;
	vr::VROverlay()->CreateOverlay("soph_wireless_transmitter", "Soph Wireless Transmitter", &handle);

	std::cout << "Hello World!\n";

	while (true) {
		vr::TrackedDevicePose_t tracked_poses[vr::k_unMaxTrackedDeviceCount] = {};
		int tracked_poses_length = sizeof(tracked_poses) / sizeof(tracked_poses[0]);
		vr::EVRCompositorError e = vr::VRCompositor()->WaitGetPoses(tracked_poses, tracked_poses_length, nullptr, 0);
		if (e != vr::EVRCompositorError::VRCompositorError_None) {
			printf("Got error %d while trying to get poses\n", e);
			std::this_thread::sleep_for(std::chrono::seconds(10));
			continue;
		}

		// Skip 1 due to HMD being 0 always
		for (int i = 1; i < tracked_poses_length; i++)
		{
			// If it's not currently connected we don't care
			if (vr::VRSystem()->IsTrackedDeviceConnected(i)) {
				// Only trackers
				uint32_t device_class = vr::VRSystem()->GetInt32TrackedDeviceProperty(i, vr::Prop_DeviceClass_Int32);
				if (device_class == vr::TrackedDeviceClass_GenericTracker || true) { // For now just allow all devices

					char serial[32];
					int res = sys->GetStringTrackedDeviceProperty(i, vr::ETrackedDeviceProperty::Prop_SerialNumber_String, serial, sizeof(serial));

					if (res == 0) {
						printf("Failed to get serial for tracker at %d", i);
						continue;
					}


					// Print the serial number and it's matrix
					printf("Matrix for device with serial '%s':\n", serial);
					for (int j = 0; j < 3; j++) {
						for (int k = 0; k < 4; k++) {
							std::cout << tracked_poses[i].mDeviceToAbsoluteTracking.m[j][k] << "\t";
						}
						std::cout << std::endl;
					}
				}
			}
		}

		std::this_thread::sleep_for(std::chrono::seconds(10));
	}

}
