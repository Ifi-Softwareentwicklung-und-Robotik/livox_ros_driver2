//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "driver_node.h"
#include "lddc.h"
#include "livox_lidar_api.h"
#include "comm/comm.h"
#include <thread>
#include <chrono>
#include <string>

namespace livox_ros {
DriverNode& DriverNode::GetNode() noexcept {
  return *this;
}

// Callback for sleep mode command
static void SleepModeCallback(livox_status status, uint32_t handle, 
                              LivoxLidarAsyncControlResponse *response, void *client_data) {
  std::string ip_str = IpNumToString(handle);
  if (status == kLivoxLidarStatusSuccess) {
    printf("=== SUCCESS: LiDAR %s entered sleep mode ===\n", ip_str.c_str());
  } else {
    printf("=== FAILED: LiDAR %s sleep command failed with status %d ===\n", 
           ip_str.c_str(), status);
  }
}

DriverNode::~DriverNode() {
  printf("=== LIVOX DRIVER SHUTTING DOWN ===\n");
  printf("=== Setting LiDAR to sleep mode ===\n");
  
  int connected_count = 0;
  for (uint8_t i = 0; i < kMaxSourceLidar; i++) {
    LidarDevice* lidar = &(lddc_ptr_->lds_->lidars_[i]);
    LidarConnectState state = lidar->connect_state;
    
    // Only process lidars that are actually connected
    if (state == kConnectStateSampling || state == kConnectStateOn || state == kConnectStateConfig) {
      uint32_t handle = lidar->handle;
      std::string ip_str = IpNumToString(handle);
      
      printf("=== LiDAR %d: IP=%s, Handle=%u, ConnectState=%d ===\n", 
             i, ip_str.c_str(), handle, static_cast<int>(state));
      printf("=== Sending sleep command to LiDAR %s ===\n", ip_str.c_str());
      
      SetLivoxLidarWorkMode(handle, kLivoxLidarWakeUp, SleepModeCallback, nullptr);
      connected_count++;
    }
  }
  
  printf("=== Found %d connected LiDAR(s) ===\n", connected_count);
  
  // Wait longer to ensure sleep commands are processed
  printf("=== Waiting for sleep commands to complete ===\n");
  std::this_thread::sleep_for(std::chrono::seconds(2));
  
  lddc_ptr_->lds_->RequestExit();
  exit_signal_.set_value();
  pointclouddata_poll_thread_->join();
  imudata_poll_thread_->join();
}

} // namespace livox_ros





