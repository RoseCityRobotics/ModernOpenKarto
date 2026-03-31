// Minimal RPLidar + OpenKarto SLAM demo.
// Reads 360-degree scans from an RPLidar, feeds them into the OpenKarto
// mapper, and saves an occupancy grid as a PGM file on Ctrl-C.

#include <OpenKarto.h>
#include <sl_lidar.h>

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

static volatile sig_atomic_t g_running = 1;

static void signalHandler(int) { g_running = 0; }

// Save an OccupancyGrid as a PGM (Portable GrayMap) image file.
static bool savePgm(const karto::OccupancyGrid* grid, const char* filename) {
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open " << filename << " for writing\n";
        return false;
    }

    const int32_t width = grid->GetWidth();
    const int32_t height = grid->GetHeight();
    const int32_t widthStep = grid->GetWidthStep();
    const uint8_t* data = grid->GetDataPointer();

    file << "P5\n" << width << " " << height << "\n255\n";

    for (int32_t y = height - 1; y >= 0; --y) {
        const uint8_t* row = data + y * widthStep;
        for (int32_t x = 0; x < width; ++x) {
            uint8_t cell = row[x];
            uint8_t pixel;
            if (cell == karto::GridStates_Free)
                pixel = 255;  // white
            else if (cell == karto::GridStates_Occupied)
                pixel = 0;    // black
            else
                pixel = 128;  // gray (unknown)
            file.put(static_cast<char>(pixel));
        }
    }

    std::cout << "Saved " << width << "x" << height << " map to " << filename << "\n";
    return true;
}

int main(int argc, char* argv[]) {
    const char* port = "/dev/ttyUSB0";
    if (argc > 1) port = argv[1];

    std::signal(SIGINT, signalHandler);

    // --- RPLidar setup ---
    sl::ILidarDriver* drv = *sl::createLidarDriver();
    if (!drv) {
        std::cerr << "Failed to create RPLidar driver\n";
        return 1;
    }

    sl::IChannel* channel = *sl::createSerialPortChannel(port, 115200);
    if (SL_IS_FAIL(drv->connect(channel))) {
        std::cerr << "Failed to connect to RPLidar on " << port << "\n";
        delete drv;
        return 1;
    }

    sl_lidar_response_device_info_t devInfo;
    if (SL_IS_OK(drv->getDeviceInfo(devInfo))) {
        std::cout << "RPLidar model: " << static_cast<int>(devInfo.model)
                  << "  firmware: " << (devInfo.firmware_version >> 8)
                  << "." << (devInfo.firmware_version & 0xFF) << "\n";
    }

    sl_lidar_response_device_health_t healthInfo;
    if (SL_IS_OK(drv->getHealth(healthInfo))) {
        if (healthInfo.status == SL_LIDAR_STATUS_ERROR) {
            std::cerr << "RPLidar reports error status — resetting\n";
            drv->reset();
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }

    drv->setMotorSpeed();
    drv->startScan(false, true);

    // --- Karto laser sensor (360 degrees, 1-degree resolution) ---
    constexpr double kAngularRes = 1.0;  // degrees
    constexpr int kNumReadings = 361;    // -180 to +180 inclusive

    karto::LaserRangeFinder* laser =
        karto::LaserRangeFinder::CreateLaserRangeFinder(
            karto::LaserRangeFinder_Custom, "rplidar");
    laser->SetMinimumRange(0.15);
    laser->SetMaximumRange(12.0);
    laser->SetMinimumAngle(-M_PI);
    laser->SetMaximumAngle(M_PI);
    laser->SetAngularResolution(karto::DegreesToRadians(kAngularRes));
    laser->SetRangeThreshold(12.0);

    // --- Karto mapper ---
    karto::OpenMapper mapper("rplidar_mapper");
    // Accept every scan even without movement (stationary lidar is fine)
    mapper.GetConfig().minimumTravelDistance = 0.0;
    mapper.GetConfig().minimumTravelHeading = 0.0;

    // --- Main loop ---
    int scanCount = 0;

    std::cout << "Scanning... press Ctrl-C to stop and save map.\n";

    while (g_running) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = sizeof(nodes) / sizeof(nodes[0]);

        if (SL_IS_FAIL(drv->grabScanDataHq(nodes, count, /*timeout_ms=*/2000))) {
            std::cerr << "Failed to grab scan data\n";
            continue;
        }
        drv->ascendScanData(nodes, count);

        // Convert RPLidar data to Karto range readings.
        // RPLidar: angles 0-360 degrees, distance in mm * 4 (Q2 fixed point).
        // Karto: readings from -PI to +PI at 1-degree steps, distance in meters.
        karto::RangeReadingsList readings(kNumReadings, laser->GetMaximumRange());

        for (size_t i = 0; i < count; ++i) {
            if (nodes[i].quality == 0) continue;

            double angleDeg = nodes[i].angle_z_q14 * 90.0 / 16384.0;  // 0-360
            if (angleDeg > 180.0) angleDeg -= 360.0;                   // -180 to +180

            double distM = (nodes[i].dist_mm_q2 / 4.0) / 1000.0;
            if (distM < laser->GetMinimumRange() || distM > laser->GetMaximumRange())
                continue;

            int idx = static_cast<int>(std::round(angleDeg)) + 180;  // 0-360 index
            if (idx < 0 || idx >= kNumReadings) continue;

            // Keep closest reading per bin
            if (distM < readings[idx])
                readings[idx] = distM;
        }

        // Create scan and feed to mapper
        const auto& constReadings = readings;
        auto* scan = new karto::LocalizedRangeScan(laser->GetName(), constReadings);
        scan->SetLaserRangeFinder(laser);
        scan->SetOdometricPose(karto::Pose2(0.0, 0.0, 0.0));
        scan->SetCorrectedPose(karto::Pose2(0.0, 0.0, 0.0));

        if (mapper.Process(scan)) {
            karto::Pose2 corrected = scan->GetCorrectedPose();
            ++scanCount;
            std::cout << "Scan " << scanCount << " accepted. Corrected pose: ("
                      << corrected.GetX() << ", " << corrected.GetY()
                      << ", " << karto::RadiansToDegrees(corrected.GetHeading())
                      << " deg)\n";
        } else {
            delete scan;
        }
    }

    // --- Shutdown ---
    std::cout << "\nStopping lidar...\n";
    drv->stop();
    drv->setMotorSpeed(0);
    delete drv;

    if (scanCount > 0) {
        std::cout << "Generating occupancy grid from " << scanCount << " scans...\n";
        karto::OccupancyGrid* grid =
            karto::OccupancyGrid::CreateFromMapper(&mapper, 0.05);
        if (grid) {
            savePgm(grid, "slam_output.pgm");
            delete grid;
        }
    } else {
        std::cout << "No scans were processed — no map to save.\n";
    }

    delete laser;
    return 0;
}
