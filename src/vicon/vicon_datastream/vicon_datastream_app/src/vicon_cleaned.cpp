///////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) OMG Plc 2009.
// All rights reserved.  This software is protected by copyright
// law and international treaties.  No part of this software / document
// may be reproduced or distributed in any form or by any means,
// whether transiently or incidentally to some other use of this software,
// without the written permission of the copyright owner.
//
///////////////////////////////////////////////////////////////////////////////

#include "DataStreamClient.h"

#include <cassert>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#ifdef WIN32
#include <conio.h>   // For _kbhit()
#include <cstdio>    // For getchar()
#include <windows.h> // For Sleep()
#else
#include <unistd.h> // For sleep()
#endif              // WIN32

#include <string.h>
#include <time.h>

using namespace ViconDataStreamSDK::CPP;

namespace {
std::string Adapt(const bool i_Value) { return i_Value ? "True" : "False"; }

// Set time standard
std::string Adapt(const TimecodeStandard::Enum i_Standard) {
  switch (i_Standard) {
  default:
  case TimecodeStandard::None:
    return "0";
  case TimecodeStandard::PAL:
    return "1";
  case TimecodeStandard::NTSC:
    return "2";
  case TimecodeStandard::NTSCDrop:
    return "3";
  case TimecodeStandard::Film:
    return "4";
  case TimecodeStandard::NTSCFilm:
    return "5";
  case TimecodeStandard::ATSC:
    return "6";
  }
}

// Doubt - Set the direction for i axis
std::string Adapt(const Direction::Enum i_Direction) {
  switch (i_Direction) {
  case Direction::Forward:
    return "Forward";
  case Direction::Backward:
    return "Backward";
  case Direction::Left:
    return "Left";
  case Direction::Right:
    return "Right";
  case Direction::Up:
    return "Up";
  case Direction::Down:
    return "Down";
  default:
    return "Unknown";
  }
}

// Enable forceplate input device (not relevant)
std::string Adapt(const DeviceType::Enum i_DeviceType) {
  switch (i_DeviceType) {
  case DeviceType::ForcePlate:
    return "ForcePlate";
  case DeviceType::Unknown:
  default:
    return "Unknown";
  }
}

// Set the unit for state variable
std::string Adapt(const Unit::Enum i_Unit) {
  switch (i_Unit) {
  case Unit::Meter:
    return "Meter";
  case Unit::Volt:
    return "Volt";
  case Unit::NewtonMeter:
    return "NewtonMeter";
  case Unit::Newton:
    return "Newton";
  case Unit::Kilogram:
    return "Kilogram";
  case Unit::Second:
    return "Second";
  case Unit::Ampere:
    return "Ampere";
  case Unit::Kelvin:
    return "Kelvin";
  case Unit::Mole:
    return "Mole";
  case Unit::Candela:
    return "Candela";
  case Unit::Radian:
    return "Radian";
  case Unit::Steradian:
    return "Steradian";
  case Unit::MeterSquared:
    return "MeterSquared";
  case Unit::MeterCubed:
    return "MeterCubed";
  case Unit::MeterPerSecond:
    return "MeterPerSecond";
  case Unit::MeterPerSecondSquared:
    return "MeterPerSecondSquared";
  case Unit::RadianPerSecond:
    return "RadianPerSecond";
  case Unit::RadianPerSecondSquared:
    return "RadianPerSecondSquared";
  case Unit::Hertz:
    return "Hertz";
  case Unit::Joule:
    return "Joule";
  case Unit::Watt:
    return "Watt";
  case Unit::Pascal:
    return "Pascal";
  case Unit::Lumen:
    return "Lumen";
  case Unit::Lux:
    return "Lux";
  case Unit::Coulomb:
    return "Coulomb";
  case Unit::Ohm:
    return "Ohm";
  case Unit::Farad:
    return "Farad";
  case Unit::Weber:
    return "Weber";
  case Unit::Tesla:
    return "Tesla";
  case Unit::Henry:
    return "Henry";
  case Unit::Siemens:
    return "Siemens";
  case Unit::Becquerel:
    return "Becquerel";
  case Unit::Gray:
    return "Gray";
  case Unit::Sievert:
    return "Sievert";
  case Unit::Katal:
    return "Katal";

  case Unit::Unknown:
  default:
    return "Unknown";
  }
}
#ifdef WIN32
bool Hit() {
  bool hit = false;
  while (_kbhit()) {
    getchar();
    hit = true;
  }
  return hit;
}
#endif

class NullBuffer : public std::streambuf {
public:
  int overflow(int c) { return c; }
};

NullBuffer Null;
std::ostream NullStream(&Null);

} // namespace

int main(int argc, char *argv[]) {
  // Program options

  std::vector<std::string> Hosts;
  int Arg = 1;
  for (Arg; Arg < argc; ++Arg) {
    if (strncmp(argv[Arg], "--", 2) == 0) {
      break;
    }
    Hosts.push_back(argv[Arg]);
  }

  if (Hosts.empty()) {
    Hosts.push_back("localhost:801");
  }

  bool bOptimizeWireless = false;
  bool bQuiet = false;

  std::vector<std::string> HapticOnList(0);
  unsigned int ClientBufferSize = 0;
  std::string AxisMapping = "ZUp";
  std::vector<std::string> FilteredSubjects;
  std::vector<std::string> LocalAdapters;

  std::ostream &OutputStream(bQuiet ? NullStream : std::cout);

  bool First = true;
  std::string HostName;
  for (const auto &rHost : Hosts) {
    if (!First) {
      HostName += ";";
    }
    HostName += rHost;
    First = false;
  }

  // Make a new client
  ViconDataStreamSDK::CPP::Client DirectClient;

  if (bOptimizeWireless) {
    const Output_ConfigureWireless ConfigureWirelessResult =
        DirectClient.ConfigureWireless();

    if (ConfigureWirelessResult.Result != Result::Success) {
      std::cout << "Wireless Config: " << ConfigureWirelessResult.Error
                << std::endl;
    }
  }

  std::cout << "Connecting to " << HostName << " ..." << std::flush;
  while (!DirectClient.IsConnected().Connected) {

    // Direct connection
    const Output_Connect ConnectResult = DirectClient.Connect(HostName);
    const bool ok = (ConnectResult.Result == Result::Success);

    if (!ok) {
      std::cout << "Warning - connect failed... ";
      switch (ConnectResult.Result) {
      case Result::ClientAlreadyConnected:
        std::cout << "Client Already Connected" << std::endl;
        break;
      case Result::InvalidHostName:
        std::cout << "Invalid Host Name" << std::endl;
        break;
      case Result::ClientConnectionFailed:
        std::cout << "Client Connection Failed" << std::endl;
        break;
      default:
        std::cout << "Unrecognized Error: " << ConnectResult.Result
                  << std::endl;
        break;
      }
    }

    std::cout << ".";
#ifdef WIN32
    Sleep(1000);
#else
    sleep(1);
#endif
    // }

    std::cout << std::endl;

    std::cout << std::endl;
    // Enable some different data types
    DirectClient.EnableSegmentData();

    DirectClient.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);
    // }

    // Set the global up axis
    DirectClient.SetAxisMapping(Direction::Forward, Direction::Left,
                                Direction::Up); // Z-up

    if (AxisMapping == "YUp") {
      DirectClient.SetAxisMapping(Direction::Forward, Direction::Up,
                                  Direction::Right); // Y-up
    } else if (AxisMapping == "XUp") {
      DirectClient.SetAxisMapping(Direction::Up, Direction::Forward,
                                  Direction::Left); // Y-up
    }

    Output_GetAxisMapping _Output_GetAxisMapping =
        DirectClient.GetAxisMapping();
    std::cout << "Axis Mapping: X-" << Adapt(_Output_GetAxisMapping.XAxis)
              << " Y-" << Adapt(_Output_GetAxisMapping.YAxis) << " Z-"
              << Adapt(_Output_GetAxisMapping.ZAxis) << std::endl;

    // Discover the version number
    Output_GetVersion _Output_GetVersion = DirectClient.GetVersion();
    std::cout << "Version: " << _Output_GetVersion.Major << "."
              << _Output_GetVersion.Minor << "." << _Output_GetVersion.Point
              << "." << _Output_GetVersion.Revision << std::endl;

    if (ClientBufferSize > 0) {
      DirectClient.SetBufferSize(ClientBufferSize);
      std::cout << "Setting client buffer size to " << ClientBufferSize
                << std::endl;
    }

    bool bSubjectFilterApplied = false;

    ViconDataStreamSDK::CPP::Client &MyClient(DirectClient);

    size_t Counter = 0;
    const std::chrono::high_resolution_clock::time_point StartTime =
        std::chrono::high_resolution_clock::now();
    // Loop until a key is pressed
#ifdef WIN32
    while (!Hit())
#else
    while (true)
#endif
    {
      // Get a frame
      OutputStream << "Waiting for new frame...";
      while (MyClient.GetFrame().Result != Result::Success) {
// Sleep a little so that we don't lumber the CPU with a busy poll
#ifdef WIN32
        Sleep(200);
#else
        sleep(1);
#endif

        OutputStream << ".";
      }
      OutputStream << std::endl;

      // We have to call this after the call to get frame, otherwise we don't
      // have any subject info to map the name to ids
      if (!bSubjectFilterApplied) {
        for (const auto &rSubject : FilteredSubjects) {
          Output_AddToSubjectFilter SubjectFilterResult =
              MyClient.AddToSubjectFilter(rSubject);
          bSubjectFilterApplied = bSubjectFilterApplied ||
                                  SubjectFilterResult.Result == Result::Success;
        }
      }

      const std::chrono::high_resolution_clock::time_point Now =
          std::chrono::high_resolution_clock::now();

      // Get the frame number
      Output_GetFrameNumber _Output_GetFrameNumber = MyClient.GetFrameNumber();
      OutputStream << "Frame Number: " << _Output_GetFrameNumber.FrameNumber
                   << std::endl;

      Output_GetFrameRate Rate = MyClient.GetFrameRate();
      OutputStream << "Frame rate: " << Rate.FrameRateHz << std::endl;

      // Show frame rates
      for (unsigned int FramerateIndex = 0;
           FramerateIndex < MyClient.GetFrameRateCount().Count;
           ++FramerateIndex) {
        std::string FramerateName =
            MyClient.GetFrameRateName(FramerateIndex).Name;
        double FramerateValue = MyClient.GetFrameRateValue(FramerateName).Value;

        OutputStream << FramerateName << ": " << FramerateValue << "Hz"
                     << std::endl;
      }
      OutputStream << std::endl;

      // Get the timecode
      Output_GetTimecode _Output_GetTimecode = MyClient.GetTimecode();

      OutputStream << "Timecode: " << _Output_GetTimecode.Hours << "h "
                   << _Output_GetTimecode.Minutes << "m "
                   << _Output_GetTimecode.Seconds << "s "
                   << _Output_GetTimecode.Frames << "f "
                   << _Output_GetTimecode.SubFrame << "sf "
                   << Adapt(_Output_GetTimecode.FieldFlag) << " "
                   << Adapt(_Output_GetTimecode.Standard) << " "
                   << _Output_GetTimecode.SubFramesPerFrame << " "
                   << _Output_GetTimecode.UserBits << std::endl
                   << std::endl;

      // Get the latency
      OutputStream << "Latency: " << MyClient.GetLatencyTotal().Total << "s"
                   << std::endl;

      for (unsigned int LatencySampleIndex = 0;
           LatencySampleIndex < MyClient.GetLatencySampleCount().Count;
           ++LatencySampleIndex) {
        std::string SampleName =
            MyClient.GetLatencySampleName(LatencySampleIndex).Name;
        double SampleValue = MyClient.GetLatencySampleValue(SampleName).Value;

        OutputStream << "  " << SampleName << " " << SampleValue << "s"
                     << std::endl;
      }
      OutputStream << std::endl;

      Output_GetHardwareFrameNumber _Output_GetHardwareFrameNumber =
          MyClient.GetHardwareFrameNumber();
      OutputStream << "Hardware Frame Number: "
                   << _Output_GetHardwareFrameNumber.HardwareFrameNumber
                   << std::endl;

      // Count the number of subjects
      unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;
      OutputStream << "Subjects (" << SubjectCount << "):" << std::endl;
      for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount;
           ++SubjectIndex) {
        OutputStream << "  Subject #" << SubjectIndex << std::endl;

        // Get the subject name
        std::string SubjectName =
            MyClient.GetSubjectName(SubjectIndex).SubjectName;
        OutputStream << "    Name: " << SubjectName << std::endl;

        // Get the root segment
        std::string RootSegment =
            MyClient.GetSubjectRootSegmentName(SubjectName).SegmentName;
        OutputStream << "    Root Segment: " << RootSegment << std::endl;

        // Count the number of segments
        unsigned int SegmentCount =
            MyClient.GetSegmentCount(SubjectName).SegmentCount;
        OutputStream << "    Segments (" << SegmentCount << "):" << std::endl;
        for (unsigned int SegmentIndex = 0; SegmentIndex < SegmentCount;
             ++SegmentIndex) {
          OutputStream << "      Segment #" << SegmentIndex << std::endl;

          // Get the segment name
          std::string SegmentName =
              MyClient.GetSegmentName(SubjectName, SegmentIndex).SegmentName;
          OutputStream << "        Name: " << SegmentName << std::endl;

          // Get the segment parent
          std::string SegmentParentName =
              MyClient.GetSegmentParentName(SubjectName, SegmentName)
                  .SegmentName;
          OutputStream << "        Parent: " << SegmentParentName << std::endl;

          // Get the segment's children
          unsigned int ChildCount =
              MyClient.GetSegmentChildCount(SubjectName, SegmentName)
                  .SegmentCount;
          OutputStream << "     Children (" << ChildCount << "):" << std::endl;
          for (unsigned int ChildIndex = 0; ChildIndex < ChildCount;
               ++ChildIndex) {
            std::string ChildName =
                MyClient
                    .GetSegmentChildName(SubjectName, SegmentName, ChildIndex)
                    .SegmentName;
            OutputStream << "       " << ChildName << std::endl;
          }

          // Get the static segment scale
          Output_GetSegmentStaticScale _Output_GetSegmentStaticScale =
              MyClient.GetSegmentStaticScale(SubjectName, SegmentName);
          if (_Output_GetSegmentStaticScale.Result == Result::Success) {
            OutputStream << "        Static Scale: ("
                         << _Output_GetSegmentStaticScale.Scale[0] << ", "
                         << _Output_GetSegmentStaticScale.Scale[1] << ", "
                         << _Output_GetSegmentStaticScale.Scale[2] << ")"
                         << std::endl;
          }

          // Get the static segment translation
          Output_GetSegmentStaticTranslation
              _Output_GetSegmentStaticTranslation =
                  MyClient.GetSegmentStaticTranslation(SubjectName,
                                                       SegmentName);
          OutputStream << "        Static Translation: ("
                       << _Output_GetSegmentStaticTranslation.Translation[0]
                       << ", "
                       << _Output_GetSegmentStaticTranslation.Translation[1]
                       << ", "
                       << _Output_GetSegmentStaticTranslation.Translation[2]
                       << ")" << std::endl;

          // Get the static segment rotation in helical co-ordinates
          Output_GetSegmentStaticRotationHelical
              _Output_GetSegmentStaticRotationHelical =
                  MyClient.GetSegmentStaticRotationHelical(SubjectName,
                                                           SegmentName);
          OutputStream << "        Static Rotation Helical: ("
                       << _Output_GetSegmentStaticRotationHelical.Rotation[0]
                       << ", "
                       << _Output_GetSegmentStaticRotationHelical.Rotation[1]
                       << ", "
                       << _Output_GetSegmentStaticRotationHelical.Rotation[2]
                       << ")" << std::endl;

          // Get the static segment rotation as a matrix
          Output_GetSegmentStaticRotationMatrix
              _Output_GetSegmentStaticRotationMatrix =
                  MyClient.GetSegmentStaticRotationMatrix(SubjectName,
                                                          SegmentName);
          OutputStream
              << "        Static Rotation Matrix: ("
              << _Output_GetSegmentStaticRotationMatrix.Rotation[0] << ", "
              << _Output_GetSegmentStaticRotationMatrix.Rotation[1] << ", "
              << _Output_GetSegmentStaticRotationMatrix.Rotation[2] << ", "
              << _Output_GetSegmentStaticRotationMatrix.Rotation[3] << ", "
              << _Output_GetSegmentStaticRotationMatrix.Rotation[4] << ", "
              << _Output_GetSegmentStaticRotationMatrix.Rotation[5] << ", "
              << _Output_GetSegmentStaticRotationMatrix.Rotation[6] << ", "
              << _Output_GetSegmentStaticRotationMatrix.Rotation[7] << ", "
              << _Output_GetSegmentStaticRotationMatrix.Rotation[8] << ")"
              << std::endl;

          // Get the static segment rotation in quaternion co-ordinates
          Output_GetSegmentStaticRotationQuaternion
              _Output_GetSegmentStaticRotationQuaternion =
                  MyClient.GetSegmentStaticRotationQuaternion(SubjectName,
                                                              SegmentName);
          OutputStream << "        Static Rotation Quaternion: ("
                       << _Output_GetSegmentStaticRotationQuaternion.Rotation[0]
                       << ", "
                       << _Output_GetSegmentStaticRotationQuaternion.Rotation[1]
                       << ", "
                       << _Output_GetSegmentStaticRotationQuaternion.Rotation[2]
                       << ", "
                       << _Output_GetSegmentStaticRotationQuaternion.Rotation[3]
                       << ")" << std::endl;

          // Get the static segment rotation in EulerXYZ co-ordinates
          Output_GetSegmentStaticRotationEulerXYZ
              _Output_GetSegmentStaticRotationEulerXYZ =
                  MyClient.GetSegmentStaticRotationEulerXYZ(SubjectName,
                                                            SegmentName);
          OutputStream << "        Static Rotation EulerXYZ: ("
                       << _Output_GetSegmentStaticRotationEulerXYZ.Rotation[0]
                       << ", "
                       << _Output_GetSegmentStaticRotationEulerXYZ.Rotation[1]
                       << ", "
                       << _Output_GetSegmentStaticRotationEulerXYZ.Rotation[2]
                       << ")" << std::endl;

          // Get the global segment translation
          Output_GetSegmentGlobalTranslation
              _Output_GetSegmentGlobalTranslation =
                  MyClient.GetSegmentGlobalTranslation(SubjectName,
                                                       SegmentName);
          OutputStream << "        Global Translation: ("
                       << _Output_GetSegmentGlobalTranslation.Translation[0]
                       << ", "
                       << _Output_GetSegmentGlobalTranslation.Translation[1]
                       << ", "
                       << _Output_GetSegmentGlobalTranslation.Translation[2]
                       << ") "
                       << Adapt(_Output_GetSegmentGlobalTranslation.Occluded)
                       << std::endl;

          // Get the global segment rotation in helical co-ordinates
          Output_GetSegmentGlobalRotationHelical
              _Output_GetSegmentGlobalRotationHelical =
                  MyClient.GetSegmentGlobalRotationHelical(SubjectName,
                                                           SegmentName);
          OutputStream << "        Global Rotation Helical: ("
                       << _Output_GetSegmentGlobalRotationHelical.Rotation[0]
                       << ", "
                       << _Output_GetSegmentGlobalRotationHelical.Rotation[1]
                       << ", "
                       << _Output_GetSegmentGlobalRotationHelical.Rotation[2]
                       << ") "
                       << Adapt(
                              _Output_GetSegmentGlobalRotationHelical.Occluded)
                       << std::endl;

          // Get the global segment rotation as a matrix
          Output_GetSegmentGlobalRotationMatrix
              _Output_GetSegmentGlobalRotationMatrix =
                  MyClient.GetSegmentGlobalRotationMatrix(SubjectName,
                                                          SegmentName);
          OutputStream
              << "        Global Rotation Matrix: ("
              << _Output_GetSegmentGlobalRotationMatrix.Rotation[0] << ", "
              << _Output_GetSegmentGlobalRotationMatrix.Rotation[1] << ", "
              << _Output_GetSegmentGlobalRotationMatrix.Rotation[2] << ", "
              << _Output_GetSegmentGlobalRotationMatrix.Rotation[3] << ", "
              << _Output_GetSegmentGlobalRotationMatrix.Rotation[4] << ", "
              << _Output_GetSegmentGlobalRotationMatrix.Rotation[5] << ", "
              << _Output_GetSegmentGlobalRotationMatrix.Rotation[6] << ", "
              << _Output_GetSegmentGlobalRotationMatrix.Rotation[7] << ", "
              << _Output_GetSegmentGlobalRotationMatrix.Rotation[8] << ") "
              << Adapt(_Output_GetSegmentGlobalRotationMatrix.Occluded)
              << std::endl;

          // Get the global segment rotation in quaternion co-ordinates
          Output_GetSegmentGlobalRotationQuaternion
              _Output_GetSegmentGlobalRotationQuaternion =
                  MyClient.GetSegmentGlobalRotationQuaternion(SubjectName,
                                                              SegmentName);
          OutputStream
              << "        Global Rotation Quaternion: ("
              << _Output_GetSegmentGlobalRotationQuaternion.Rotation[0] << ", "
              << _Output_GetSegmentGlobalRotationQuaternion.Rotation[1] << ", "
              << _Output_GetSegmentGlobalRotationQuaternion.Rotation[2] << ", "
              << _Output_GetSegmentGlobalRotationQuaternion.Rotation[3] << ") "
              << Adapt(_Output_GetSegmentGlobalRotationQuaternion.Occluded)
              << std::endl;

          // Get the global segment rotation in EulerXYZ co-ordinates
          Output_GetSegmentGlobalRotationEulerXYZ
              _Output_GetSegmentGlobalRotationEulerXYZ =
                  MyClient.GetSegmentGlobalRotationEulerXYZ(SubjectName,
                                                            SegmentName);
          OutputStream << "        Global Rotation EulerXYZ: ("
                       << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[0]
                       << ", "
                       << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[1]
                       << ", "
                       << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[2]
                       << ") "
                       << Adapt(
                              _Output_GetSegmentGlobalRotationEulerXYZ.Occluded)
                       << std::endl;

          // Get the local segment translation
          Output_GetSegmentLocalTranslation _Output_GetSegmentLocalTranslation =
              MyClient.GetSegmentLocalTranslation(SubjectName, SegmentName);
          OutputStream << "        Local Translation: ("
                       << _Output_GetSegmentLocalTranslation.Translation[0]
                       << ", "
                       << _Output_GetSegmentLocalTranslation.Translation[1]
                       << ", "
                       << _Output_GetSegmentLocalTranslation.Translation[2]
                       << ") "
                       << Adapt(_Output_GetSegmentLocalTranslation.Occluded)
                       << std::endl;

          // Get the local segment rotation in helical co-ordinates
          Output_GetSegmentLocalRotationHelical
              _Output_GetSegmentLocalRotationHelical =
                  MyClient.GetSegmentLocalRotationHelical(SubjectName,
                                                          SegmentName);
          OutputStream << "        Local Rotation Helical: ("
                       << _Output_GetSegmentLocalRotationHelical.Rotation[0]
                       << ", "
                       << _Output_GetSegmentLocalRotationHelical.Rotation[1]
                       << ", "
                       << _Output_GetSegmentLocalRotationHelical.Rotation[2]
                       << ") "
                       << Adapt(_Output_GetSegmentLocalRotationHelical.Occluded)
                       << std::endl;

          // Get the local segment rotation as a matrix
          Output_GetSegmentLocalRotationMatrix
              _Output_GetSegmentLocalRotationMatrix =
                  MyClient.GetSegmentLocalRotationMatrix(SubjectName,
                                                         SegmentName);
          OutputStream
              << "        Local Rotation Matrix: ("
              << _Output_GetSegmentLocalRotationMatrix.Rotation[0] << ", "
              << _Output_GetSegmentLocalRotationMatrix.Rotation[1] << ", "
              << _Output_GetSegmentLocalRotationMatrix.Rotation[2] << ", "
              << _Output_GetSegmentLocalRotationMatrix.Rotation[3] << ", "
              << _Output_GetSegmentLocalRotationMatrix.Rotation[4] << ", "
              << _Output_GetSegmentLocalRotationMatrix.Rotation[5] << ", "
              << _Output_GetSegmentLocalRotationMatrix.Rotation[6] << ", "
              << _Output_GetSegmentLocalRotationMatrix.Rotation[7] << ", "
              << _Output_GetSegmentLocalRotationMatrix.Rotation[8] << ") "
              << Adapt(_Output_GetSegmentLocalRotationMatrix.Occluded)
              << std::endl;

          // Get the local segment rotation in quaternion co-ordinates
          Output_GetSegmentLocalRotationQuaternion
              _Output_GetSegmentLocalRotationQuaternion =
                  MyClient.GetSegmentLocalRotationQuaternion(SubjectName,
                                                             SegmentName);
          OutputStream
              << "        Local Rotation Quaternion: ("
              << _Output_GetSegmentLocalRotationQuaternion.Rotation[0] << ", "
              << _Output_GetSegmentLocalRotationQuaternion.Rotation[1] << ", "
              << _Output_GetSegmentLocalRotationQuaternion.Rotation[2] << ", "
              << _Output_GetSegmentLocalRotationQuaternion.Rotation[3] << ") "
              << Adapt(_Output_GetSegmentLocalRotationQuaternion.Occluded)
              << std::endl;

          // Get the local segment rotation in EulerXYZ co-ordinates
          Output_GetSegmentLocalRotationEulerXYZ
              _Output_GetSegmentLocalRotationEulerXYZ =
                  MyClient.GetSegmentLocalRotationEulerXYZ(SubjectName,
                                                           SegmentName);
          OutputStream << "        Local Rotation EulerXYZ: ("
                       << _Output_GetSegmentLocalRotationEulerXYZ.Rotation[0]
                       << ", "
                       << _Output_GetSegmentLocalRotationEulerXYZ.Rotation[1]
                       << ", "
                       << _Output_GetSegmentLocalRotationEulerXYZ.Rotation[2]
                       << ") "
                       << Adapt(
                              _Output_GetSegmentLocalRotationEulerXYZ.Occluded)
                       << std::endl;
        }

        // Get the quality of the subject (object) if supported
        Output_GetObjectQuality _Output_GetObjectQuality =
            MyClient.GetObjectQuality(SubjectName);
        if (_Output_GetObjectQuality.Result == Result::Success) {
          double Quality = _Output_GetObjectQuality.Quality;
          OutputStream << "    Quality: " << Quality << std::endl;
        }

        // Count the number of markers
        unsigned int MarkerCount =
            MyClient.GetMarkerCount(SubjectName).MarkerCount;
        OutputStream << "    Markers (" << MarkerCount << "):" << std::endl;
        for (unsigned int MarkerIndex = 0; MarkerIndex < MarkerCount;
             ++MarkerIndex) {
          // Get the marker name
          std::string MarkerName =
              MyClient.GetMarkerName(SubjectName, MarkerIndex).MarkerName;

          // Get the marker parent
          std::string MarkerParentName =
              MyClient.GetMarkerParentName(SubjectName, MarkerName).SegmentName;

          // Get the global marker translation
          Output_GetMarkerGlobalTranslation _Output_GetMarkerGlobalTranslation =
              MyClient.GetMarkerGlobalTranslation(SubjectName, MarkerName);

          OutputStream << "      Marker #" << MarkerIndex << ": " << MarkerName
                       << " ("
                       << _Output_GetMarkerGlobalTranslation.Translation[0]
                       << ", "
                       << _Output_GetMarkerGlobalTranslation.Translation[1]
                       << ", "
                       << _Output_GetMarkerGlobalTranslation.Translation[2]
                       << ") "
                       << Adapt(_Output_GetMarkerGlobalTranslation.Occluded)
                       << std::endl;
        }
      }
    }

    // Get the unlabeled markers
    unsigned int UnlabeledMarkerCount =
        MyClient.GetUnlabeledMarkerCount().MarkerCount;
    OutputStream << "  Unlabeled Markers (" << UnlabeledMarkerCount
                 << "):" << std::endl;
    for (unsigned int UnlabeledMarkerIndex = 0;
         UnlabeledMarkerIndex < UnlabeledMarkerCount; ++UnlabeledMarkerIndex) {
      // Get the global marker translation
      Output_GetUnlabeledMarkerGlobalTranslation
          _Output_GetUnlabeledMarkerGlobalTranslation =
              MyClient.GetUnlabeledMarkerGlobalTranslation(
                  UnlabeledMarkerIndex);

      std::ostringstream MarkerTrajIDs;
      OutputStream << "    Marker #" << UnlabeledMarkerIndex << ": ("
                   << _Output_GetUnlabeledMarkerGlobalTranslation.Translation[0]
                   << ", "
                   << _Output_GetUnlabeledMarkerGlobalTranslation.Translation[1]
                   << ", "
                   << _Output_GetUnlabeledMarkerGlobalTranslation.Translation[2]
                   << ")" << MarkerTrajIDs.str() << std::endl;
    }

    // Get the labeled markers
    unsigned int LabeledMarkerCount =
        MyClient.GetLabeledMarkerCount().MarkerCount;
    OutputStream << "  Labeled Markers (" << LabeledMarkerCount
                 << "):" << std::endl;
    for (unsigned int LabeledMarkerIndex = 0;
         LabeledMarkerIndex < LabeledMarkerCount; ++LabeledMarkerIndex) {
      // Get the global marker translation
      Output_GetLabeledMarkerGlobalTranslation
          _Output_GetLabeledMarkerGlobalTranslation =
              MyClient.GetLabeledMarkerGlobalTranslation(LabeledMarkerIndex);

      std::ostringstream MarkerTrajIDs;
      OutputStream << "    Marker #" << LabeledMarkerIndex << ": ("
                   << _Output_GetLabeledMarkerGlobalTranslation.Translation[0]
                   << ", "
                   << _Output_GetLabeledMarkerGlobalTranslation.Translation[1]
                   << ", "
                   << _Output_GetLabeledMarkerGlobalTranslation.Translation[2]
                   << ")" << MarkerTrajIDs.str() << std::endl;
    }

    // Count the number of devices
    unsigned int DeviceCount = MyClient.GetDeviceCount().DeviceCount;
    OutputStream << "  Devices (" << DeviceCount << "):" << std::endl;
    for (unsigned int DeviceIndex = 0; DeviceIndex < DeviceCount;
         ++DeviceIndex) {
      OutputStream << "    Device #" << DeviceIndex << ":" << std::endl;

      // Get the device name and type
      Output_GetDeviceName _Output_GetDeviceName =
          MyClient.GetDeviceName(DeviceIndex);
      OutputStream << "      Name: " << _Output_GetDeviceName.DeviceName
                   << std::endl;
      OutputStream << "      Type: " << Adapt(_Output_GetDeviceName.DeviceType)
                   << std::endl;

      // Count the number of device outputs
      unsigned int DeviceOutputCount =
          MyClient.GetDeviceOutputCount(_Output_GetDeviceName.DeviceName)
              .DeviceOutputCount;
      OutputStream << "      Device Outputs (" << DeviceOutputCount
                   << "):" << std::endl;
      for (unsigned int DeviceOutputIndex = 0;
           DeviceOutputIndex < DeviceOutputCount; ++DeviceOutputIndex) {
        // Get the device output name and unit
        Output_GetDeviceOutputComponentName
            _Output_GetDeviceOutputComponentName =
                MyClient.GetDeviceOutputComponentName(
                    _Output_GetDeviceName.DeviceName, DeviceOutputIndex);

        unsigned int DeviceOutputSubsamples =
            MyClient
                .GetDeviceOutputSubsamples(
                    _Output_GetDeviceName.DeviceName,
                    _Output_GetDeviceOutputComponentName.DeviceOutputName,
                    _Output_GetDeviceOutputComponentName
                        .DeviceOutputComponentName)
                .DeviceOutputSubsamples;

        OutputStream << "      Device Output #" << DeviceOutputIndex << ":"
                     << std::endl;
        OutputStream << "      Samples (" << DeviceOutputSubsamples
                     << "):" << std::endl;

        for (unsigned int DeviceOutputSubsample = 0;
             DeviceOutputSubsample < DeviceOutputSubsamples;
             ++DeviceOutputSubsample) {
          OutputStream << "        Sample #" << DeviceOutputSubsample << ":"
                       << std::endl;

          // Get the device output value
          Output_GetDeviceOutputValue _Output_GetDeviceOutputValue =
              MyClient.GetDeviceOutputValue(
                  _Output_GetDeviceName.DeviceName,
                  _Output_GetDeviceOutputComponentName.DeviceOutputName,
                  _Output_GetDeviceOutputComponentName
                      .DeviceOutputComponentName,
                  DeviceOutputSubsample);

          OutputStream
              << "          '"
              << _Output_GetDeviceOutputComponentName.DeviceOutputComponentName
              << "' " << _Output_GetDeviceOutputValue.Value << " "
              << Adapt(_Output_GetDeviceOutputComponentName.DeviceOutputUnit)
              << " " << Adapt(_Output_GetDeviceOutputValue.Occluded)
              << std::endl;
        }
      }
    }
    ++Counter;
  }
}
