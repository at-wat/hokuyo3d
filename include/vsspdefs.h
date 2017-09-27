/*
 * Copyright (c) 2014, ATR, Atsushi Watanabe
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VSSPDEFS_H
#define VSSPDEFS_H

#include <cstdint>
#include <cmath>

namespace vssp
{

static const uint32_t VSSP_MARK = ('V' << 0) | ('S' << 8) | ('S' << 16) | ('P' << 24);
static const uint32_t STATUS_OK = ('0' << 0) | ('0' << 8) | ('0' << 16) | (0xA << 24);
static const uint32_t STATUS_COMMAND_UNKNOWN = ('1' << 0) | ('0' << 8) | ('1' << 16) | (0xA << 24);
static const uint32_t STATUS_COMMAND_INVALID = ('1' << 0) | ('0' << 8) | ('2' << 16) | (0xA << 24);
static const uint32_t STATUS_COMMAND_INVALUD_METHOD = ('1' << 0) | ('0' << 8) | ('3' << 16) | (0xA << 24);
static const uint32_t STATUS_COMMAND_OUT_OF_RANGE = ('1' << 0) | ('0' << 8) | ('4' << 16) | (0xA << 24);
static const uint32_t STATUS_COMMUNICATION_FAILURE = ('2' << 0) | ('0' << 8) | ('1' << 16) | (0xA << 24);
static const uint32_t STATUS_UNKNOWN_ERROR = ('3' << 0) | ('0' << 8) | ('1' << 16) | (0xA << 24);

static const uint32_t TYPE_GET = ('G' << 0) | ('E' << 8) | ('T' << 16) | (':' << 24);
static const uint32_t TYPE_SET = ('S' << 0) | ('E' << 8) | ('T' << 16) | (':' << 24);
static const uint32_t TYPE_DAT = ('D' << 0) | ('A' << 8) | ('T' << 16) | (':' << 24);
static const uint32_t TYPE_VER = ('V' << 0) | ('E' << 8) | ('R' << 16) | (':' << 24);
static const uint32_t TYPE_PNG = ('P' << 0) | ('N' << 8) | ('G' << 16) | (':' << 24);
static const uint32_t TYPE_ERR = ('E' << 0) | ('R' << 8) | ('R' << 16) | (':' << 24);
static const uint32_t TYPE_RI = ('_' << 0) | ('r' << 8) | ('i' << 16) | (':' << 24);
static const uint32_t TYPE_RO = ('_' << 0) | ('r' << 8) | ('o' << 16) | (':' << 24);
static const uint32_t TYPE_AX = ('_' << 0) | ('a' << 8) | ('x' << 16) | (':' << 24);
static const uint32_t TYPE_ER = ('_' << 0) | ('e' << 8) | ('r' << 16) | (':' << 24);

enum AuxId
{
  AX_MASK_ANGVEL_X = 31,
  AX_MASK_ANGVEL_Y = 30,
  AX_MASK_ANGVEL_Z = 29,
  AX_MASK_LINACC_X = 28,
  AX_MASK_LINACC_Y = 27,
  AX_MASK_LINACC_Z = 26,
  AX_MASK_MAG_X = 25,
  AX_MASK_MAG_Y = 24,
  AX_MASK_MAG_Z = 23,
  AX_MASK_TEMP = 22,
  AX_MASK_FIRST = 22,
  AX_MASK_LAST = 31
};
static const uint32_t AX_MASK_ANGVEL = (1 << AX_MASK_ANGVEL_X) | (1 << AX_MASK_ANGVEL_Y) | (1 << AX_MASK_ANGVEL_Z);
static const uint32_t AX_MASK_LINACC = (1 << AX_MASK_LINACC_X) | (1 << AX_MASK_LINACC_Y) | (1 << AX_MASK_LINACC_Z);
static const uint32_t AX_MASK_MAG = (1 << AX_MASK_MAG_X) | (1 << AX_MASK_MAG_Y) | (1 << AX_MASK_MAG_Z);

#pragma pack(push, 1)
struct Header
{
  uint32_t mark;
  uint32_t type;
  uint32_t status;
  uint16_t header_length;
  uint16_t length;
  uint32_t received_time_ms;
  uint32_t send_time_ms;
};
struct RangeHeader
{
  uint16_t header_length;
  uint32_t line_head_timestamp_ms;
  uint32_t line_tail_timestamp_ms;
  int16_t line_head_h_angle_ratio;
  int16_t line_tail_h_angle_ratio;
  uint8_t frame;
  uint8_t field;
  uint16_t line;
  uint16_t spot;
};
struct RangeIndex
{
  uint16_t index_length;
  uint16_t nspots;
};
struct DataRangeIntensity
{
  uint16_t range_mm;
  uint16_t intensity;
};
struct DataRangeOnly
{
  uint16_t range_mm;
};
struct AuxHeader
{
  uint16_t header_length;
  uint32_t timestamp_ms;
  uint32_t data_bitfield;
  uint8_t data_count;
  uint8_t data_ms;
};
struct AuxData
{
  int32_t val;
};
#pragma pack(pop)

class AuxFactorArray
{
public:
  double operator[](AuxId id) const
  {
    return k_[static_cast<int>(id)];
  };
  double k_[AX_MASK_LAST + 1];
};
struct TableSincos
{
  double s;
  double c;
};
struct XYZ
{
  double x;
  double y;
  double z;
};
class XYZI
{
public:
  double x;
  double y;
  double z;
  double i;
  double r;

  XYZI()
  {
  }
  XYZI(const double &v_sin, const double &v_cos, const double &h_sin, const double &h_cos)
  {
    i = 0;
    r = 0;
    x = v_cos * h_cos;
    y = v_cos * h_sin;
    z = v_sin;
  }
  XYZI operator*(const DataRangeIntensity &data) const
  {
    XYZI ret = *this;
    double r = data.range_mm * 0.001;
    ret.i = data.intensity;
    ret.x *= r;
    ret.y *= r;
    ret.z *= r;
    ret.r = r;
    return ret;
  }
  XYZI operator*(const DataRangeOnly &data) const
  {
    XYZI ret = *this;
    double r = data.range_mm * 0.001;
    ret.i = 0;
    ret.x *= r;
    ret.y *= r;
    ret.z *= r;
    ret.r = r;
    return *this;
  }
};
class Aux
{
public:
  XYZ ang_vel;
  XYZ lin_acc;
  XYZ mag;
  double temp;

  Aux()
  {
    ang_vel.x = ang_vel.y = ang_vel.z = 0.0;
    lin_acc.x = lin_acc.y = lin_acc.z = 0.0;
    mag.x = mag.y = mag.z = 0.0;
    temp = 0.0;
  }
  double operator[](AuxId id) const
  {
    return operator[](id);
  }
  double &operator[](AuxId id)
  {
    switch (id)
    {
      case vssp::AX_MASK_ANGVEL_X:
        return ang_vel.x;
      case vssp::AX_MASK_ANGVEL_Y:
        return ang_vel.y;
      case vssp::AX_MASK_ANGVEL_Z:
        return ang_vel.z;
      case vssp::AX_MASK_LINACC_X:
        return lin_acc.x;
      case vssp::AX_MASK_LINACC_Y:
        return lin_acc.y;
      case vssp::AX_MASK_LINACC_Z:
        return lin_acc.z;
      case vssp::AX_MASK_MAG_X:
        return mag.x;
      case vssp::AX_MASK_MAG_Y:
        return mag.y;
      case vssp::AX_MASK_MAG_Z:
        return mag.z;
      case vssp::AX_MASK_TEMP:
        return temp;
    }
    throw "Invalid AUX data id";
  }
};

static const double G = 9.807;
static const double DEG = (M_PI / 180.0);

// Default value of Aux data scaling factor
// note: there is no way to get these values in communication
//      with prototype edition of the HOKUYO 3D sensor
static const AuxFactorArray AUX_FACTOR_DEFAULT =
    {
      {
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        1.0,                                 // Temperature [?]
        0.6e-6, 0.6e-6, 0.6e-6,              // Magnetometer [T]
        G / 8192.0, G / 8192.0, G / 8192.0,  // Accelerometer [m/ss]
        DEG / 65.5, DEG / 65.5, DEG / 65.5   // Gyrometer [rad/s]
      }
    };

}  // namespace vssp

#endif  // VSSPDEFS_H
