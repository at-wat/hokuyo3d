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

#ifndef VSSP_H
#define VSSP_H

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <boost/serialization/access.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/shared_array.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/chrono.hpp>

#include <vector>
#include <string>

#include <vsspdefs.h>

namespace vssp
{

class VsspDriver
{
private:
  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket socket_;
  boost::asio::deadline_timer timer_;
  bool closed_;
  AuxFactorArray aux_factor_;

  boost::function<void(
      const vssp::Header &,
      const vssp::RangeHeader &,
      const vssp::RangeIndex &,
      const boost::shared_array<uint16_t> &,
      const boost::shared_array<vssp::XYZI> &,
      const boost::posix_time::ptime &)> cb_point_;
  boost::function<void(
      const vssp::Header &,
      const vssp::AuxHeader &,
      const boost::shared_array<vssp::Aux> &,
      const boost::posix_time::ptime &)> cb_aux_;
  boost::function<void(
      const vssp::Header &,
      const boost::posix_time::ptime &)> cb_ping_;
  boost::function<void(
      const vssp::Header &,
      const std::string &,
      const boost::posix_time::ptime &)> cb_error_;
  boost::function<void(bool)> cb_connect_;
  boost::shared_array<const double> tbl_h_;
  std::vector<boost::shared_array<const TableSincos>> tbl_v_;
  bool tbl_h_loaded_;
  bool tbl_v_loaded_;
  std::vector<bool> tbl_vn_loaded_;
  double timeout_;

  boost::asio::streambuf buf_;

public:
  VsspDriver()
    : socket_(io_service_)
    , timer_(io_service_)
    , closed_(false)
    , aux_factor_(AUX_FACTOR_DEFAULT)
    , cb_point_(0)
    , cb_aux_(0)
    , cb_ping_(0)
    , tbl_h_loaded_(false)
    , tbl_v_loaded_(false)
    , timeout_(1.0)
  {
  }
  void setTimeout(const double to)
  {
    timeout_ = to;
  }
  void connect(const char *ip, const unsigned int port, decltype(cb_connect_) cb)
  {
    cb_connect_ = cb;
    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(ip), port);
    timer_.expires_from_now(
        boost::posix_time::milliseconds(static_cast<long>(1000 * timeout_)));
    timer_.async_wait(boost::bind(&VsspDriver::onTimeoutConnect, this, boost::asio::placeholders::error));
    socket_.async_connect(endpoint, boost::bind(&vssp::VsspDriver::onConnect, this, boost::asio::placeholders::error));
  }
  void registerErrorCallback(decltype(cb_error_) cb)
  {
    cb_error_ = cb;
  }
  void registerCallback(decltype(cb_point_) cb)
  {
    cb_point_ = cb;
  }
  void registerAuxCallback(decltype(cb_aux_) cb)
  {
    cb_aux_ = cb;
  }
  void registerPingCallback(decltype(cb_ping_) cb)
  {
    cb_ping_ = cb;
  }
  void setAutoReset(const bool enable)
  {
    if (enable)
      send(std::string("SET:_ars=1\n"));
    else
      send(std::string("SET:_ars=0\n"));
  }
  [[deprecated("use setHorizontalInterlace() instead of setInterlace()")]]
  void setInterlace(const int itl)
  {
    setHorizontalInterlace(itl);
  }
  void setHorizontalInterlace(const int itl)
  {
    send((boost::format("SET:_itl=0,%02d\n") % itl).str());
  }
  void setVerticalInterlace(const int itl)
  {
    send((boost::format("SET:_itv=0,%02d\n") % itl).str());
  }
  void requestVerticalTable(const int itl = 1)
  {
    tbl_v_.resize(itl);
    tbl_vn_loaded_.resize(itl);
    if (itl == 1)
    {
      send(std::string("GET:tblv\n"));
    }
    else
    {
      for (int i = 0; i < itl; ++i)
      {
        send((boost::format("GET:tv%02d\n") % i).str());
        tbl_vn_loaded_[i] = false;
      }
    }
  }
  void requestHorizontalTable()
  {
    send(std::string("GET:tblh\n"));
  }
  void requestPing()
  {
    send(std::string("PNG\n"));
  }
  void requestAuxData(const bool start = 1)
  {
    send((boost::format("DAT:ax=%d\n") % static_cast<int>(start)).str());
  }
  void requestData(const bool intensity = 1, const bool start = 1)
  {
    if (intensity)
    {
      send((boost::format("DAT:ri=%d\n") % static_cast<int>(start)).str());
    }
    else
    {
      send((boost::format("DAT:ro=%d\n") % static_cast<int>(start)).str());
    }
  }
  void receivePackets()
  {
    timer_.cancel();
    timer_.expires_from_now(
        boost::posix_time::milliseconds(static_cast<long>(1000 * timeout_)));
    timer_.async_wait(boost::bind(&VsspDriver::onTimeout, this, boost::asio::placeholders::error));
    // Read at least 4 bytes.
    // In most case, callback function will be called for each VSSP line.
    boost::asio::async_read(socket_, buf_, boost::asio::transfer_at_least(4),
                            boost::bind(&VsspDriver::onRead, this, boost::asio::placeholders::error));
  }
  bool poll()
  {
    boost::system::error_code ec;
    io_service_.poll(ec);
    if (!ec)
      return true;
    return false;
  }
  void spin()
  {
    io_service_.run();
  }
  void stop()
  {
    io_service_.stop();
  }
  boost::asio::io_service &getIoService()
  {
    return io_service_;
  }

private:
  void send(const std::string cmd)
  {
    boost::shared_ptr<std::string> data(new std::string(cmd));
    boost::asio::async_write(socket_, boost::asio::buffer(*data),
                             boost::bind(&VsspDriver::onSend, this, boost::asio::placeholders::error, data));
  }
  void onTimeoutConnect(const boost::system::error_code &error)
  {
    if (!error)
    {
      closed_ = true;
      io_service_.stop();
    }
  }
  void onTimeout(const boost::system::error_code &error)
  {
    if (!error)
    {
      closed_ = true;
      io_service_.stop();
    }
  }
  void onConnect(const boost::system::error_code &error)
  {
    timer_.cancel();
    if (error)
    {
      closed_ = true;
      cb_connect_(false);
      return;
    }
    cb_connect_(true);
  }
  void onSend(const boost::system::error_code &error, boost::shared_ptr<std::string> data)
  {
    if (error)
    {
      closed_ = true;
      return;
    }
  }
  template <class DATA_TYPE>
  bool rangeToXYZ(
      const vssp::RangeHeader &range_header,
      const vssp::RangeHeaderV2R1 &range_header_v2r1,
      const vssp::RangeIndex &range_index,
      const boost::shared_array<const uint16_t> &index,
      const boost::shared_array<vssp::XYZI> &points)
  {
    if (tbl_vn_loaded_.size() != range_header_v2r1.vertical_interlace)
      return false;

    int i = 0;
    const double h_head = range_header.line_head_h_angle_ratio * 2.0 * M_PI / 65535.0;
    const double h_tail = range_header.line_tail_h_angle_ratio * 2.0 * M_PI / 65535.0;
    const DATA_TYPE *data = boost::asio::buffer_cast<const DATA_TYPE *>(buf_.data());
    const int tv = range_header_v2r1.vertical_field;
    for (int s = 0; s < range_index.nspots; s++)
    {
      const double spot = s + range_header.spot;
      const double h_rad = h_head + (h_tail - h_head) * tbl_h_[spot];
      const double h_cos = cos(h_rad);
      const double h_sin = sin(h_rad);
      const vssp::XYZI dir(tbl_v_[tv][spot].s, tbl_v_[tv][spot].c, h_sin, h_cos);
      for (int e = index[s]; e < index[s + 1]; e++)
        points[i++] = dir * data[e];
    }
    return true;
  }
  void onRead(const boost::system::error_code &error)
  {
    const auto time_read = boost::posix_time::microsec_clock::universal_time();
    const auto length_total = buf_.size();
    if (error == boost::asio::error::eof)
    {
      // Connection closed_
      closed_ = true;
      io_service_.stop();
      return;
    }
    else if (error)
    {
      // Connection error
      closed_ = true;
      io_service_.stop();
    }
    while (true)
    {
      if (buf_.size() < sizeof(vssp::Header))
      {
        break;
      }
      // Read packet Header
      const vssp::Header header = *boost::asio::buffer_cast<const vssp::Header *>(buf_.data());
      if (header.mark != vssp::VSSP_MARK)
      {
        // Invalid packet
        // find VSSP mark
        const uint8_t *data = boost::asio::buffer_cast<const uint8_t *>(buf_.data());
        for (size_t i = 1; i < buf_.size() - sizeof(uint32_t); i++)
        {
          const uint32_t *mark = reinterpret_cast<const uint32_t *>(data + i);
          if (*mark == vssp::VSSP_MARK)
          {
            buf_.consume(i);
            break;
          }
        }
        break;
      }
      if (buf_.size() < header.length)
        break;

      size_t length = header.length - header.header_length;
      buf_.consume(header.header_length);

      do
      {
        switch (header.type)
        {
          case TYPE_ERR:
          case TYPE_ER:
            // Error message
            {
              const std::string data(boost::asio::buffer_cast<const char *>(buf_.data()));
              std::string message(data, 0, header.length - header.header_length - 1);
              if (cb_error_)
                cb_error_(header, message, time_read);
            }
            break;
          default:
            break;
        }
        if (header.status != vssp::STATUS_OK)
          break;

        switch (header.type)
        {
          case TYPE_GET:
            // Response to get command
            {
              const std::string data(boost::asio::buffer_cast<const char *>(buf_.data()));
              std::vector<std::string> lines;
              boost::algorithm::split(lines, data, boost::algorithm::is_any_of("\n\r"));
              if (lines.size() == 0)
                break;

              if (lines[0].compare(0, 7, "GET:tbl") == 0 ||
                  lines[0].compare(0, 6, "GET:tv") == 0)
              {
                if (lines.size() < 2)
                  break;
                std::vector<std::string> cells;
                boost::algorithm::split(cells, lines[1], boost::algorithm::is_any_of(","));

                if (lines[0].compare("GET:tblv") == 0 ||
                    lines[0].compare(0, 6, "GET:tv") == 0)
                {
                  int tv = 0;
                  if (lines[0].compare(0, 6, "GET:tv") == 0)
                    tv = std::stoi(lines[0].substr(6));

                  if (tv >= static_cast<int>(tbl_v_.size()))
                    break;

                  boost::shared_array<TableSincos> tbl_v(new TableSincos[cells.size()]);
                  int i = 0;
                  for (auto &cell : cells)
                  {
                    const double rad(std::strtol(cell.c_str(), nullptr, 16) * 2.0 * M_PI / 65535.0);
                    sincos(rad, &tbl_v[i].s, &tbl_v[i].c);
                    i++;
                  }
                  tbl_v_[tv] = tbl_v;
                  tbl_vn_loaded_[tv] = true;

                  bool load_finished = true;
                  for (auto loaded : tbl_vn_loaded_)
                  {
                    if (!loaded)
                      load_finished = false;
                  }
                  tbl_v_loaded_ = load_finished;
                }
                else if (lines[0].compare("GET:tblh") == 0)
                {
                  boost::shared_array<double> tbl_h(new double[cells.size()]);
                  int i = 0;
                  for (auto &cell : cells)
                  {
                    tbl_h[i] = std::strtol(cell.c_str(), nullptr, 16) / 65535.0;
                    i++;
                  }
                  tbl_h_ = tbl_h;
                  tbl_h_loaded_ = true;
                }
              }
            }
            break;
          case TYPE_SET:
            // Response to set command
            break;
          case TYPE_DAT:
            // Response to data request command
            break;
          case TYPE_VER:
            // Response to version request command
            break;
          case TYPE_PNG:
            // Response to ping command
            if (cb_ping_)
              cb_ping_(header, time_read);
            break;
          case TYPE_RI:
          case TYPE_RO:
            // Range data
            if (!tbl_h_loaded_ || !tbl_v_loaded_ || !cb_point_)
            {
              // Something wrong
              break;
            }
            {
              // Decode range data Header
              const vssp::RangeHeader range_header = *boost::asio::buffer_cast<const vssp::RangeHeader *>(buf_.data());
              vssp::RangeHeaderV2R1 range_header_v2r1 = RANGE_HEADER_V2R1_DEFAULT;
              if (range_header.header_length >= 24)
              {
                range_header_v2r1 = *boost::asio::buffer_cast<const vssp::RangeHeaderV2R1 *>(
                    buf_.data() + sizeof(vssp::RangeHeader));
              }
              buf_.consume(range_header.header_length);
              length -= range_header.header_length;

              // Decode range index Header
              const vssp::RangeIndex range_index = *boost::asio::buffer_cast<const vssp::RangeIndex *>(buf_.data());
              size_t index_length = range_index.index_length;
              buf_.consume(sizeof(vssp::RangeIndex));
              index_length -= sizeof(vssp::RangeIndex);
              length -= sizeof(vssp::RangeIndex);

              // Decode range index
              boost::shared_array<uint16_t> index(new uint16_t[range_index.nspots + 1]);
              std::memcpy(index.get(), boost::asio::buffer_cast<const vssp::RangeIndex *>(buf_.data()),
                          sizeof(uint16_t) * (range_index.nspots + 1));
              buf_.consume(index_length);
              length -= index_length;

              // Decode range data
              boost::shared_array<vssp::XYZI> points(new vssp::XYZI[index[range_index.nspots]]);
              bool success;
              switch (header.type)
              {
              case TYPE_RI:
                // Range and Intensity
                success = rangeToXYZ<vssp::DataRangeIntensity>(
                    range_header, range_header_v2r1, range_index, index, points);
                break;
              case TYPE_RO:
                // Range
                success = rangeToXYZ<vssp::DataRangeOnly>(
                    range_header, range_header_v2r1, range_index, index, points);
                break;
              default:
                success = false;
                break;
              }
              if (!success)
                break;
              cb_point_(header, range_header, range_index, index, points, time_read);
            }
            break;
          case TYPE_AX:
            // Aux data
            {
              // Decode range data Header
              const vssp::AuxHeader aux_header = *boost::asio::buffer_cast<const vssp::AuxHeader *>(buf_.data());
              buf_.consume(aux_header.header_length);
              length -= aux_header.header_length;

              // Decode Aux data
              boost::shared_array<vssp::Aux> auxs(new vssp::Aux[aux_header.data_count]);
              for (int i = 0; i < aux_header.data_count; i++)
              {
                const vssp::AuxData *data = boost::asio::buffer_cast<const vssp::AuxData *>(buf_.data());
                int offset = 0;
                for (AuxId b = vssp::AX_MASK_LAST; b >= vssp::AX_MASK_FIRST; b = static_cast<AuxId>(b - 1))
                {
                  if (aux_header.data_bitfield & (1 << static_cast<int>(b)))
                    auxs[i][b] = aux_factor_[b] * data[offset++].val;
                }
                buf_.consume(sizeof(int32_t) * offset);
                length -= sizeof(int32_t) * offset;
              }
              if (cb_aux_)
                cb_aux_(header, aux_header, auxs, time_read);
            }
            break;
          default:
            break;
        }
      }
      while (false);
      buf_.consume(length);
    }
    receivePackets();
    return;
  }
};

}  // namespace vssp

#endif  // VSSP_H
