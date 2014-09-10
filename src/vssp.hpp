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

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <boost/serialization/access.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/shared_array.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <vector>

namespace vssp
{
	static const uint32_t VSSP_MARK
		= ('V' << 0) | ('S' << 8) | ('S' << 16) | ('P' << 24);
	static const uint32_t STATUS_OK
		= ('0' << 0) | ('0' << 8) | ('0' << 16) | (0xA << 24);
	static const uint32_t STATUS_COMMAND_UNKNOWN
		= ('1' << 0) | ('0' << 8) | ('1' << 16) | (0xA << 24);
	static const uint32_t STATUS_COMMAND_INVALID
		= ('1' << 0) | ('0' << 8) | ('2' << 16) | (0xA << 24);
	static const uint32_t STATUS_COMMAND_INVALUD_METHOD
		= ('1' << 0) | ('0' << 8) | ('3' << 16) | (0xA << 24);
	static const uint32_t STATUS_COMMAND_OUT_OF_RANGE
		= ('1' << 0) | ('0' << 8) | ('4' << 16) | (0xA << 24);
	static const uint32_t STATUS_COMMUNICATION_FAILURE
		= ('2' << 0) | ('0' << 8) | ('1' << 16) | (0xA << 24);
	static const uint32_t STATUS_UNKNOWN_ERROR
		= ('3' << 0) | ('0' << 8) | ('1' << 16) | (0xA << 24);

	static const uint32_t TYPE_GET
		= ('G' << 0) | ('E' << 8) | ('T' << 16) | (':' << 24);
	static const uint32_t TYPE_SET
		= ('S' << 0) | ('E' << 8) | ('T' << 16) | (':' << 24);
	static const uint32_t TYPE_DAT
		= ('D' << 0) | ('A' << 8) | ('T' << 16) | (':' << 24);
	static const uint32_t TYPE_VER
		= ('V' << 0) | ('E' << 8) | ('R' << 16) | (':' << 24);
	static const uint32_t TYPE_PNG
		= ('P' << 0) | ('N' << 8) | ('G' << 16) | (':' << 24);
	static const uint32_t TYPE_RI
		= ('_' << 0) | ('r' << 8) | ('i' << 16) | (':' << 24);
	static const uint32_t TYPE_RO
		= ('_' << 0) | ('r' << 8) | ('o' << 16) | (':' << 24);
	static const uint32_t TYPE_AX
		= ('_' << 0) | ('a' << 8) | ('x' << 16) | (':' << 24);

	enum aux_id
	{
		AX_MASK_ANGVEL_X = 31,
		AX_MASK_ANGVEL_Y = 30,
		AX_MASK_ANGVEL_Z = 29,
		AX_MASK_LINACC_X = 28,
		AX_MASK_LINACC_Y = 27,
		AX_MASK_LINACC_Z = 26,
		AX_MASK_MAG_X    = 25,
		AX_MASK_MAG_Y    = 24,
		AX_MASK_MAG_Z    = 23,
		AX_MASK_TEMP     = 22,
		AX_MASK_FIRST    = 22,
		AX_MASK_LAST     = 31
	};
	static const uint32_t AX_MASK_ANGVEL = (1 << AX_MASK_ANGVEL_X) | (1 << AX_MASK_ANGVEL_Y) |(1 << AX_MASK_ANGVEL_Z);
	static const uint32_t AX_MASK_LINACC = (1 << AX_MASK_LINACC_X) | (1 << AX_MASK_LINACC_Y) |(1 << AX_MASK_LINACC_Z);
	static const uint32_t AX_MASK_MAG = (1 << AX_MASK_MAG_X) | (1 << AX_MASK_MAG_Y) |(1 << AX_MASK_MAG_Z);

#pragma pack(push, 1)
	struct header
	{
		uint32_t mark;
		uint32_t type;
		uint32_t status;
		uint16_t header_length;
		uint16_t length;
		uint32_t received_time_ms;
		uint32_t send_time_ms;
	};
	struct range_header
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
	struct range_index
	{
		uint16_t index_length;
		uint16_t nspots;
	};
	struct data_range_intensity
	{
		uint16_t range_mm;
		uint16_t intensity;
	};
	struct data_range_only
	{
		uint16_t range_mm;
	};
	struct aux_header
	{
		uint16_t header_length;
		uint32_t timestamp_ms;
		uint32_t data_bitfield;
		uint8_t data_count;
		uint8_t data_ms;
	};
#pragma pack(pop)

	struct table_sincos
	{
		double s;
		double c;
	};
	class xyzi
	{
		public:
			double x;
			double y;
			double z;
			double i;

			xyzi()
			{
			};
			xyzi(double &v_sin, double &v_cos, double &h_sin, double &h_cos)
			{
				i = 0;
				x = v_cos * h_cos;
				y = v_cos * h_sin;
				z = v_sin;
			};
			xyzi operator *(const data_range_intensity &data)
			{
				xyzi ret = *this;
				double r = data.range_mm * 0.001;
				ret.i = data.intensity;
				ret.x *= r;
				ret.y *= r;
				ret.z *= r;
				return ret;
			};
			xyzi operator *(const data_range_only &data)
			{
				xyzi ret = *this;
				double r = data.range_mm * 0.001;
				ret.i = 0;
				ret.x *= r;
				ret.y *= r;
				ret.z *= r;
				return *this;
			};
	};
	class aux
	{
		public:
			struct
			{
				double x;
				double y;
				double z;
			} ang_vel;
			struct
			{
				double x;
				double y;
				double z;
			} lin_acc;
			struct
			{
				double x;
				double y;
				double z;
			} mag;
			double temp;

			aux()
			{
				ang_vel.x = ang_vel.y = ang_vel.z = 0.0;
				lin_acc.x = lin_acc.y = lin_acc.z = 0.0;
				mag.x = mag.y = mag.z = 0.0;
				temp = 0.0;
			};
			double &operator[](aux_id id)
			{
				switch(id)
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
			};
	};


	class vsspDriver;
};

class vssp::vsspDriver
{
	public:
		vsspDriver() : 
			socket(io_service),
			timer(io_service),
			closed(false),
			tblH_loaded(false),
			tblV_loaded(false),
			timeout(1.0)
		{
		};
		void setTimeout(double to)
		{
			timeout = to;
		};
		void connect(const char *ip, unsigned int port, boost::function<void(bool)> cb)
		{
			cbConnect = cb;
			boost::asio::ip::tcp::endpoint endpoint(
					boost::asio::ip::address::from_string(ip), port);
			timer.expires_from_now(boost::posix_time::seconds(timeout));
			timer.async_wait(boost::bind(&vsspDriver::on_timeout_connect, this,
						boost::asio::placeholders::error));
			socket.async_connect(endpoint, 
					boost::bind(&vssp::vsspDriver::on_connect, 
						this, 
						boost::asio::placeholders::error));
		};
		void registerCallback(boost::function<void(
					const vssp::header&, 
					const vssp::range_header&, 
					const vssp::range_index&, 
					const boost::shared_array<vssp::xyzi>&)> cb)
		{
			cbPoint = cb;
		};
		void registerAuxCallback(boost::function<void(
					const vssp::header&, 
					const vssp::aux_header&, 
					const boost::shared_array<vssp::aux>&)> cb)
		{
			cbAux = cb;
		};
		void setInterlace(int itl)
		{
			send((boost::format("SET:_itl=0,%02d\n") % itl).str());
		};
		void requestVerticalTable()
		{
			send(std::string("GET:tblv\n"));
		};
		void requestHorizontalTable()
		{
			send(std::string("GET:tblh\n"));
		};
		void requestPing()
		{
			send(std::string("PNG\n"));
		};
		void requestAuxData(bool start = 1)
		{
			send((boost::format("DAT:ax=%d\n") % (int)start).str());
		}
		void requestData(bool intensity = 1, bool start = 1)
		{
			if(intensity)
			{
				send((boost::format("DAT:ri=%d\n") % (int)start).str());
			}
			else
			{
				send((boost::format("DAT:ro=%d\n") % (int)start).str());
			}
		};
		void receivePackets()
		{
			timer.cancel();
			timer.expires_from_now(boost::posix_time::seconds(timeout));
			timer.async_wait(boost::bind(&vsspDriver::on_timeout, this, 
						boost::asio::placeholders::error));
			boost::asio::async_read(
					socket,
					buf,
					boost::asio::transfer_at_least(65536),
					boost::bind(
						&vsspDriver::on_read, 
						this, 
						boost::asio::placeholders::error)
					);
		};
		bool poll()
		{
			if(!closed)
			{
				boost::system::error_code ec;
				io_service.poll(ec);
				if(!ec) return true;
				closed = true;
			}
			return false;
		};
	private:
		boost::asio::io_service io_service;
		boost::asio::ip::tcp::socket socket;
		boost::asio::deadline_timer timer;
		bool closed;

		boost::function<void(const vssp::header&,
				const vssp::range_header&, 
				const vssp::range_index&, 
				const boost::shared_array<vssp::xyzi>&)> cbPoint;
		boost::function<void(const vssp::header&,
				const vssp::aux_header&, 
				const boost::shared_array<vssp::aux>&)> cbAux;
		boost::function<void(bool)> cbConnect;
		boost::shared_array<double> tblH;
		boost::shared_array<table_sincos> tblV;
		bool tblH_loaded;
		bool tblV_loaded;
		double timeout;

		boost::asio::streambuf buf;

		void send(std::string cmd)
		{
			boost::shared_ptr<std::string> data(new std::string(cmd));
			boost::asio::async_write(
					socket, 
					boost::asio::buffer(*data),
					boost::bind(
						&vsspDriver::on_send,
						this,
						boost::asio::placeholders::error,
						data)
					);
		};
		void on_timeout_connect(const boost::system::error_code& error)
		{
			if(!error)
			{
				closed = true;
				socket.cancel();
			}
		}
		void on_timeout(const boost::system::error_code& error)
		{
			if(!error)
			{
				closed = true;
				socket.cancel();
			}
		}
		void on_connect(const boost::system::error_code& error)
		{
			timer.cancel();
			if(error)
			{
				closed = true;
				cbConnect(false);
				return;
			}
			cbConnect(true);
		};
		void on_send(const boost::system::error_code& error, 
				boost::shared_ptr<std::string> data)
		{
			if(error)
			{
				closed = true;
				return;
			}
		};
		template<class DATA_TYPE>
		void rangeToXYZ(const vssp::range_header &range_header,
				const vssp::range_index &range_index, 
				boost::shared_array<uint16_t> &index,
				boost::shared_array<vssp::xyzi> &points)
		{
			int i = 0;

			double h_head = range_header.line_head_h_angle_ratio * 2.0 * M_PI / 65535.0;
			double h_tail = range_header.line_tail_h_angle_ratio * 2.0 * M_PI / 65535.0;
			const DATA_TYPE *data =
				boost::asio::buffer_cast
				<const DATA_TYPE*>(buf.data());
			for(int s = 0; s < range_index.nspots; s ++)
			{
				double spot = s + range_header.spot;
				double h_rad = h_head + (h_tail - h_head) * tblH[spot];
				double h_cos = cos(h_rad);
				double h_sin = sin(h_rad);
				vssp::xyzi dir(tblV[spot].s, tblV[spot].c, h_sin, h_cos);
				for(int e = index[s]; e < index[s+1]; e++)
					points[i++] = dir * data[e];
			}
		};
		void on_read(const boost::system::error_code& error)
		{
			if(error == boost::asio::error::eof)
			{
				// Connection closed
				closed = true;
				return;
			}
			else if(error)
			{
				// Connection error
				closed = true;
				return;
			}
			while(true)
			{
				if(buf.size() < sizeof(vssp::header))
				{
					break;
				}
				// Read packet header
				const vssp::header header = 
					*boost::asio::buffer_cast<const vssp::header*>(buf.data());
				if(header.mark != vssp::VSSP_MARK)
				{
					// Exception
					break;
				}
				if(buf.size() < header.length) break;

				size_t length = header.length - header.header_length;
				buf.consume(header.header_length);

				do
				{
					if(header.status != vssp::STATUS_OK) break;

					switch(header.type)
					{
					case TYPE_GET:
						// Response to get command
						{
							const std::string data(boost::asio::buffer_cast<const char*>(buf.data()));
							std::vector<std::string> lines;
							boost::algorithm::split(lines, data, boost::algorithm::is_any_of("\n\r"));
							if(lines.size() == 0) break;

							if(lines[0].compare(0, 7, "GET:tbl") == 0)
							{
								if(lines.size() < 2) break;
								std::vector<std::string> cells;
								boost::algorithm::split(cells, lines[1], boost::algorithm::is_any_of(","));
								int i = 0;

								if(lines[0].compare("GET:tblv") == 0)
								{
									tblV.reset(new table_sincos[cells.size()]);
									BOOST_FOREACH(std::string &cell, cells)
									{
										double rad = (double)std::strtol(cell.c_str(), NULL, 16) * 2.0 * M_PI / 65535;
										sincos(rad, &tblV[i].s, &tblV[i].c);
										i ++;
									}
									tblV_loaded = true;
								}
								else if(lines[0].compare("GET:tblh") == 0)
								{
									tblH.reset(new double[cells.size()]);
									BOOST_FOREACH(std::string &cell, cells)
									{
										tblH[i] = (double)std::strtol(cell.c_str(), NULL, 16) / 65535;
										i ++;
									}
									tblH_loaded = true;
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
						break;
					case TYPE_RI:
					case TYPE_RO:
						// Range data
						if(!tblH_loaded || !tblV_loaded || cbPoint.empty())
						{
							// Something wrong
							break;
						}
						{
							// Decode range data header
							const vssp::range_header range_header = 
								*boost::asio::buffer_cast<const vssp::range_header*>(buf.data());
							buf.consume(range_header.header_length);
							length -= range_header.header_length;

							// Decode range index header
							const vssp::range_index range_index = 
								*boost::asio::buffer_cast<const vssp::range_index*>(buf.data());
							size_t index_length = range_index.index_length;
							buf.consume(sizeof(vssp::range_index));
							index_length -= sizeof(vssp::range_index);
							length -= sizeof(vssp::range_index);

							// Decode range index
							boost::shared_array<uint16_t> index(new uint16_t[range_index.nspots+1]);
							std::memcpy(index.get(), 
									boost::asio::buffer_cast<const vssp::range_index*>(buf.data()),
									sizeof(uint16_t) * (range_index.nspots+1));
							buf.consume(index_length);
							length -= index_length;

							// Decode range data
							boost::shared_array<vssp::xyzi> points(new vssp::xyzi[index[range_index.nspots]]);
							switch(header.type)
							{
							case TYPE_RI:
								// Range and Intensity
								rangeToXYZ<vssp::data_range_intensity>
									(range_header, range_index, index, points);
								break;
							case TYPE_RO:
								// Range
								rangeToXYZ<vssp::data_range_only>
									(range_header, range_index, index, points);
								break;
							}
							cbPoint(header, range_header, range_index, points);
						}
						break;
					case TYPE_AX:
						// Aux data
						{
							// Decode range data header
							const vssp::aux_header aux_header = 
								*boost::asio::buffer_cast<const vssp::aux_header*>(buf.data());
							buf.consume(aux_header.header_length);
							length -= aux_header.header_length;

							// Decode aux data
							boost::shared_array<vssp::aux> auxs(new vssp::aux[aux_header.data_count]);
							for(int i = 0; i < aux_header.data_count; i ++)
							{
								const int32_t *data =
									boost::asio::buffer_cast
									<const int32_t*>(buf.data());
								int offset = 0;
								for(aux_id b = vssp::AX_MASK_LAST; 
										b >= vssp::AX_MASK_FIRST; 
										b = static_cast<aux_id>(b - 1))
								{
									if(aux_header.data_bitfield & (1 << b))
										auxs[i][b] = 1.0 * data[offset ++];
								}
								buf.consume(sizeof(int32_t) * offset);
								length -= sizeof(int32_t) * offset;
							}
							if(!cbAux.empty()) cbAux(header, aux_header, auxs);
						}
						break;
					}
				}
				while(false);
				buf.consume(length);
			}
			receivePackets();
			return;
		};
};

