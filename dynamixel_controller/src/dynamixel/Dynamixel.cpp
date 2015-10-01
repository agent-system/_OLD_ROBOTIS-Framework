/*
 *=====================================================
 * File   :  DXL.cpp
 * Author :  zerom <zerom@robotis.com>
 * Copyright (C) ROBOTIS, 2015
 *=====================================================
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <string.h>
#include "dynamixel/MX28.h"
#include "dynamixel/DXLPRO.h"
#include "dynamixel/UNKNOWN.h"
#include "dynamixel/Dynamixel.h"

using namespace ROBOTIS;

Dynamixel::Dynamixel(SerialPort *port, long min_value, long max_value, long center_value, double min_radian, double max_radian) :
    comPort(port),
    MIN_VALUE(min_value), MAX_VALUE(max_value), CENTER_VALUE(center_value), MIN_RADIAN(min_radian), MAX_RADIAN(max_radian)
{ }

int Dynamixel::getAddrLength(int addr)
{
    std::map<int, int>::iterator iter_addr_length;
    iter_addr_length = addr_length.find(addr);
    if(iter_addr_length != addr_length.end())
        return iter_addr_length->second;

    return -1;
}

Dynamixel *Dynamixel::getInstance(SerialPort *port, int id, const char *joint_name, int model_number, float protocol_ver)
{
    Dynamixel *ret = 0;
    switch(model_number)
    {
    case 29:    // MX-28
        ret = new MX28(port);
        break;

    case 35072: // L42-10-S300-R
        ret = new DXLPRO(port, -2047, 2048, 0, -PI, PI);
        break;

    case 38176: // L54-50-S290-R
        ret = new DXLPRO(port, -103860, 103860, 0, -PI, PI);
        break;

    case 37928: // L54-30-S400-R
        ret = new DXLPRO(port, -144198, 144198, 0, -PI, PI);
        break;

    case 38152: // L54-50-S500-R
    case 37896: // L54-30-S500-R
        ret = new DXLPRO(port, -180684, 180684, 0, -PI, PI);
        break;

    case 43288: // M42-10-S260-R
        ret = new DXLPRO(port, -131584, 131584, 0, -PI, PI);
        break;

    case 46096: // M54-40-S250-R
    case 46352: // M54-60-S250-R
        ret = new DXLPRO(port, -125700, 125700, 0, -PI, PI);
        break;

    case 51200: // H42-20-S300-R
        ret = new DXLPRO(port, -151900, 151900, 0, -PI, PI);
        break;

    case 53768: // H54-100-S500-R
    case 54024: // H54-200-S500-R
    case 54152: // H54-200-B500-R
        ret = new DXLPRO(port, -250950, 250950, 0, -PI, PI);
        break;

    default:
    	ret = new UNKNOWN(port);
        break;
    }

    if(ret != 0)
    {
        ret->PROTOCOL_VERSION = protocol_ver;
        ret->packetHandler = PacketHandler::getPacketHandler(protocol_ver);
        ret->ID = id;
        strcpy(ret->jointName, joint_name);
    }

    return ret;
}

char* Dynamixel::getJointName()
{
    return jointName;
}

SerialPort* Dynamixel::getSerialPort()
{
    return comPort;
}

int Dynamixel::read(int address, long *data, int *error)
{
    int data_len = getAddrLength(address);

    if(data_len < 0)
    	return COMM_NOT_AVAILABLE;

    unsigned char *read_data = new unsigned char[data_len];

    int result = packetHandler->read(comPort, ID, address, data_len, read_data, error);

    switch(data_len)
    {
    case 1:
        *data = read_data[0];
        break;
    case 2:
        *data = DXL_MAKEWORD(read_data[0], read_data[1]);
        break;
    case 4:
        *data = DXL_MAKEDWORD(DXL_MAKEWORD(read_data[0], read_data[1]), DXL_MAKEWORD(read_data[2], read_data[3]));
        break;
    default:
        break;
    }

    return result;
}

int Dynamixel::read(int address, long *data, LENGTH_TYPE length, int *error)
{
    unsigned char *read_data = new unsigned char[length];

    int result = packetHandler->read(comPort, ID, address, length, read_data, error);

    switch(length)
    {
    case 1:
        *data = read_data[0];
        break;
    case 2:
        *data = DXL_MAKEWORD(read_data[0], read_data[1]);
        break;
    case 4:
        *data = DXL_MAKEDWORD(DXL_MAKEWORD(read_data[0], read_data[1]), DXL_MAKEWORD(read_data[2], read_data[3]));
        break;
    default:
        break;
    }

    return result;
}

int Dynamixel::write(int address, long data, int *error)
{
    int data_len = getAddrLength(address);

    if(data_len < 0)
    	return COMM_NOT_AVAILABLE;

    unsigned char *write_data = new unsigned char[data_len];

    switch(data_len)
    {
    case 1:
        write_data[0] = data;
        break;
    case 2:
        write_data[0] = DXL_LOBYTE(data);
        write_data[1] = DXL_HIBYTE(data);
        break;
    case 4:
        write_data[0] = DXL_LOBYTE(DXL_LOWORD(data));
        write_data[1] = DXL_HIBYTE(DXL_LOWORD(data));
        write_data[2] = DXL_LOBYTE(DXL_HIWORD(data));
        write_data[3] = DXL_HIBYTE(DXL_HIWORD(data));
        break;
    default:
        break;
    }

    return packetHandler->write(comPort, ID, address, data_len, write_data, error);
}

int Dynamixel::write(int address, long data, LENGTH_TYPE length, int *error)
{
    unsigned char *write_data = new unsigned char[length];

    switch(length)
    {
    case 1:
        write_data[0] = data;
        break;
    case 2:
        write_data[0] = DXL_LOBYTE(data);
        write_data[1] = DXL_HIBYTE(data);
        break;
    case 4:
        write_data[0] = DXL_LOBYTE(DXL_LOWORD(data));
        write_data[1] = DXL_HIBYTE(DXL_LOWORD(data));
        write_data[2] = DXL_LOBYTE(DXL_HIWORD(data));
        write_data[3] = DXL_HIBYTE(DXL_HIWORD(data));
        break;
    default:
        break;
    }

    return packetHandler->write(comPort, ID, address, length, write_data, error);
}
