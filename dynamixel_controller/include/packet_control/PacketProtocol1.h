/*
 *=====================================================
 * File   :  PacketProtocol1.h
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

#ifndef PACKETPROTOCOL1_H_
#define PACKETPROTOCOL1_H_

#include "packet_control/PacketHandler.h"

namespace ROBOTIS
{

class PacketProtocol1 : public PacketHandler
{
public:
    PacketProtocol1();
    virtual ~PacketProtocol1() { };

    virtual int txPacket(SerialPort *port, unsigned char *txpacket);
    virtual int rxPacket(SerialPort *port, unsigned char *rxpacket);
    virtual int txRxPacket(SerialPort *port, unsigned char *txpacket, unsigned char *rxpacket, int *error = 0);

    virtual int bulkReadTxPacket(SerialPort *port, std::vector<BulkReadData>& data);
    virtual int bulkReadRxPacket(SerialPort *port, std::vector<BulkReadData>& data);

    virtual int ping(SerialPort *port, int id, int *error = 0);

    virtual int read(SerialPort *port, int id, int address, int length, unsigned char *data, int *error = 0);
    virtual int write(SerialPort *port, int id, int address, int length, unsigned char *data, int *error = 0);
    virtual int syncWrite(SerialPort *port, int start_addr, int data_length, unsigned char* param, int param_length);
};

}

#endif /* PACKETPROTOCOL1_H_ */
