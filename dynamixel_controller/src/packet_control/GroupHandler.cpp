/*
 * GroupHandler.cpp
 *
 *  Created on: 2015. 9. 24.
 *      Author: zerom
 */


#include "packet_control/GroupHandler.h"

using namespace ROBOTIS;

GroupHandler::GroupHandler(DynamixelController *controller)
    : dxlController(controller), comPort(0), packetHandler(0)
{ }

bool GroupHandler::pushBulkRead(int id, int start_addr, int length)
{
    if(start_addr <= 0 || length < 0)
        return false;

    if(length == 0)
    {
        length = dxlController->getDynamixel(id)->getAddrLength(start_addr);
        if(length < 0)
            return false;
    }

    if(bulkReadData.size() != 0)
    {
        for(unsigned int i = 0; i < bulkReadData.size(); i++)
        {
            if(bulkReadData[i].id == id)
                return false;   // already exist.
        }

        if(comPort->getPortName() != dxlController->getDynamixel(id)->getSerialPort()->getPortName())
            return false;
        if(dxlController->getDynamixel(bulkReadData[0].id)->PROTOCOL_VERSION != dxlController->getDynamixel(id)->PROTOCOL_VERSION)
            return false;
    }
    else
    {
        comPort = dxlController->getDynamixel(id)->getSerialPort();
        packetHandler = PacketHandler::getPacketHandler(dxlController->getDynamixel(id)->PROTOCOL_VERSION);
    }

    BulkReadData data;
    data.id = id;
    data.startAddr = start_addr;
    data.dataLength = length;
    data.data = new unsigned char[length];

    data.error = 0;
    data.commResult = -1;

    bulkReadData.push_back(data);

    return true;
}

bool GroupHandler::changeBulkRead(int id, int start_addr, int length)
{
    if(start_addr <= 0 || length < 0)
        return false;

    if(length == 0)
    {
        length = dxlController->getDynamixel(id)->getAddrLength(start_addr);
        if(length < 0)
            return false;
    }

	if(bulkReadData.size() == 0)
		return false;

	for(unsigned int i = 0; i < bulkReadData.size(); i++)
	{
		if(bulkReadData[i].id == id)
		{
			bulkReadData[i].startAddr = start_addr;
			bulkReadData[i].dataLength = length;
			if(bulkReadData[i].data != 0)
				delete[] bulkReadData[i].data;
			bulkReadData[i].data = new unsigned char[length];

			return true;
		}
	}

	return false;
}

void GroupHandler::clearBulkRead()
{
	if(bulkReadData.size() != 0)
	{
		for(unsigned int i = 0; i < bulkReadData.size(); i++)
		{
			if(bulkReadData[i].data != 0)
				delete[] bulkReadData[i].data;
		}
		bulkReadData.clear();
		comPort = 0;
		packetHandler = 0;
	}
}

void GroupHandler::runBulkRead()
{
	if(bulkReadData.size() == 0)
		return;

	for(unsigned int i = 0; i < bulkReadData.size(); i++)
		bulkReadData[i].commResult = -1;

	packetHandler->bulkReadTxPacket(comPort, bulkReadData);
	packetHandler->bulkReadRxPacket(comPort, bulkReadData);
}

bool GroupHandler::getReadData(int id, int addr, long *data, int length)
{
    if(addr <= 0 || length < 0)
        return false;

    if(length == 0)
    {
        length = dxlController->getDynamixel(id)->getAddrLength(addr);
        if(length < 0)
            return false;
    }

    if(bulkReadData.size() == 0)
        return false;

    for(unsigned int i = 0; i < bulkReadData.size(); i++)
    {
        if(bulkReadData[i].id == id)
        {
            if (bulkReadData[i].commResult == -1 ||
                addr < bulkReadData[i].startAddr ||
                bulkReadData[i].startAddr + bulkReadData[i].dataLength - length < addr)
            {
                return false;
            }

            switch(length)
            {
            case 1:
                *data = bulkReadData[i].data[addr - bulkReadData[i].startAddr];
                break;
            case 2:
                *data = DXL_MAKEWORD(bulkReadData[i].data[addr - bulkReadData[i].startAddr],
                                     bulkReadData[i].data[addr - bulkReadData[i].startAddr + 1]);
                break;
            case 4:
                *data = DXL_MAKEDWORD(DXL_MAKEWORD(bulkReadData[i].data[addr - bulkReadData[i].startAddr],
                                                   bulkReadData[i].data[addr - bulkReadData[i].startAddr + 1]),
                                      DXL_MAKEWORD(bulkReadData[i].data[addr - bulkReadData[i].startAddr + 2],
                                                   bulkReadData[i].data[addr - bulkReadData[i].startAddr + 3]));
                break;
            default:
                return false;
            }
            return true;
        }
    }
    return false;
}
