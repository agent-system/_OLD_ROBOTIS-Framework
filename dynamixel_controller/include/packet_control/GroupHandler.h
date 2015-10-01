/*
 * GroupHandler.h
 *
 *  Created on: 2015. 9. 24.
 *      Author: zerom
 */

#ifndef ROBOTIS_FRAMEWORK_DYNAMIXEL_CONTROLLER_INCLUDE_PACKET_CONTROL_GROUPHANDLER_H_
#define ROBOTIS_FRAMEWORK_DYNAMIXEL_CONTROLLER_INCLUDE_PACKET_CONTROL_GROUPHANDLER_H_


#include <vector>
#include "DynamixelController.h"
#include "packet_control/BulkReadData.h"

namespace ROBOTIS
{

class GroupHandler
{
private:
    DynamixelController *dxlController;
    SerialPort          *comPort;
    PacketHandler       *packetHandler;
	std::vector<BulkReadData> bulkReadData;

public:
	GroupHandler(DynamixelController *controller);
	virtual ~GroupHandler() { }

	bool pushBulkRead(int id, int start_addr, int length = 0);
	bool changeBulkRead(int id, int start_addr, int length = 0);
	void clearBulkRead();

	void runBulkRead();

	bool getReadData(int id, int addr, long *data, int length = 0);
};

}


#endif /* ROBOTIS_FRAMEWORK_DYNAMIXEL_CONTROLLER_INCLUDE_PACKET_CONTROL_GROUPHANDLER_H_ */
