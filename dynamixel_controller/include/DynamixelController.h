/*
 *=====================================================
 * File   :  DynamixelController.h
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

#ifndef DYNAMIXELCONTROLLER_H_
#define DYNAMIXELCONTROLLER_H_


#include <vector>
#include "dynamixel/Dynamixel.h"
#include "packet_control/PacketHandler.h"

namespace ROBOTIS
{


class DynamixelController
{
private:
    Dynamixel                   *dxlList[MAX_ID+1];
    std::vector<SerialPort *>   portList;
    std::vector<PacketHandler *> packetHandlerList;

public:
    std::vector<int>            idList;

    DynamixelController();
    virtual ~DynamixelController();

    bool    initialize();

    bool    addSerialPort(const char* port_name, int baudrate = SerialPort::DEFAULT_BAUDRATE);
    void    addDynamixel(SerialPort* port, int id, const char* joint_name, int model_number, float protocol_ver = 2.0);

    Dynamixel *getDynamixel(int id);

    int     read(int id, int address, long *data, int *error = 0);
    int     read(int id, int address, long *data, LENGTH_TYPE length, int *error = 0);

    int     write(int id, int address, long data, int *error = 0);
    int     write(int id, int address, long data, LENGTH_TYPE length, int *error = 0);

    int     getTorqueEnable(int id, int *enable);
    int     setTorqueEnable(int id, int enable);

    int     getPresentPositionRadian(int id, double *radian);
    int     getPresentPositionValue(int id, long *position);
    int     getPresentVelocity(int id, long *velocity);
    int     getPresentLoad(int id, long *load);

    int     getGoalPositionRadian(int id, double *radian);
    int     setGoalPositionRadian(int id, double radian);
    int     getGoalPositionValue(int id, long *position);
    int     setGoalPositionValue(int id, long position);

    int     getGoalVelocity(int id, long *velocity);
    int     setGoalVelocity(int id, long velocity);

    int     getGoalTorque(int id, long *torque);
    int     setGoalTorque(int id, long torque);

    int     getPositionPGain(int id, int *pgain);
    int     setPositionPGain(int id, int pgain);

    int     getPositionIGain(int id, int *igain);
    int     setPositionIGain(int id, int igain);

    int     getPositionDGain(int id, int *dgain);
    int     setPositionDGain(int id, int dgain);

    int     isMoving(int id, bool *ismoving);

//    ADDR_RETURN_DELAY_TIME;
//    ADDR_RETURN_LEVEL;
//    ADDR_MIN_POSITION_LIMIT;
//    ADDR_MAX_POSITION_LIMIT;
};


}



#endif /* DYNAMIXELCONTROLLER_H_ */
