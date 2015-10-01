/*
 *=====================================================
 * File   :  DynamixelController.cpp
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

#include <ros/ros.h>
#include "DynamixelController.h"

using namespace ROBOTIS;

DynamixelController::DynamixelController()
{
    packetHandlerList.push_back(PacketHandler::getPacketHandler(1.0));
    packetHandlerList.push_back(PacketHandler::getPacketHandler(2.0));
}

DynamixelController::~DynamixelController()
{

}

bool DynamixelController::initialize()
{
    ros::NodeHandle nh("~");

    bool result = false;
    int port_num;
    nh.getParam("port_num", port_num);

    if(port_num < 1)
        return false;

    for(int i = 0; i < port_num; i++)
    {
        std::stringstream dev_idx;
        dev_idx << i;

        std::string dev_name;
        nh.getParam("serial_ports/dxl_tty"+dev_idx.str()+"/port_name", dev_name);

        int baudrate;
        nh.getParam("serial_ports/dxl_tty"+dev_idx.str()+"/baudrate", baudrate);

        result = addSerialPort(dev_name.c_str(), baudrate);
        if(result == false)
            return result;

        ROS_INFO("Port [ %s ] added.", dev_name.c_str());

        int dxl_num;
        nh.getParam("serial_ports/dxl_tty"+dev_idx.str()+"/dxl_num", dxl_num);

        int cnt = 0;
        if(dxl_num > 0)
        {
            for(int c = 0; c < dxl_num; c++)
            {
                int id = 0, model_num = 0;
                double prot_ver = 2.0;
                std::string joint_name = "";

                std::stringstream dxl_idx;
                dxl_idx << c;
                nh.getParam("serial_ports/dxl_tty"+dev_idx.str()+"/dxl"+dxl_idx.str()+"/id", id);
                nh.getParam("serial_ports/dxl_tty"+dev_idx.str()+"/dxl"+dxl_idx.str()+"/model_num", model_num);
                nh.getParam("serial_ports/dxl_tty"+dev_idx.str()+"/dxl"+dxl_idx.str()+"/protocol", prot_ver);
                nh.getParam("serial_ports/dxl_tty"+dev_idx.str()+"/dxl"+dxl_idx.str()+"/joint_name", joint_name);

                addDynamixel(portList.back(), id, joint_name.c_str(), model_num, (float)prot_ver);
                ROS_INFO("  -Dynamixel added (total:%3d) - [ID:%03d] %s", ++cnt, id, joint_name.c_str());
            }
        }
    }
}

bool DynamixelController::addSerialPort(const char* port_name, int baudrate)
{
    portList.push_back(new SerialPort(port_name));
    portList.back()->openPort();
    portList.back()->changeBaudRate(baudrate);
    return true;
}

void DynamixelController::addDynamixel(SerialPort* port, int id, const char* joint_name, int model_number, float protocol_ver)
{
    idList.push_back(id);
    dxlList[id] = Dynamixel::getInstance(port, id, joint_name, model_number, protocol_ver);
}

Dynamixel *DynamixelController::getDynamixel(int id)
{
    return dxlList[id];
}

int DynamixelController::read(int id, int address, long *data, int *error)
{
    if(std::find(idList.begin(), idList.end(), id) == idList.end() || dxlList[id] == NULL) // id is not exist.
        return COMM_TX_ERROR;

    return dxlList[id]->read(address, data, error);
}

int DynamixelController::read(int id, int address, long *data, LENGTH_TYPE length, int *error)
{
    if(std::find(idList.begin(), idList.end(), id) == idList.end() || dxlList[id] == NULL) // id is not exist.
        return COMM_TX_ERROR;

    return dxlList[id]->read(address, data, length, error);
}

int DynamixelController::write(int id, int address, long data, int *error)
{
    if(std::find(idList.begin(), idList.end(), id) == idList.end() || dxlList[id] == NULL) // id is not exist.
        return COMM_TX_ERROR;

    return dxlList[id]->write(address, data, error);
}

int DynamixelController::write(int id, int address, long data, LENGTH_TYPE length, int *error)
{
    if(id == BROADCAST_ID)
    {
        for(int i = 0; i < portList.size(); i++)
        {
            for(int j = 0; j < packetHandlerList.size(); j++)
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

                packetHandlerList[j]->write(portList[i], id, address, (int)length, write_data);
            }
        }
    }
    else if(std::find(idList.begin(), idList.end(), id) == idList.end() || dxlList[id] == NULL) // id is not exist.
    {
        return COMM_TX_ERROR;
    }
    else
    {
        return dxlList[id]->write(address, data, length, error);
    }
}

int DynamixelController::getTorqueEnable(int id, int *enable)
{
    return dxlList[id]->read(dxlList[id]->ADDR_TORQUE_ENABLE, (long*)enable);
}

int DynamixelController::setTorqueEnable(int id, int enable)
{
    return dxlList[id]->write(dxlList[id]->ADDR_TORQUE_ENABLE, enable);
}

int DynamixelController::getPresentPositionRadian(int id, double *radian)
{
    long position = 0;
    int result = dxlList[id]->read(dxlList[id]->ADDR_PRESENT_POSITION, &position);

    if(result == 0)
        *radian = dxlList[id]->value2Rad(position);
    else
        radian = 0;

    return result;
}

int DynamixelController::getPresentPositionValue(int id, long *position)
{
    return dxlList[id]->read(dxlList[id]->ADDR_PRESENT_POSITION, position);;
}

int DynamixelController::getPresentVelocity(int id, long *velocity)
{
    return dxlList[id]->read(dxlList[id]->ADDR_PRESENT_VELOCITY, velocity);;
}

int DynamixelController::getPresentLoad(int id, long *load)
{
    return dxlList[id]->read(dxlList[id]->ADDR_PRESENT_LOAD, load);;
}

int DynamixelController::getGoalPositionRadian(int id, double *radian)
{
    long position = 0;
    int result = dxlList[id]->read(dxlList[id]->ADDR_GOAL_POSITION, &position);

    if(result == 0)
        *radian = dxlList[id]->value2Rad(position);
    else
        radian = 0;

    return result;
}

int DynamixelController::setGoalPositionRadian(int id, double radian)
{
    long position = dxlList[id]->rad2Value(radian);

    return dxlList[id]->write(dxlList[id]->ADDR_GOAL_POSITION, position);
}

int DynamixelController::getGoalPositionValue(int id, long *position)
{
    return dxlList[id]->read(dxlList[id]->ADDR_GOAL_POSITION, position);;
}

int DynamixelController::setGoalPositionValue(int id, long position)
{
    return dxlList[id]->write(dxlList[id]->ADDR_GOAL_POSITION, position);
}

int DynamixelController::getGoalVelocity(int id, long *velocity)
{
    return dxlList[id]->read(dxlList[id]->ADDR_GOAL_VELOCITY, velocity);;
}

int DynamixelController::setGoalVelocity(int id, long velocity)
{
    return dxlList[id]->write(dxlList[id]->ADDR_GOAL_VELOCITY, velocity);
}

int DynamixelController::getGoalTorque(int id, long *torque)
{
    return dxlList[id]->read(dxlList[id]->ADDR_GOAL_TORQUE, torque);;
}

int DynamixelController::setGoalTorque(int id, long torque)
{
    return dxlList[id]->write(dxlList[id]->ADDR_GOAL_TORQUE, torque);
}

int DynamixelController::getPositionPGain(int id, int *pgain)
{
    return dxlList[id]->read(dxlList[id]->ADDR_POSITION_P_GAIN, (long*)pgain);
}

int DynamixelController::setPositionPGain(int id, int pgain)
{
    return dxlList[id]->write(dxlList[id]->ADDR_POSITION_P_GAIN, pgain);
}

int DynamixelController::getPositionIGain(int id, int *igain)
{
    return dxlList[id]->read(dxlList[id]->ADDR_POSITION_I_GAIN, (long*)igain);
}

int DynamixelController::setPositionIGain(int id, int igain)
{
    return dxlList[id]->write(dxlList[id]->ADDR_POSITION_I_GAIN, igain);
}

int DynamixelController::getPositionDGain(int id, int *dgain)
{
    return dxlList[id]->read(dxlList[id]->ADDR_POSITION_D_GAIN, (long*)dgain);
}

int DynamixelController::setPositionDGain(int id, int dgain)
{
    return dxlList[id]->write(dxlList[id]->ADDR_POSITION_D_GAIN, dgain);
}

int DynamixelController::isMoving(int id, bool *ismoving)
{
    long _moving;
    int result = dxlList[id]->read(dxlList[id]->ADDR_MOVING, &_moving);
    if(result == COMM_SUCCESS)
    {
        if(_moving == 0)
            *ismoving = false;
        else
            *ismoving = true;
    }
    return result;
}
