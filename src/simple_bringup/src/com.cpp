#include "simple_bringup/com.h"

Com::Com() {}

Com::~Com() {
    this->closeSerial();
}

bool Com::initSerial(std::string port_name, int baudrate) {
    ROS_INFO("port_name: %s", port_name.c_str());
    ROS_INFO("braudrate: %d", baudrate);
    try
    {
        this->ser.setPort(port_name);
        this->ser.setBaudrate(baudrate);
         serial::Timeout to = serial::Timeout::simpleTimeout(5000); //1667 when baud is 57600, 0.6ms
         this->ser.setTimeout(to);                                        //2857 when baud is 115200, 0.35ms
        this->ser.open();    
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR("Unable to open port %s", port_name.c_str());
        return false;
    }

    if(this->ser.isOpen())
        ROS_INFO("Serial Port initialized %s", port_name.c_str());
    else
        return false;
    
    return true;

}

bool Com::closeSerial() {
    this->ser.close();

    if(this->ser.isOpen()) {
        ROS_ERROR("Serial Port close failure");
        return false;
    } else {
      ROS_INFO("Serial Port Closed");
    }
    return true;
}

void Com::serialWrite(std::vector<uint8_t>& data) {
    try {
        for(int i=0; i < data.size(); i++) {
            ROS_INFO("data [%d]: %d", i, data[i]);
        }
        this->ser.write(data);
        this->ser.flush();

    } catch (serial::PortNotOpenedException &e) {
        ROS_ERROR("Port not opened. %s", e.what());
    }
}

uint16_t Com::computeCrc(std::vector<uint8_t>& data)
{
    uint16_t crc = 0xFFFF;
    for (int i=0; i< data.size(); ++i)
    {
        crc = (uint16_t)((crc>>8) ^ CrcTable[(crc ^ data[i]) & 0xFF]);
    }

    return crc;
}


void Com::setTargetVelocity(int16_t leftRPM, int16_t rightRPM) {
     //range -3000 r/min ~ 3000 r/min : I16 value
    leftRPM = leftRPM * this->motor1Direction;
    rightRPM = rightRPM * this->motor2Direction;

    if (leftRPM > 2999) leftRPM = 2999;
    if (leftRPM <- 2999) leftRPM = -2999;
    if (rightRPM > 2999) rightRPM = 2999;
    if (rightRPM <- 2999) rightRPM = -2999;

    std::vector<uint8_t> data;
    data.push_back(0x01); //id
    data.push_back(0x06); //write 1 value to address
    data.push_back(0x20); //0x 2088 : left target velocity
    data.push_back(0x88);
    uint8_t valHigh = (leftRPM >> 8)&0xFF;
    uint8_t valLow = leftRPM & 0xFF;
    data.push_back(valHigh);
    data.push_back(valLow);
    uint16_t crc = this->computeCrc(data);
    uint8_t high = (crc & 0xFF00)>>8;
    uint8_t low = crc & 0xFF;
    data.push_back(low);
    data.push_back(high);

    this->serialWrite(data);

}


void Com::setVelocitiMode() {
    std::vector<uint8_t> data;
    data.push_back(0x01); //id
    data.push_back(0x06); //write 1 value to adress
    data.push_back(0x20);  //0x 200D : control mode
    data.push_back(0x0D);
    data.push_back(0x00); //0x0003  : velocity Mode
    data.push_back(0x03);
    uint16_t crc = this->computeCrc(data);
    uint8_t high = (crc & 0xFF00)>>8;
    uint8_t low = crc & 0xFF;
    data.push_back(low);
    data.push_back(high);


    this->serialWrite(data);
}

void Com::setStop() {
    std::vector<uint8_t> data;
    data.push_back(0x01); //id
    data.push_back(0x06); //write 1 value to adress
    data.push_back(0x20);  //0x 200E : control word
    data.push_back(0x0E);
    data.push_back(0x00); //0x0007  : stop
    data.push_back(0x07); // control word
    uint16_t crc = this->computeCrc(data);
    uint8_t high = (crc & 0xFF00)>>8;
    uint8_t low = crc & 0xFF;
    data.push_back(low);
    data.push_back(high);
    this->serialWrite(data);
}

void Com::setEnable() {
    std::vector<uint8_t> data;
    data.push_back(0x01); //id
    data.push_back(0x06); //write 1 value to adress
    data.push_back(0x20);  //0x 200E : control word
    data.push_back(0x0E);
    data.push_back(0x00); //0x0007  : stop
    data.push_back(0x08); // control word
    uint16_t crc = this->computeCrc(data);
    uint8_t high = (crc & 0xFF00)>>8;
    uint8_t low = crc & 0xFF;
    data.push_back(low);
    data.push_back(high);
    this->serialWrite(data);
}


// Clear faults
void Com::resetAlarm() {
    std::vector<uint8_t> data;
    data.push_back(0x01); //id
    data.push_back(0x06); //write 1 value to adress
    data.push_back(0x20);  //0x 200E : control word
    data.push_back(0x0E);
    data.push_back(0x00); //0x0007  : stop
    data.push_back(0x06); 
    uint16_t crc = this->computeCrc(data);
    uint8_t high = (crc & 0xFF00)>>8;
    uint8_t low = crc & 0xFF;
    data.push_back(low);
    data.push_back(high);
    this->serialWrite(data);
}


// void Com::setZero()
// {
//     std::vector<uint8_t> data;
//     data.push_back(0x01); //id
//     data.push_back(0x06); //write 1 value to adress
//     data.push_back(0x20);  //0x 2005 : clear feedback position
//     data.push_back(0x05);
//     data.push_back(0x00); //0x0000: invalid, 0x0001 left, 0x0002 right,  0x0003  : both left & right
//     data.push_back(0x03);
//     uint16_t crc = ComputeCrc(data);
//     uint8_t high = (crc & 0xFF00)>>8;
//     uint8_t low = crc & 0xFF;
//     data.push_back(low);
//     data.push_back(high);
//     this->serialWrite(data);
// }


// void Com::setLeftAccelerationTime(uint16_t ms)
// {
//     //ms should be 0 ~ 32767
//     std::vector<uint8_t> data;
//     data.push_back(0x01); //id
//     data.push_back(0x06); //write 1 value to adress
//     data.push_back(0x20);  //0x 2080 : acceleration time (left)
//     data.push_back(0x80);
//     uint16_t valHigh = (ms >> 8)&0xFF;
//     uint16_t valLow = ms & 0xFF;
//     data.push_back(valHigh);
//     data.push_back(valLow);
//     uint16_t crc = ComputeCrc(data);
//     uint8_t high = (crc & 0xFF00)>>8;
//     uint8_t low = crc & 0xFF;
//     data.push_back(low);
//     data.push_back(high);
//     SerialWrite(data);
// }


// void Com::setRightAccelerationTime(uint16_t ms)
// {
//     //ms should be 0 ~ 32767
//     std::vector<uint8_t> data;
//     data.push_back(0x01); //id
//     data.push_back(0x06); //write 1 value to adress
//     data.push_back(0x20);  //0x 2081 : acceleration time (right)
//     data.push_back(0x81);
//     uint8_t valHigh = (ms >> 8)&0xFF;
//     uint8_t valLow = ms & 0xFF;
//     data.push_back(valHigh);
//     data.push_back(valLow);
//     uint16_t crc = ComputeCrc(data);
//     uint8_t high = (crc & 0xFF00)>>8;
//     uint8_t low = crc & 0xFF;
//     data.push_back(low);
//     data.push_back(high);
//     SerialWrite(data);
// }


// 뒤에 계속 고치기 - zltech 이랑 메뉴얼 확인

// void Com::setLeftDecelerationTime(uint16_t ms)
// {
//     //ms should be 0 ~ 32767
//     std::vector<uint8_t> data;
//     data.push_back(0x01); //id
//     data.push_back(0x06); //write 1 value to adress
//     data.push_back(0x20);  //0x 2082 : deceleration time (left)
//     data.push_back(0x82);
//     uint8_t valHigh = (ms >> 8)&0xFF;
//     uint8_t valLow = ms & 0xFF;
//     data.push_back(valHigh);
//     data.push_back(valLow);
//     uint16_t crc = ComputeCrc(data);
//     uint8_t high = (crc & 0xFF00)>>8;
//     uint8_t low = crc & 0xFF;
//     data.push_back(low);
//     data.push_back(high);
//     SerialWrite(data);
// }

// void Com::setRightDecelerationTime(uint16_t ms)
// {
//     //ms should be 0 ~ 32767
//     std::vector<uint8_t> data;
//     data.push_back(0x01); //id
//     data.push_back(0x06); //write 1 value to adress
//     data.push_back(0x20);  //0x 2083 : deceleration time (right)
//     data.push_back(0x83);
//     uint8_t valHigh = (ms >> 8)&0xFF;
//     uint8_t valLow = ms & 0xFF;
//     data.push_back(valHigh);
//     data.push_back(valLow);
//     uint16_t crc = ComputeCrc(data);
//     uint8_t high = (crc & 0xFF00)>>8;
//     uint8_t low = crc & 0xFF;
//     data.push_back(low);
//     data.push_back(high);
//     SerialWrite(data);
// }


// void Com::setTargetVelocity(int16_t leftRPM, int16_t rightRPM)
// {
//     //range -3000 r/min ~ 3000 r/min : I16 value
//     leftRPM = leftRPM * motor1Direction;
//     rightRPM = rightRPM * motor2Direction;

//     if (leftRPM > 2999) leftRPM = 2999;
//     if (leftRPM <- 2999) leftRPM = -2999;
//     if (rightRPM > 2999) rightRPM = 2999;
//     if (rightRPM <- 2999) rightRPM = -2999;



//     std::vector<uint8_t> data;
//     data.push_back(0x01); //id
//     data.push_back(0x06); //write 1 value to adress
//     data.push_back(0x20);  //0x 2088 : left target velocity
//     data.push_back(0x88);
//     uint8_t valHigh = (leftRPM >> 8)&0xFF;
//     uint8_t valLow = leftRPM & 0xFF;
//     data.push_back(valHigh);
//     data.push_back(valLow);
//     uint16_t crc = ComputeCrc(data);
//     uint8_t high = (crc & 0xFF00)>>8;
//     uint8_t low = crc & 0xFF;
//     data.push_back(low);
//     data.push_back(high);
//     SerialWrite(data);

//     std::vector<uint8_t> data2;
//     data2.push_back(0x01); //id
//     data2.push_back(0x06); //write 1 value to adress
//     data2.push_back(0x20);  //0x 2089 : right target velocity
//     data2.push_back(0x89);
//     uint8_t valHigh2 = (rightRPM >> 8)&0xFF;
//     uint8_t valLow2 = rightRPM & 0xFF;
//     data2.push_back(valHigh2);
//     data2.push_back(valLow2);
//     uint16_t crc2 = ComputeCrc(data2);
//     uint8_t high2 = (crc2 & 0xFF00)>>8;
//     uint8_t low2 = crc2 & 0xFF;
//     data2.push_back(low2);
//     data2.push_back(high2);
//     SerialWrite(data2);
// }

// void Com::requestMotorCounts()
// {
//     std::vector<uint8_t> data;
//     data.push_back(0x01); //id
//     data.push_back(0x03); //read multiple registers
//     data.push_back(0x20);  //0x 20A7 : left actual motor count (high 16bits)
//     data.push_back(0xA7);
//     data.push_back(0x00); //word count to read
//     data.push_back(0x04); //word count to read
//     uint16_t crc = ComputeCrc(data);
//     uint8_t high = (crc & 0xFF00)>>8;
//     uint8_t low = crc & 0xFF;
//     data.push_back(low);
//     data.push_back(high);
//     SerialWrite(data);
// }