#include "SBGC.h"

#include <stdio.h>
#include <string.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

using namespace std;

int serial_port; 

class nanoSBGC_COMobj : public SBGC_ComObj{
public:
    int init(std::string a){
        
        serial_port = open(a.c_str(),O_RDWR);
       
        struct termios tty;
        if(tcgetattr(serial_port,&tty) != 0){
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
            return 1;
        }

        tty.c_cflag &= ~PARENB; // No parity
        tty.c_cflag &= ~CSTOPB; // One stop bit

        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8; // 8 bits

        tty.c_cflag &= ~CRTSCTS;
        // // tty.c_cflag |= CREAD | CLOCAL;

        tty.c_lflag &= ~ICANON; // Disable canonical mode
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;

        tty.c_lflag &= ~ISIG;
        tty.c_lflag &= ~(IXON | IXOFF | IXANY); // Turn OFF flow control
        
        // Disable Special Handling of bytes in receive
        tty.c_lflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

        tty.c_oflag &= ~OPOST; 
        tty.c_oflag &= ~ONLCR;

        tty.c_cc[VTIME] = 150;
        tty.c_cc[VMIN] = 0;

        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
            printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
            return 1;
        }
    }

    virtual uint16_t getBytesAvailable(){
        uint16_t buf;
        return buf;
    }

    virtual uint8_t readByte(){
        uint8_t buf;
        read(serial_port, &buf, sizeof(buf));
        return buf;
    }

    uint8_t readMessage(u_char *buf, int &len) {
		len = read(serial_port, &buf, sizeof(buf));
	}
	

    virtual void writeByte(uint8_t data){
        write(serial_port, &data, sizeof(data));
    }

    virtual uint16_t getOutEmptySpace(){
        return 0xFFFF;
    }
};


// void getPitchAngle();
// void getYawAngle();

// void goToHomePos();
// void turnLeft();
// void turnRight();
// void turnUp();
// void turnDown();

// void setLimits();


int main(){
    SBGC_Parser sbgc_parser;
    nanoSBGC_COMobj com_obj;
    
    com_obj.init("/dev/ttyUSB0");
    sbgc_parser.init(&com_obj);

    SerialCommand cmd;
    cmd.init(SBGC_CMD_MOTORS_OFF);
    // sbgc_parser.send_cmd(cmd, 0);

    // SerialCommand cmd;
    // cmd.init(SBGC_CMD_REALTIME_DATA);
    sbgc_parser.send_cmd(cmd, 0);

    u_char out[255];
    memset(&out, 0, sizeof(out));
    int numRead = 0;

    numRead = read(serial_port, out, sizeof(out));
    // if(!com_obj.readMessage(out, numRead)){
    //     std::cout << "Could not read from comport. Error " << std::endl;
    //     return 0;
    // }
    // com_obj.readMessage(out, numRead);

    if(numRead < 0) {
        std::cout << "Could not read from commport. Error " << std::endl;
        return 0;
    }
    std::cout << "Read " << numRead << " bytes from commport" << std::endl;
   
    // for (int i = 0; i < numRead; i++) {
    //     printf("%u ", out[i]);
    // } printf("\n");

    if(!sbgc_parser.parse_message(out, sizeof(out))) {
        std::cout << "Failed to parse message." << std::endl;
        return 0;
	}

    SBGC_cmd_realtime_data_t rt_data;

		SerialCommand rec_cmd = sbgc_parser.in_cmd;
		switch(rec_cmd.id) {
		case SBGC_CMD_REALTIME_DATA: {
			uint8_t r = SBGC_cmd_realtime_data_unpack(rt_data, rec_cmd);
			if(r != 0) {
				std::cout << "Failed to unpack messge. Error: " << r << std::endl;
				return 0;
			}
			break;
		}
		default: {
			std::cout << "Failed to unpack message. Unknown message ID! (" << rec_cmd.id << ")" << std::endl;
			return 0; 
			break;
		}
		}

		system("CLS");
		// t_now = clock();
		printf(" r: %3.3f, p: %3.3f, y: %3.3f\n", SBGC_ANGLE_TO_DEGREE(rt_data.imu_angle[0])*5, 
														 SBGC_ANGLE_TO_DEGREE(rt_data.imu_angle[1])*5,
														 SBGC_ANGLE_TO_DEGREE(rt_data.imu_angle[2])*5);

	// cout << "Finished! Average computation time per loop: " << (double)(t_now - t_start) / CLOCKS_PER_SEC / (double)loopmax << std::endl;




    // SBGC_cmd_control_t c = { 0, 0, 0, 0, 0, 0, 0 };

	// c.mode = SBGC_CONTROL_MODE_ANGLE;
	// c.speedROLL = c.speedPITCH = c.speedYAW = 30 * SBGC_SPEED_SCALE;
	// SBGC_cmd_control_send(c, sbgc_parser);
	// c.mode = SBGC_CONTROL_MODE_ANGLE;
	// c.anglePITCH = SBGC_DEGREE_TO_ANGLE(45);
	// c.angleYAW = SBGC_DEGREE_TO_ANGLE(45);
    // SBGC_cmd_control_send(c, sbgc_parser);

    // memset(&out, 0, sizeof(out));
    // numRead = 0;
    
    // if(!com_obj.readMessage(out, numRead)) {
    //     std::cout << "Could not read from commport. Error " << std::endl;
    //     return 0;
    // }
    // std::cout << "Read " << numRead << " bytes from commport" << std::endl;
    // for (int i = 0; i < numRead; i++) {
    //     printf("%i ", out[i]);
    // } printf("\n");

}