/**************************************************
 * Author: David Keltgen                           *
 * Company: Montana State University: MOSES LAB    *
 * File Name: roesim.c              *
 * Date:  November 2014                              *
 * Description: The sole purpose of roesim.c is to 
 *              simulate the command functionality of the ROE.
 *              It will simply wait for a command from the 
 *              flight computer, and send a response.
 *              It will then go back into the loop and 
 *              wait for the next signal.         *
 **************************************************/

#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>

#include "roesim.h"

/*
 * 
 */
int main(int argc, char** argv) {

    int result;
    char ack;

    connection = activateConnection();

    /* Continuously do this */
    while (1) {

        /* Wait on the Exit default command (0x41)*/
        result = receiveCmd(connection, &ack, 1, 0x41);

        if (result == 1) {
            /* Flight Computer sent command, tell it that this program received it, by sending ack*/
            write(connection, (char *) 0x03, 1);
            printf("Received flight computer signal, send ack back.\n");
            
        } else { /* nothing sent from FC during this time out period, just continue waiting. */
            printf("Nothing from the flight computer, wait again.\n");
        }
    }

    return (EXIT_SUCCESS);
}

int readFC(int fd, char *data, int size) {
    //printf("fd readRoe: %d\n", fd);
    //Times out after 50000 tries
    //int timeout;
    int input;
    //wait for one second
    input = input_timeout_roe(fd, 1);
    if (input > 0) {
        if (read(fd, data, size) != -1) {
            //printf("data read, exiting readRoe %c<---data\n", *data);
            return 0;
        }
    }
    //printf("readRoe Error\n");
    return -1;
}

//Receive Acknowledgement from ROE

int receiveCmd(int fd, char *data, int size, char target) {
    int timeout;
    char msg[100];
    printf("Inside receiveAck\n");

    for (timeout = 0; timeout < 5; timeout++) //Times out after 5 seconds
    {
        if (readFC(fd, data, size) != -1) { //Return only if read data is an acknowledgement
            printf(msg, "Data:%c, target:%c\n", *data, target);
            if (*data == target) {
                printf("Acknowledgment successful\n");
                return 0;
            }
        }
    }
    printf(msg, "Aknowledgment timeout %c<--data\n", *data);
    return -1;
}

int atoh_roe(char c) {
    return (c >= 0 && c <= 9) ? (c & 0x0F) : ((c & 0x0F) + 9);
}

int input_timeout_roe(int filedes, unsigned int seconds) {
    fd_set set;
    struct timeval timeout;

    /*initialize the file descriptor set for select function*/
    FD_ZERO(&set);
    FD_SET(filedes, &set);

    /*initialize timout data structure for select function*/
    timeout.tv_sec = seconds;
    timeout.tv_usec = 0;

    /*select returns 0 if timeout, 1 if input data is available, -1 if error*/
    //return TEMP_FAILURE_RETRY(select(FD_SETSIZE, &set, NULL, NULL, &timeout));
    return select(FD_SETSIZE, &set, NULL, NULL, &timeout);
}

int activateConnection() {
    int fd;
    //Open Serial Device
    fd = open(ROE_DEV, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        printf("%s\n", strerror(errno));
        printf("Couldnt connect\n");
        exit(-1);
    }
    fcntl(fd, F_SETFL, FNDELAY);
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tcsetattr(fd, TCSANOW, &options);

    printf("Connection established. DEFAULT MODE\n");

printf("ROE Active\n");
return fd;

}
