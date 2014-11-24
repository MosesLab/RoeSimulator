/* 
 * File:   roesim.h
 * Author: david
 *
 * Created on November 21, 2014, 4:24 PM
 */

#ifndef ROESIM_H
#define	ROESIM_H

#define ROE_DEV         "/dev/ttyUSB0"

int readFC(int fd, char *data, int size);
int receiveCmd(int fd, char *data, int size, char target);
int activateConnection();

int connection;


#endif	/* ROESIM_H */

