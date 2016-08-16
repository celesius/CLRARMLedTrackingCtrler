/*
 * CLRI2CInterface.h
 *
 *  Created on: 2016年8月12日
 *      Author: clover
 */
#ifndef CLRI2CINTERFACE_H_
#define CLRI2CINTERFACE_H_

class CLRI2CInterface {
public:
	CLRI2CInterface();
	virtual ~CLRI2CInterface();
    void write(int address,char *data_write, int data_count, char stop_flag); // no stop
    bool read(int address, char *data_read, int data_count, char stop_flag);
private:

};

#endif /* CLRI2CINTERFACE_H_ */
