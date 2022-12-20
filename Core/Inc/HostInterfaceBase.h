/*
 * HostInterfaceBase.h
 *
 *  Created on: 16 дек. 2022 г.
 *      Author: mak22
 */

#ifndef HOSTINTERFACEBASE_H_
#define HOSTINTERFACEBASE_H_

class HostInterfaceBase {
public:
	HostInterfaceBase();
	virtual ~HostInterfaceBase();

	virtual void messageCallback();
};

#endif /* HOSTINTERFACEBASE_H_ */
