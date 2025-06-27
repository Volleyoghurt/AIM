/*
 * linkdetect.h
 *
 *  Created on: Jun 27, 2025
 *      Author: basge
 */

#ifndef INC_LINKDETECT_H_
#define INC_LINKDETECT_H_


void CheckInitialLinkStatus(void);
void OnLinkError(void);
void OnLinkRecovered(void);
void CheckLinkTimeout(void);


#endif /* INC_LINKDETECT_H_ */
