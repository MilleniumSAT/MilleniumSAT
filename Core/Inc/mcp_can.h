/*
  mcp_can.h

  The MIT License (MIT)

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/
#ifndef _MCP2515_H_
#define _MCP2515_H_

#include "mcp_can_dfs.h"

#define MAX_CHAR_IN_MESSAGE 8



/*
*  mcp2515 driver function
*/


void mcp2515_reset(void);                                   // reset mcp2515

uint8_t mcp2515_readRegister(const uint8_t address);              // read mcp2515's register

void mcp2515_readRegisterS(const uint8_t address,
					   uint8_t values[],
						   const uint8_t n);
void mcp2515_setRegister(const uint8_t address,                // set mcp2515's register
						 const uint8_t value);

void mcp2515_setRegisterS(const uint8_t address,               // set mcp2515's registers
						  const uint8_t values[],
						  const uint8_t n);

void mcp2515_initCANBuffers(void);

void mcp2515_modifyRegister(const uint8_t address,             // set bit of one register
							const uint8_t mask,
							const uint8_t data);

uint8_t mcp2515_readStatus(void);                              // read mcp2515's Status
uint8_t mcp2515_setCANCTRL_Mode(const uint8_t newmode);           // set mode
uint8_t mcp2515_configRate(const uint8_t canSpeed);               // set boadrate
uint8_t mcp2515_init(const uint8_t canSpeed);                     // mcp2515init

void mcp2515_write_id( const uint8_t mcp_addr,                 // write can id
						   const uint8_t ext,
						   const unsigned long id );

void mcp2515_read_id( const uint8_t mcp_addr,                  // read can id
								uint8_t* ext,
								unsigned long* id );

void mcp2515_write_canMsg( const uint8_t buffer_sidh_addr, int rtrBit );   // write can msg
void mcp2515_read_canMsg( const uint8_t buffer_sidh_addr);     // read can msg
void mcp2515_start_transmit(const uint8_t mcp_addr);           // start transmit
uint8_t mcp2515_getNextFreeTXBuf(uint8_t *txbuf_n);               // get Next free txbuf

/*
*  can operator function
*/

uint8_t setMsgExt(unsigned long id, uint8_t ext, uint8_t len, uint8_t rtr, uint8_t *pData);   // set message
uint8_t setMsg(unsigned long id, uint8_t ext, uint8_t len, uint8_t *pData);             //  set message
uint8_t clearMsg();                                                // clear all message to zero
uint8_t readMsg();                                                 // read message
uint8_t sendMsg(int rtrBit);                                                 // send message

MCP_CAN(uint8_t _CS);
uint8_t begin(uint8_t speedset);                                      // init can
uint8_t init_Mask(uint8_t num, uint8_t ext, unsigned long ulData);       // init Masks
uint8_t init_Filt(uint8_t num, uint8_t ext, unsigned long ulData);       // init filters
uint8_t sendMsgBufExt(unsigned long id, uint8_t ext, uint8_t rtr, uint8_t len, uint8_t *buf);     // send buf
uint8_t sendMsgBuf(unsigned long id, uint8_t ext, uint8_t len, uint8_t *buf);               // send buf
uint8_t readMsgBuf(uint8_t *len, uint8_t *buf);                          // read buf
uint8_t readMsgBufID(unsigned long *ID, uint8_t *len, uint8_t *buf);     // read buf with object ID
uint8_t checkReceive(void);                                        // if something received
uint8_t checkError(void);                                          // if something error
unsigned long getCanId(void);                                   // get can id when receive
uint8_t isRemoteRequest(void);                                     // get RR flag when receive
uint8_t isExtendedFrame(void);                                     // did we recieve 29bit frame?

#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
