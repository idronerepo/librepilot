/** @file */

/*
 *  com_manager.h
 *
 *  Created on: Aug 22, 2013
 *      Author: Walt Johnson
 */
#ifndef COM_MANAGER_H
#define COM_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif
    
#include <stdint.h>
#include "data_sets.h"
#include "linked_list.h"

/*! Defines the 4 parts to the communications version. Major changes involve changes to the com manager. Minor changes involve additions to data structures */
#define COMM_VERSION_CHAR0      1   // Major (in com_manager.h)
#define COMM_VERSION_CHAR1      1
// #define COMM_VERSION_CHAR2      0
// #define COMM_VERSION_CHAR3      0   // Minor (in data_sets.h)

// if a Windows DLL is needed, this can be redefined using the commented out code below
#define SHAREDLIB_EXPORT

/*
#if defined(COMMANAGER_LIBRARY)
#  define SHAREDLIB_EXPORT __declspec(dllexport)
#elif defined(COMMANAGER_NO_DLL)
#  define SHAREDLIB_EXPORT
#else
#  define SHAREDLIB_EXPORT __declspec(dllimport)
#endif
*/

#ifndef max 

#define max(a,b) (((a) > (b)) ? (a) : (b))

#endif

#ifndef min

#define min(a,b) (((a) < (b)) ? (a) : (b))

#endif

#ifndef LIMIT2

#define LIMIT2(x, xmin, xmax) { if ((x) < (xmin)) { (x) = (xmin); } else if ((x) > (xmax)) { (x) = (xmax); } }

#endif

/*! The maximum allowable dataset size */
#define MAX_DATASET_SIZE        1024

/*! The decoded overhead involved in sending a packet - 4 bytes for header, 4 bytes for footer */
#define PKT_OVERHEAD_SIZE       8       // = START_BYTE + INFO_BYTE + COUNTER_BYTE + FLAGS_BYTE + RESERVED_BYTE + CHECKSUM_BYTE_1 + CHECKSUM_BYTE_2 + END_BYTE

/*! The maximum buffer space that is used for sending and receiving packets */
#define PKT_BUF_SIZE            512

/*! The maximum encoded overhead size in sending a packet (7 bytes for header, 7 bytes for footer). The packet start and end bytes are never encoded. */
#define MAX_PKT_OVERHEAD_SIZE   (PKT_OVERHEAD_SIZE + PKT_OVERHEAD_SIZE - 2)  // worst case for packet encoding header / footer

/*! The maximum size of an decoded packet body */
#define MAX_PKT_BODY_SIZE       (((PKT_BUF_SIZE - MAX_PKT_OVERHEAD_SIZE) / 2) & 0xFFFFFFFE) // worst case for packet encoding body, rounded down to even number

/*! The maximum size of decoded data in a packet body */
#define MAX_P_DATA_BODY_SIZE    (MAX_PKT_BODY_SIZE-sizeof(p_data_hdr_t))    // Data size limit

/*! The maximum size of a decoded ACK message */
#define MAX_P_ACK_BODY_SIZE     (MAX_PKT_BODY_SIZE-sizeof(p_ack_hdr_t))     // Ack data size

/*
Packet Overview

Byte
0			Packet start byte
1			Packet indo: ID (mask 0x1F) | reserved bits (mask 0xE)
2			Packet counter (for ACK and retry)
3			Packet flags

// packet body, may or may not exist depending on packet id - packet body is made up of 4 byte or 8 byte values.
4-7			Data identifier
8-11		Data length
12-15		Data offset
16-19		Data start
(n-8)-(n-5)	Last piece of data
// end data

n-4			Reserved
n-3			Checksum high byte
n-2			Checksum low byte
n-1			Packet end byte
*/
typedef enum
{
	/*! Invalid packet id */
    PID_INVALID = 0,

	/*! ACK */
    PID_ACK,

	/*! NACK */
    PID_NACK,

	/*! Request for data to be broadcast, response is PID_DATA. See data structures for list of possible broadcast data. */
    PID_GET_DATA,

	/*! Data received from PID_GET_DATA, no ACK is sent back */
    PID_DATA,

	/*! Set data on the device, such as configuration options, sends an ACK back */
    PID_SET_DATA,

	/*! Stop all data broadcasts, sends an ACK back */
    PID_STOP_ALL_BROADCASTS,

	/*! Stop a specific broadcast */
    PID_STOP_DID_BROADCAST,

	/*! The number of packet identifiers, keep this at the end! */
    PID_COUNT,

	/*! The maximum count of packet identifiers, 0x1F (PACKET_INFO_ID_MASK) */
    PID_MAX_COUNT = 32
} pkt_info_byte_t;

/*! Represents size number of bytes in memory, up to a maximum of PKT_BUF_SIZE */
typedef struct
{
	/*! Number of bytes - for partial data requests, this will be less than the size of the data structure */
    uint32_t            size;

	/*! Buffer to hold the bytes */
    uint8_t             buf[PKT_BUF_SIZE];
} buffer_t;

/*! Represents size number of bytes in memory, pointing to a BYTE pointer that is owned elsewhere */
typedef struct
{
	/*! External bytes owned elsewhere */
	uint8_t             *ptr;

	/*! Number of bytes in ptr */
	uint32_t            size;
} bufPtr_t;

/*! Represents both a send and receive buffer */
typedef struct
{
	/*! send buffer */
    uint8_t             *txPtr;

	/*! receive buffer */
    uint8_t             *rxPtr;

	/*! size of both buffers */
    uint32_t            size;
} bufTxRxPtr_t;

/*! We must not allow any packing or shifting as these data structures must match exactly in memory on all devices */
#if defined(AVR) || defined(ARM)
#pragma pack(1)
#else
#pragma pack(push, 1)
#endif

/*! Types of values allowed in ASCII data */
typedef enum
{
	/*! 32 bit integer */
	asciiTypeInt = 0,

	/*! 32 bit unsigned integer */
	asciiTypeUInt = 1,

	/*! 32 bit floating point */
	asciiTypeFloat = 2,

	/*! 64 bit floating point */
	asciiTypeDouble = 3
} asciiDataType;

/*! Represents an ASCII message and how it is mapped to a structure in memory */
typedef struct
{
	/*! the message, always 4 characters long */
	unsigned char messageId[4];
	
	/*! the ptr of the start of the struct to modify */
	uint8_t* ptr;

	/*! the total size of the structure that ptr points to */
	int ptrSize;
	
	/*! field count - the number of items in fieldsAndOffsets */
	int fieldCount;
		
	/*! an array of 1 byte asciiDataType and 1 byte offset (shifted << 8) */
	uint16_t* fieldsAndOffsets;
} asciiMessageMap_t;

/*! create a uint from an ASCII message id that is the same, regardless of CPU architecture */
#define ASCII_MESSAGEID_TO_UINT(c4) ((uint32_t)c4[0] << 24 | ((uint32_t)c4[1] << 16) | ((uint32_t)c4[2] << 8) | ((uint32_t)c4[3]))

/*! Represents the 4 bytes that begin each binary packet */
typedef struct
{
	/*! Packet start byte, always 0xFF */
    uint8_t             startByte;

	/*! Packet identifier */
	uint8_t             pid;

	/*! Packet counter, for ACK and retry */
	uint8_t             counter;

	/*!
	Packet flags
	Bit 0 : unset means big endian, set means little endian format
	Bit 1 : unset means no valid communication received yet, set means valid communication received
	Bit 2 : unset means no more related packets available, set means additional packet(s) available related to this packet
	Bit 3 : unset means perform swap, set means do not swap
	*/
 	uint8_t             flags;
} packet_hdr_t;

enum ePktHdrFlags
{
	CM_PKT_FLAGS_LITTLE_ENDIAN			= 0x01,
	CM_PKT_FLAGS_ENDIANNESS_MASK		= 0x01,
	CM_PKT_FLAGS_RX_VALID_DATA			= 0x02,
	CM_PKT_FLAGS_MORE_DATA_AVAILABLE	= 0x04,
	CM_PKT_FLAGS_RAW_DATA_NO_SWAP		= 0x08,		// Allow for arbitrary length in bytes of data, not necessarily multiple of 4.  Don't swap bytes for endianness 
};


/*! Represents the 4 bytes that end each binary packet */
typedef struct
{
	/*! Reserved for future use */
 	uint8_t             reserved;

	/*! Checksum high byte */
    uint8_t             cksum1;

	/*! Checksum low byte */
    uint8_t             cksum2;

	/*! Packet end byte, always 0xFE */
    uint8_t             stopByte;
} packet_ftr_t;

/*! Represents a packet header and body */
typedef struct
{
	/*! Packet header */
    packet_hdr_t        hdr;

	/*! Packet body */
    bufPtr_t            body;
} packet_t;

/*! Represents a packet header, packet body and a buffer with data to send */
typedef struct
{
    packet_hdr_t        hdr;                    // Packet header
	bufPtr_t            bodyHdr;                // Body header
    bufPtr_t            txData;                 // Pointer and size of data to send
} pkt_info_t;

/*! Specifies the data id, size and offset of a PID_DATA and PID_DATA_SET packet */
typedef struct  
{
	/*! Data identifier */
    uint32_t            id;

	/*! Size of data, for partial requests this will be less than the size of the data structure */
    uint32_t            size;

	/*! Offset into data structure */
    uint32_t            offset;
} p_data_hdr_t;

/*! Represents the complete packet body of a PID_DATA and PID_DATA_SET packet */
typedef struct
{
	/*! Header with id, size and offset */
    p_data_hdr_t        hdr;

	/*! Data */
	uint8_t             buf[MAX_DATASET_SIZE];
} p_data_t, p_data_set_t;

/*! Represents the complete body of a PID_DATA_GET packet */
typedef struct
{
	/*! ID of data being requested */
    uint32_t            id;

	/*! Byte length of data from offset */
    uint32_t            size;

	/*! Byte offset into data */
    uint32_t            offset;

	/*!
	The broadcast period in milliseconds, or 0 for a one-time broadcast. Depending on data size and baud/transfer rates,
	some data may be dropped if this period is too short.
	*/
    uint32_t            bc_period_ms;
} p_data_get_t;

/*! Represents the body of a disable broadcast for data id packet */
typedef struct
{
	/*! The packet identifier to disable broadcasts for */
    uint32_t            id;
} p_data_disable_t;

/*! Represents the body header of an ACK or NACK packet */
typedef struct
{
	/*! Packet info of the received packet */
    uint32_t            pktInfo;

	/*! Packet counter of the received packet */
    uint32_t            pktCounter;
} p_ack_hdr_t;

/*! Represents the entire body of an ACK or NACK packet */
typedef struct
{
	/*! Body header */
    p_ack_hdr_t         hdr;

	/*! Body buffer */
    uint8_t             buf[MAX_P_ACK_BODY_SIZE];
} p_ack_t, p_nack_t;

/*! Pop off the packing argument, we can safely allow packing and shifting in memory at this point */
#if defined(AVR) || defined(ARM)
#pragma pack(4)
#else
#pragma pack(pop)
#endif

/*!
All possible special bytes that will need to be encoded in the binary packet format.
A byte is encoded by writing a 0xFD byte (encoded byte marker), followed by the encoded byte, which is created by inverting all the bits of the original byte.
These bytes are not encoded when written in the proper spot in the packet (i.e. when writing the first byte for a binary packet, the 0xFF byte, no encoding
is performed).
*/
typedef enum
{
	/*! Dollar sign ($), used by ASCII protocol to signify start of message */
	PSC_ASCII_START_BYTE	= 0x24,

	/*! New line (\n), used by ASCII protocol to signify end of message */
	PSC_ASCII_END_BYTE		= 0x0A,

	/*! Binary packet start byte, must only exist at the very start of a binary packet and no where else */
    PSC_START_BYTE			= 0xFF,

	/*! Binary packet end byte, must only exist at the end of a binary packet and no where else */
    PSC_END_BYTE			= 0xFE,

	/*! Encoded byte marker, must only be used to prefix encoded bytes */
    PSC_RESERVED_KEY		= 0xFD
}pkt_special_chars_t;

/*! Contains status for the com manager */
typedef struct  
{
	/*! Contains the port with the most recent error */
	int						rxErrorPHandle;

	/*! 0 if no errors encounteted, otherwise non-zero */
	int						rxError;

	/*! Total read count */
	int						rxCount;

	/*! Total write count */
	int						txCount;
} com_manager_status_t;

// com manager instance / handle is a void*
typedef void* CMHANDLE;

// com manager callback prototypes
// readFnc read data from the serial port represented by pHandle - return number of bytes read
typedef int(*pfnComManagerRead)(CMHANDLE cmHandle, int pHandle, unsigned char* readIntoBytes, int numberOfBytes);

// sendFnc send data to the serial port represented by pHandle - return number of bytes written
typedef int(*pfnComManagerSend)(CMHANDLE cmHandle, int pHandle, buffer_t* bufferToSend);

// txFreeFnc optional, return the number of free bytes in the send buffer for the serial port represented by pHandle
typedef int(*pfnComManagerSendBufferAvailableBytes)(CMHANDLE cmHandle, int pHandle);

// pstRxFnc optional, called after data is sent to the serial port represented by pHandle
typedef void(*pfnComManagerPostRead)(CMHANDLE cmHandle, int pHandle, p_data_t* dataRead);

// pstAckFnc optional, called after an ACK is received by the serial port represented by pHandle
typedef void(*pfnComManagerPostAck)(CMHANDLE cmHandle, int pHandle, p_ack_t* ack, unsigned char packetIdentifier);

// disableBcastFnc optional, mostly for internal use, this can be left as 0 or NULL
typedef void(*pfnComManagerDisableBroadcasts)(CMHANDLE cmHandle, int pHandle);

// called right before data is sent
typedef void(*pfnComManagerPreSend)(CMHANDLE cmHandle, int pHandle);

// ASCII message handler function
typedef int(*pfnComManagerAsciiMessageHandler)(CMHANDLE cmHandle, unsigned char* messageId, unsigned char* line, int lineLength);

// get the global instance of the com manager - this is only needed if you are working with multiple com managers and need to compare instances
CMHANDLE getGlobalComManager(void);

/*!
Initializes the default global com manager. This is generally only called once on program start.
The global com manager is used by the functions that do not have the Instance suffix and first parameter of void* cmInstance.
The instance functions can be ignored, unless you have a reason to have two com managers in the same process.

@param numHandles the max number of serial ports possible
@param maxEnsuredPackets the max number of ensured packets
@param stepPeriodMilliseconds how many milliseconds you are waiting in between calls to stepComManager
@param retryCount the number of times to retry failed packets
@param readFnc read data from the serial port represented by pHandle
@param sendFnc send data to the serial port represented by pHandle
@param txFreeFnc optional, return the number of free bytes in the send buffer for the serial port represented by pHandle
@param pstRxFnc optional, called after data is sent to the serial port represented by pHandle
@param pstAckFnc optional, called after an ACK is received by the serial port represented by pHandle
@param disableBcastFnc mostly for internal use, this can be left as 0 or NULL

Example:
@code
initComManager(20, 20, 10, 25, staticReadPacket, staticSendPacket, NULL, staticProcessRxData, staticProcessAck, 0);
@endcode
*/
SHAREDLIB_EXPORT void initComManager
(
	int numHandles,
	int maxEnsuredPackets,
	int stepPeriodMilliseconds,
	int retryCount,
	pfnComManagerRead readFnc,
	pfnComManagerSend sendFnc,
	pfnComManagerSendBufferAvailableBytes txFreeFnc,
	pfnComManagerPostRead pstRxFnc,
	pfnComManagerPostAck pstAckFnc,
	pfnComManagerDisableBroadcasts disableBcastFnc
);

// Initialize and return an instance to a com manager that can be passed to instance functions and can later be freed with freeComManagerInstance
// this function may be called multiple times
SHAREDLIB_EXPORT CMHANDLE initComManagerInstance
(
	int numHandles,
	int maxEnsuredPackets,
	int stepPeriodMilliseconds,
	int retryCount,
	pfnComManagerRead readFnc,
	pfnComManagerSend sendFnc,
	pfnComManagerSendBufferAvailableBytes txFreeFnc,
	pfnComManagerPostRead pstRxFnc,
	pfnComManagerPostAck pstAckFnc,
	pfnComManagerDisableBroadcasts disableBcastFnc
);

/*!
Performs one round of sending and receiving message. This should be called as often as you want to send and receive data.
*/
SHAREDLIB_EXPORT void stepComManager(void);
SHAREDLIB_EXPORT void stepComManagerInstance(CMHANDLE);

/*!
Get the most recent status of the com manager

@return com manager status, this pointer is owned by the com manager
*/
SHAREDLIB_EXPORT com_manager_status_t* getStatusComManager(void);
com_manager_status_t* getStatusComManagerInstance(CMHANDLE cmInstance);

/*!
Make a request to a port handle to broadcast a piece of data at a set interval

@param pHandle the port handle to request broadcast data from
@param dataId the data id to broadcast
@param offset offset into the structure for the data id to broadcast - pass offset and size of 0 to receive the entire data set
@param size number of bytes in the data structure from offset to broadcast - pass offset and size of 0 to receive the entire data set
@param period_ms the interval in milliseconds in which to broadcast data

Example that makes a request to receive the device info just once:
@code
getDataComManager(0, DID_DEV_INFO, 0, sizeof(dev_info_t), 0);
@endcode

Example that broadcasts INS data every 50 milliseconds:
@code
getDataComManager(0, DID_INS_1, 0, sizeof(ins_1_t), 50);
@endcode
*/
SHAREDLIB_EXPORT void getDataComManager(int pHandle, uint32_t dataId, int offset, int size, int period_ms);
SHAREDLIB_EXPORT void getDataComManagerInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId, int offset, int size, int period_ms);

/*!
Disable a broadcast for a specified port handle and data identifier

@param pHandle the port handle to disable a broadcast for
@param dataId the data id to disable boradcast for
@return 0 if success, anything else if failure

Example:
@code
disableDataComManager(0, DID_INS_1);
@endcode
*/
SHAREDLIB_EXPORT int disableDataComManager(int pHandle, uint32_t dataId);
SHAREDLIB_EXPORT int disableDataComManagerInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId);

/*!
Send a packet to a port handle

@param pHandle the port handle to send the packet to
@param pktInfo the type of packet (PID)
@param bodyHdr optional, can contain information about the actual data of the body (txData), usually the data id, offset and size
@param txData optional, the actual body of the packet
@return 0 if success, anything else if failure

Example:
@code
p_data_get_t request;
bufPtr_t data;
request.id = DID_INS_1;
request.offset = 0;
request.size = sizeof(ins_1_t);
request.bc_period_ms = 50;
data.ptr = (uint8_t*)&request;
data.size = sizeof(request);
sendComManager(pHandle, PID_GET_DATA, 0, &data)
@endcode
*/
SHAREDLIB_EXPORT int sendComManager(int pHandle, pkt_info_byte_t pktInfo, bufPtr_t* bodyHdr, bufPtr_t* txData, uint8_t pktFlags);
SHAREDLIB_EXPORT int sendComManagerInstance(CMHANDLE cmInstance, int pHandle, pkt_info_byte_t pktInfo, bufPtr_t* bodyHdr, bufPtr_t* txData, uint8_t pktFlags);

/*!
Convenience function that wraps sendComManager for sending data structures.  Must be multiple of 4 bytes in size.

@param pHandle the port handle to send data to
@param dataId the data id of the data to send
@param dataPtr pointer to the data structure to send
@param dataSize number of bytes to send
@param dataOffset offset into dataPtr to send at
@return 0 if success, anything else if failure

Example:
@code
sendDataComManager(0, DID_DEV_INFO, &g_devInfo, sizeof(dev_info_t), 0);
@endcode
*/
SHAREDLIB_EXPORT int sendDataComManager(int pHandle, uint32_t dataId, void* dataPtr, int dataSize, int dataOffset);
SHAREDLIB_EXPORT int sendDataComManagerInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId, void* dataPtr, int dataSize, int dataOffset);

/*!
Same as sendComManager, except that the com manager may retry the send if an ACK is not received

@param pHandle the port handle to send the packet to
@param pktInfo the type of packet (PID)
@param bodyHdr optional, can contain information about the actual data of the body (txData), usually the data id, offset and size
@param txData optional, the actual body of the packet
@return 0 if success, anything else if failure
*/
SHAREDLIB_EXPORT int sendEnsuredComManager(int pHandle, pkt_info_byte_t pktInfo, unsigned char *data, unsigned int dataSize);
SHAREDLIB_EXPORT int sendEnsuredComManagerInstance(CMHANDLE cmInstance, int pHandle, pkt_info_byte_t pktInfo, unsigned char *data, unsigned int dataSize);

// INTERNAL FUNCTIONS...
/*!
Same as sendComManager, except that no retry is attempted

@param pHandle the port handle to send the packet to
@param dataId Data structure ID number.
@param dataPtr Pointer to actual data.
@param dataSize Size of data to send in number of bytes.
@param dataOffset Offset into data structure where copied data starts.
@param pFlags Additional packet flags if needed.
@return 0 if success, anything else if failure
*/
SHAREDLIB_EXPORT int sendDataComManagerNoAck(int pHandle, uint32_t dataId, void* dataPtr, int dataSize, int dataOffset);
SHAREDLIB_EXPORT int sendDataComManagerNoAckInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId, void* dataPtr, int dataSize, int dataOffset);

/*!
Convenience function that wraps sendComManager for sending data structures.  Allows arbitrary bytes size, 4 byte multiple not required. 
No byte swapping occurs.

@param pHandle the port handle to send data to
@param dataId the data id of the data to send
@param dataPtr pointer to the data structure to send
@param dataSize number of bytes to send
@param dataOffset offset into dataPtr to send at
@return 0 if success, anything else if failure

Example:
@code
sendDataComManager(0, DID_DEV_INFO, &g_devInfo, sizeof(dev_info_t), 0);
@endcode
*/
SHAREDLIB_EXPORT int sendRawDataComManager(int pHandle, uint32_t dataId, void* dataPtr, int dataSize, int dataOffset);
SHAREDLIB_EXPORT int sendRawDataComManagerInstance(CMHANDLE cmInstance, int pHandle, uint32_t dataId, void* dataPtr, int dataSize, int dataOffset);

/*!
Internal use function that disables broadcasts of all messages
*/
SHAREDLIB_EXPORT void disableAllBroadcasts(void);
SHAREDLIB_EXPORT void disableAllBroadcastsInstance(CMHANDLE cmInstance);

/*!
Checks if any valid data has been received by the com manager

@return non-zero if valid data has been received
*/
SHAREDLIB_EXPORT unsigned char comManagerHasReceivedValidData(void);
SHAREDLIB_EXPORT unsigned char comManagerHasReceivedValidDataInstance(CMHANDLE cmInstance);

/*!
Register message handling function for a received data id (binary). This is mostly an internal use function,
but can be used if you are implementing your own receiver on a device.

@param dataId the data id to register the handler for
@param txFnc called right before the data is sent
@param pstRxFnc called after data is received for the data id
@param txDataPtr a pointer to the structure in memory of the data to send
@param rxDataPtr a pointer to the structure in memory to copy received data to
@param dataSize size of the data structure in txDataPtr and rxDataPtr

Example:
@code
registerComManager(DID_INS_1, prepMsgINS, writeMsgINS, &g_insData, &g_insData, sizeof(ins_1_t));
@endcode
*/
SHAREDLIB_EXPORT void registerComManager(uint32_t dataId, pfnComManagerPreSend txFnc, pfnComManagerPostRead pstRxFnc, void* txDataPtr, void* rxDataPtr, int dataSize);
SHAREDLIB_EXPORT void registerComManagerInstance(CMHANDLE cmInstance, uint32_t dataId, pfnComManagerPreSend txFnc, pfnComManagerPostRead pstRxFnc, void* txDataPtr, void* rxDataPtr, int dataSize);

/*!
Register handlers for when ASCII messages are received. This is also an internal function,
but you can implement it if you are writing your own receiver on a device.

@param asciiMessages the ASCII messages to handle, ownership is assumed to be elsewhere (usually this will be a static variable)
@param asciiMessagesCount the number of items in asciiMessages
@param msgFunc handler for messages that were not in the asciiMessages array
*/
SHAREDLIB_EXPORT void registerComManagerASCII(asciiMessageMap_t* asciiMessages, int asciiMessagesCount, pfnComManagerAsciiMessageHandler msgFnc);
SHAREDLIB_EXPORT void registerComManagerASCIIInstance(CMHANDLE cmInstance, asciiMessageMap_t* asciiMessages, int asciiMessagesCount, pfnComManagerAsciiMessageHandler msgFnc);

/*!
Send all pending messages.
Normally you don't need to call this as it is called in stepComManager.
*/
SHAREDLIB_EXPORT void stepComManagerSendMessages(void);
SHAREDLIB_EXPORT void stepComManagerSendMessagesInstance(CMHANDLE cmInstance);

/*!
Free a com manager instance. This instance should never be used again. The default global com manager can never be freed.
*/
SHAREDLIB_EXPORT void freeComManagerInstance(CMHANDLE cmInstance);

/*!
Attach user defined data to a com manager instance
*/
SHAREDLIB_EXPORT void comManagerAssignUserPointer(CMHANDLE cmInstance, void* userPointer);

/*!
Get user defined data to from a com manager instance
*/
SHAREDLIB_EXPORT void* comManagerGetUserPointer(CMHANDLE cmInstance);

/*! Copies packet data into a data structure.  Returns 0 on success, -1 on failure. */
char copyDataPToStructP(void *sptr, p_data_t *data, unsigned int maxsize);

/*! Copies packet data into a data structure.  Returns 0 on success, -1 on failure. */
char copyDataPToStructP2(void *sptr, p_data_hdr_t *dataHdr, uint8_t *dataBuf, unsigned int maxsize);

#ifdef __cplusplus
}
#endif

#endif // COM_MANAGER_H
