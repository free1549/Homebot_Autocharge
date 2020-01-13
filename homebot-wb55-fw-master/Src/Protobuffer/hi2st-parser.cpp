#include <uchar.h>
#include <inttypes.h>

#include "hi2st-parser.h"
extern "C" {
	extern void Error_Handler();
//#include "uart_trace.h"
}
#include "circular-buffer.h"
#include "uuid.h"
#include "p2p_server_app.h"
#include "p2p_client_app.h"
#include "hci_tl.h"
#include "ble_conf.h"

extern HCI_TL_CmdStatus_t BleHCICmdStatus;

/*
	 BLE to STM 
	 HEADER(2) + LEN(1) + DATA[1 ~20) + CRC(1)+ TAIL(1)
	 */

extern "C" {

	/*
	 * This allocator uses the system's malloc() and free(). It is the default
	 * allocator used if NULL is passed as the ProtobufCAllocator to an exported
	 * function.
	 */


	#define TO_ST_ALLOC_BUF_SIZE 1000
	#define TO_ST_ALLOC_BUF_CNT 5
	uint8_t to_st_alloc_buf[TO_ST_ALLOC_BUF_CNT][TO_ST_ALLOC_BUF_SIZE] = {0};

	// size, buf

	static void *
	to_st_alloc(void *allocator_data, size_t size)
	{
		uint8_t index = 0;
		if(size > TO_ST_ALLOC_BUF_SIZE )
		{
			Error_Handler();
		}

		for(index = 0; index < TO_ST_ALLOC_BUF_CNT; index++)
		{
			if(to_st_alloc_buf[index][TO_ST_ALLOC_BUF_SIZE - 1] == 0)
			{
				break;
			}
		}

		if(index == TO_ST_ALLOC_BUF_CNT) // buffer is full
		{
			Error_Handler();
		}

		to_st_alloc_buf[index][TO_ST_ALLOC_BUF_SIZE - 1] = 1;

		return (void *) to_st_alloc_buf[index];
	}

	static void
	to_st_free(void *allocator_data, void *data)
	{
		uint8_t index = 0;

		for(index = 0; index < TO_ST_ALLOC_BUF_CNT; index++)
		{
			if(to_st_alloc_buf[index] == data)
			{
				break;
			}
		}

		if(index == TO_ST_ALLOC_BUF_CNT) // memory address is not matched
		{
			Error_Handler();
		}

		to_st_alloc_buf[index][TO_ST_ALLOC_BUF_SIZE - 1] = 0;
	}

	ProtobufCAllocator protobuf_c_to_st_allocator = {
		.alloc = &to_st_alloc,
		.free = &to_st_free,
		.allocator_data = NULL,
	};

} // extern "C"


class ProtoParser {
	public:
		uint32_t requiredBytes = 1; 
		uint8_t crc;
		CircularBuffer cb;
        
        enum E_STEP {
				STEP_CB = 0, STEP_B7, STEP_CRC, STEP_LENGTH, STEP_PROTOBUF
			} ;
        E_STEP step = STEP_CB;
		// virtual void onMessage(uint8_t* buf) = 0;
		uint8_t addToCB(uint8_t t) { 
			cb.write((const char *)&t,1);
			return 0;
		}
		uint8_t parseProtobuf(ParseCallbackType p)
		{
			//uint8_t readbuffbyte; 
			
			
			if(requiredBytes <= cb.size()) {
				switch(step) {
					case STEP_CB: 
						{
							uint8_t one;
							cb.read(&one, sizeof(one));
							if(one == 0xCB) {
								step = STEP_B7;
							}
							else {
								step = STEP_CB;
							}
						}
						break;
					case STEP_B7:
						{
							uint8_t one;
							cb.read(&one, sizeof(one));
							if(one == 0xB7) {
								step = STEP_CRC;
							}
							else {
								step = STEP_CB;
							}
						}
						break;
					case STEP_CRC:
						{
							uint8_t one;
							cb.read(&one, sizeof(one));
							crc = one;
							step = STEP_LENGTH;
							requiredBytes = 4; 
						}
						break;
					case STEP_LENGTH:
						{
							uint32_t one;
							cb.read(&one, sizeof(one));
							requiredBytes = one; 
							step = STEP_PROTOBUF;
							if(one <= 0 || one >= 2000) {
								step = STEP_CB;
								requiredBytes = 1;
							}
						}
						break;
					case STEP_PROTOBUF:
						{
							//uint8_t* buf = new uint8_t[requiredBytes];
							uint8_t buf[1000] = {0};
							cb.read(buf, requiredBytes);
							uint8_t crcOfRecvData = 0;
							for(int i=0; i<requiredBytes; i++) {
								crcOfRecvData += buf[i];
							}
							if(crc != crcOfRecvData) {
							}
                            p(buf, requiredBytes);

							requiredBytes = 1;
							step = STEP_CB;
						}
						break;
				}
			} 

			return cb.size();
			//return 0;
		}
}; 

class BleParser : public ProtoParser {
	public:
		static void pushModernCommand(UART_HandleTypeDef* uart, const ToHi* d) {
			int serializedSize = to_hi__get_packed_size(d);
			int totalSize = 7 + serializedSize;
			//ModernProtocolWithSTM* pkt = (ModernProtocolWithSTM*)malloc(totalSize); // pktBuffer;
			ModernProtocolWithSTM pkt;
			if(serializedSize > sizeof(pkt.data))
			{
				Error_Handler();
			}
				
			pkt.sig1 = 0xCB;
			pkt.sig2 = 0xB7;

			pkt.crc = 0;
			pkt.length = serializedSize;
			to_hi__pack(d, pkt.data);
			for(int i=0; i<serializedSize; i++) {
				pkt.crc += pkt.data[i]; 
			}
			uint8_t* bp = (uint8_t*)&pkt; // totalSize
			
            //HAL_UART_Transmit_DMA(uart, bp, totalSize);
            HAL_UART_Transmit(uart, bp, totalSize, 1000);

		}
		static void pushModernCommand(const ToHost* d) {
			int serializedSize = to_host__get_packed_size(d);
			int totalSize = 7 + serializedSize;
			//ModernProtocolWithSTM* pkt = (ModernProtocolWithSTM*)malloc(totalSize); // pktBuffer;
			ModernProtocolWithSTM pkt;

			pkt.sig1 = 0xCB;
			pkt.sig2 = 0xB7;

			pkt.crc = 0;
			pkt.length = serializedSize;
			to_host__pack(d, pkt.data);
			for(int i=0; i<serializedSize; i++) {
				pkt.crc += pkt.data[i];
			}
			uint8_t* bp = (uint8_t*)&pkt; // totalSize
            while(totalSize) {
                int segmentSize = std::min({totalSize, 20}); 
                totalSize -= segmentSize; 
                Server_Update_Char(segmentSize, (uint8_t*)bp);  // 20, 20, 7
                bp += segmentSize;
            }
		}
};

BleParser bleParserFromHi;
BleParser bleParserFromAndroid;

extern "C" { 
    // StParser stParser;
    uint8_t addToBleParserFromHi(uint8_t t) { 
        return bleParserFromHi.addToCB(t);
    } 
    uint8_t parseBleFromHi(ParseCallbackType p)
    {
        return bleParserFromHi.parseProtobuf(p);
    }

    void pushToAndroid(const ToHost* d) {
        BleParser::pushModernCommand(d);
    }

    uint8_t addToBleParserFromAndroid(uint8_t t) { 
        return bleParserFromAndroid.addToCB(t);
    }

    uint8_t parseBleFromAndroid(ParseCallbackType p)
    {
        return bleParserFromAndroid.parseProtobuf(p);
    }

    void pushToHi(UART_HandleTypeDef* uart, const ToHi* d) {
        BleParser::pushModernCommand(uart, d);
    }
}





