/*
    Author: Keenan Burnett

	***********************************************************************
	*	FILE NAME:		global_var.h
	*
	*	PURPOSE:		This file contains global variable definitions to be used across all files.
	*	FILE REFERENCES:		None
	*
	*	EXTERNAL VARIABLES:		None
	*
	*	EXTERNAL REFERENCES:	Same a File References.
	*
	*	ABORNOMAL TERMINATION CONDITIONS, ERROR AND WARNING MESSAGES: None yet.
	*
	*	ASSUMPTIONS, CONSTRAINTS, CONDITIONS:	None
	*
	*	NOTES:					All global variables and definitions should be put in this file.
	*
	*	REQUIREMENTS/ FUNCTIONAL SPECIFICATION REFERENCES:
	*
	*	DEVELOPMENT HISTORY:
	*	11/12/2015			Created.
	*
*/
#ifndef GLOBAL_VAR_H
#define GLOBAL_VAR_H

#define DATA_BUFFER_SIZE 8 // 8 bytes max

/*				MY CAN DEFINES								*/
#define SELF_ID					0 // Current SSM is EPS.

#define PACKET_LENGTH			152	// Length of the PUS packet.

#define COMMAND_OUT					0X01010101
#define COMMAND_IN					0x11111111

#define HK_TRANSMIT					0x12345678
#define CAN_MSG_DUMMY_DATA          0xFFFFFFFF

#define DUMMY_COMMAND				0XFFFFFFFF
#define MSG_ACK						0xABABABAB

#define HK_RETURNED					0XF0F0F0F0
#define HK_REQUEST					0x0F0F0F0F

#define DATA_REQUEST				0x55555555
#define DATA_RETURNED				0x00000055

#define MESSAGE_RETURNED			0X00000000

#define CAN0_MB0				1
#define CAN0_MB1				2
#define CAN0_MB2				3
#define CAN0_MB3				4
#define CAN0_MB4				5
#define CAN0_MB5				6
#define CAN0_MB6				7
#define CAN0_MB7				8

#define CAN1_MB0				10
#define CAN1_MB1				10
#define CAN1_MB2				10
#define CAN1_MB3				10
#define CAN1_MB4				14
#define CAN1_MB5				14
#define CAN1_MB6				14
#define CAN1_MB7				17

/* IDs for COMS/SUB0 mailboxes */
#define SUB0_ID0				20
#define SUB0_ID1				21
#define SUB0_ID2				22
#define SUB0_ID3				23
#define SUB0_ID4				24
#define SUB0_ID5				25

/* IDs for EPS/SUB1 mailboxes */
#define SUB1_ID0				26
#define SUB1_ID1				27
#define SUB1_ID2				28
#define SUB1_ID3				29
#define SUB1_ID4				30
#define SUB1_ID5				31

/* IDs for PAYLOAD/SUB2 mailboxes */
#define SUB2_ID0				32
#define SUB2_ID1				33
#define SUB2_ID2				34
#define SUB2_ID3				35
#define SUB2_ID4				36
#define SUB2_ID5				37

/* MessageType_ID  */
#define MT_DATA					0x00
#define MT_HK					0x01
#define MT_COM					0x02
#define MT_TC					0x03

/* SENDER_ID */
#define COMS_ID					0x00
#define EPS_ID					0x01
#define PAY_ID					0x02
#define OBC_ID					0x03
#define HK_TASK_ID				0x04
#define DATA_TASK_ID			0x05
#define TIME_TASK_ID			0x06
#define COMS_TASK_ID			0x07
#define EPS_TASK_ID				0x08
#define PAY_TASK_ID				0x09
#define OBC_PACKET_ROUTER_ID	0x0A
#define SCHEDULING_TASK_ID		0x0B
#define WD_RESET_TASK_ID		0x0D
#define MEMORY_TASK_ID			0x0E
#define HK_GROUND_ID			0x0F
#define TIME_GROUND_ID			0x10
#define MEM_GROUND_ID			0x11

/* COMMAND SMALL-TYPE: */
#define REQ_RESPONSE			0x01
#define REQ_DATA				0x02
#define REQ_HK					0x03
#define RESPONSE 				0x04
#define REQ_READ				0x05
#define ACK_READ				0x06
#define REQ_WRITE				0x07
#define ACK_WRITE				0x08
#define SET_SENSOR_HIGH			0x09
#define SET_SENSOR_LOW			0x0A
#define SET_VAR					0x0B
#define SET_TIME				0x0C
#define SEND_TM					0x0D
#define SEND_TC					0x0E
#define TM_PACKET_READY			0x0F
#define OK_START_TM_PACKET		0x10
#define TC_PACKET_READY			0x11
#define OK_START_TC_PACKET		0x12
#define TM_TRANSACTION_RESP		0x13
#define TC_TRANSACTION_RESP		0x14
#define SAFE_MODE_TYPE			0x15
#define SEND_EVENT				0x16

/* Checksum only */
#define SAFE_MODE_VAR			0x09

#define SMALLTYPE_DEFAULT		0x00

/* DATA SMALL-TYPE	   */
#define SPI_TEMP1				0x01
#define COMS_PACKET				0x02
#define BATT_TOP				0x03
#define BATT_BOTTOM				0x04

/* MESSAGE PRIORITIES	*/
#define COMMAND_PRIO			25
#define HK_REQUEST_PRIO			20
#define DATA_PRIO				10
#define DEF_PRIO				10

/* SENSOR NAMES			*/
#define PANELX_V				0x01
#define PANELX_I				0x02
#define PANELY_V				0x03
#define PANELY_I				0x04
#define BATTM_V					0x05
#define BATT_V					0x06
#define BATTIN_I				0x07
#define BATTOUT_I				0x08
#define BATT_TEMP				0x09
#define EPS_TEMP				0x0A
#define COMS_V					0x0B
#define COMS_I					0x0C
#define PAY_V					0x0D
#define PAY_I					0x0E
#define OBC_V					0x0F
#define OBC_I					0x10
#define BATT_I					0x11
#define COMS_TEMP				0x12
#define OBC_TEMP				0x13
#define PAY_TEMP0				0x14
#define PAY_TEMP1				0x15
#define PAY_TEMP2				0x16
#define PAY_TEMP3				0x17
#define PAY_TEMP4				0x18
#define PAY_HUM					0x19
#define PAY_PRESS				0x1A

/* VARIABLE NAMES		*/
#define MPPTX					0xFF
#define MPPTY					0xFE
#define COMS_MODE				0xFD
#define EPS_MODE				0xFC
#define PAY_MODE				0xFB
#define OBC_MODE				0xFA
#define PAY_STATE				0xF9
#define ABS_TIME_D				0xF8
#define ABS_TIME_H				0xF7
#define ABS_TIME_M				0xF6
#define ABS_TIME_S				0xF5
#define SPI_CHIP_1				0xF4
#define SPI_CHIP_2				0xF3
#define SPI_CHIP_3				0xF2
#define BALANCE_L				0xF1
#define BALANCE_H				0xF0

/* Global variables to be used for CAN communication */
uint8_t	status, mob_number, send_now, send_hk, send_data, set_sens_h, set_sens_l, set_varf;
uint8_t read_response, write_response;
uint8_t receive_arr[8], send_arr[8], read_arr[8], write_arr[8], data_req_arr[8];
uint8_t sensh_arr[8], sensl_arr[8], setv_arr[8];
uint8_t id_array[6];	// Necessary due to the different mailbox IDs for COMS, EPS, PAYL.

/* Global variables used for PUS packet communication */
uint8_t new_tm_msg[8], new_tc_msg[8], tm_sequence_count, new_tm_msgf, current_tm_fullf, tc_packet_readyf;
uint8_t tc_transfer_completef, start_tc_transferf, receiving_tmf;
uint8_t current_tm[PACKET_LENGTH], tm_to_downlink[PACKET_LENGTH], current_tc[PACKET_LENGTH];
uint8_t event_readyf;
uint8_t event_arr[8];

/* Global Variables for EPS		*/
uint16_t pxv_high, pxv_low, pxi_high, pxi_low, pyv_high, pyv_low, pyi_high, pyi_low;
uint16_t battmv_high, battmv_low, battv_high, battv_low, batti_high, batti_low, battemp_high;
uint16_t battemp_low, epstemp_high, epstemp_low, comsv_high, comsv_low, comsi_high, comsi_low;
uint16_t payv_high, payv_low, payi_high, payi_low, obcv_high, obcv_low, obci_high, obci_low;
uint8_t mpptx, mppty, balance_h, balance_l;

// Temporary Global Variables for testing
uint8_t pxv, pxi, pyv, pyi;

// Global variable used to store the current minute (updated by a CAN message from the OBC)
uint8_t CURRENT_MINUTE;

uint8_t data0[DATA_BUFFER_SIZE];	// Data Buffer for MOb0
uint8_t data1[DATA_BUFFER_SIZE];	// Data Buffer for MOb1
uint8_t data2[DATA_BUFFER_SIZE];	// Data Buffer for MOb2
uint8_t data3[DATA_BUFFER_SIZE];	// Data Buffer for MOb3
uint8_t data4[DATA_BUFFER_SIZE];	// Data Buffer for MOb4
uint8_t data5[DATA_BUFFER_SIZE];	// Data Buffer for MOb5
/*****************************************************/
#endif
