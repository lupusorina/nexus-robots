
#if 0
#ifndef	_PID_TABLE_H_
#define	_PID_TABLE_H_

#define	PB				6
#define	PM				5
#define	PS				4
#define	ZO				3
#define	NS				2
#define	NM				1
#define	NB				0

#define	DELTA_KP_IDX	0
#define	DELATA_KI_IDX	1
#define	ERR				7
#define	DELTA_ERR		7


const char pid_tbl[2][7][7]={
						{
							{PB,PB,PM,PM,PS,ZO,ZO},
							{PB,PB,PM,PS,PS,ZO,ZO},
							{PM,PM,PM,PS,ZO,NS,NS},
							{PM,PM,PS,ZO,NS,NM,NM},
							{PS,PS,ZO,NS,NS,NM,NM},
							{PS,ZO,NS,NM,NM,NM,NB},
							{ZO,ZO,NM,NM,NM,NB,NB},
						},
						
						{
							{NB,NB,NM,NM,NS,ZO,ZO},
							{NB,NB,NM,NS,NS,ZO,ZO},
							{NB,NM,NS,NS,ZO,PS,PS},
							{NM,NM,NS,ZO,PS,PM,PM},
							{NM,NS,ZO,PS,PS,PM,PB},
							{ZO,ZO,PS,PS,PM,PB,PB},
							{ZO,ZO,PS,PM,PM,PB,PB},
						},
					 };

const float kp_tbl[ERR] = {
						0.07987,
						0.08974,
						0.09948,
						0.12922,
						0.54896,
						0.70000,
						0.76000,
				    };


const float ki_tbl[DELTA_ERR] = {
							0.02451,
							0.07353,
							0.14706,
							0.19608,
							0.24510,
							0.30000,
							0.00000,
						   };
 
#endif	_PID_TABLE_H_

#endif

