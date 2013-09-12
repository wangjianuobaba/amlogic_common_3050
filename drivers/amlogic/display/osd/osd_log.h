#ifndef  OSD_LOG_H
#define OSD_LOG_H

#define DEBUG
#ifdef  DEBUG
#define  AMLOG   1
#define LOG_LEVEL_VAR amlog_level_osd
#define LOG_MASK_VAR amlog_mask_osd
#endif


#define  	LOG_LEVEL_HIGH    		0x00f
#define	LOG_LEVEL_1			0x001
#define 	LOG_LEVEL_LOW			0x000

#define LOG_LEVEL_DESC \
"[0x00]LOW[0X01]LEVEL1[0xf]HIGH"	

#define  	LOG_MASK_INIT			0x001
#define	LOG_MASK_IOCTL		0x002
#define	LOG_MASK_HARDWARE	0x004
#define	LOG_MASK_PARA			0x008
#define 	LOG_MASK_DESC \
"[0x01]:INIT,[0x02]:IOCTL,[0x04]:HARDWARE,[0x08]PARA"

#endif
