/**
 *Name        : cst0xx_config.h
 *Author      : gary
 *Version     : V1.0
 *Create      : 2018-1-23
 *Copyright   : zxzz
 */


#ifndef CST0XX_CONFIG_H__
#define CST0XX_CONFIG_H__

/*
 * show debug log info
 * enable it for debug, disable it for release
 */
#define HYN_DEBUG_EN                            1

/*
 * Linux MultiTouch Protocol
 * 1: Protocol B(default), 0: Protocol A
 */
#define HYN_MT_PROTOCOL_B_EN                    1


/*
 * Report Pressure in multitouch
 * 1:enable(default),0:disable
*/
#define HYN_REPORT_PRESSURE_EN                  0


/*
 * Nodes for tools, please keep enable
 */
#define HYN_SYSFS_NODE_EN                       0
#define HYN_APK_NODE_EN                         0

#define HYN_EN_AUTO_UPDATE                        0
#define HYN_EN_AUTO_UPDATE_CST0xxSE               0
#define HYN_EN_AUTO_UPDATE_CST0xx                 0


#endif /*CST0XX_CONFIG_H__*/


