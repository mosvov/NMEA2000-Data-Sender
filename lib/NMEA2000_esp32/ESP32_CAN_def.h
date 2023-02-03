#ifndef __DRIVERS_CAN_REGDEF_H_
#define __DRIVERS_CAN_REGDEF_H_

#ifdef __cplusplus
extern "C"
{
#endif

	typedef enum
	{
		CAN_SPEED_100KBPS = 100,  /**< \brief CAN Node runs at 100kBit/s. */
		CAN_SPEED_125KBPS = 125,  /**< \brief CAN Node runs at 125kBit/s. */
		CAN_SPEED_250KBPS = 250,  /**< \brief CAN Node runs at 250kBit/s. */
		CAN_SPEED_500KBPS = 500,  /**< \brief CAN Node runs at 500kBit/s. */
		CAN_SPEED_800KBPS = 800,  /**< \brief CAN Node runs at 800kBit/s. */
		CAN_SPEED_1000KBPS = 1000 /**< \brief CAN Node runs at 1000kBit/s. */
	} CAN_speed_t;

#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_CAN_REGDEF_H_ */
