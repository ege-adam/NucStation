/*
 * Button.h
 *
 *  Created on: May 14, 2022
 *      Author: egemen
 */

#ifndef GamepadButton_H
#define GamepadButton_H


#include "stm32l0xx_hal.h"

class GamepadButton {
private:
	GPIO_TypeDef* but_port;
	uint16_t but_pin;

	bool lastStatus;
public:
	bool IsPressing;
	//Delegate<void, Button> OnDown;
	//Delegate<void, Button> OnUp;

	GamepadButton(GPIO_TypeDef* _PORT, uint16_t _PIN);
	void UpdateButton();
};


#endif /* SRC_BUTTON_H_ */
