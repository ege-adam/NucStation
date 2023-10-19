/*
 * Button.cpp
 *
 *  Created on: May 14, 2022
 *      Author: egemen
 */

#include <GamepadButton.hpp>


GamepadButton::GamepadButton(GPIO_TypeDef* _PORT, uint16_t _PIN) {
	but_pin = _PIN;
	but_port = _PORT;
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/*Configure GPIO pins */
	GPIO_InitStruct.Pin = _PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(_PORT, &GPIO_InitStruct);
}

void GamepadButton::UpdateButton()
{
	if(HAL_GPIO_ReadPin(but_port, but_pin) == GPIO_PIN_RESET)
	{
		IsPressing = true;
	}
	else
	{
		IsPressing = false;
	}

	if(IsPressing != lastStatus)
	{
		if(lastStatus == false)
		{
			//OnUp.Execute(this);
		}
		else
		{
			//OnDown.Execute(this);
		}

		lastStatus = IsPressing;
	}
}

