#pragma once


#include<stdint.h>
#include<cmath>

#ifdef ARDUINO_NAGA_LIB
#include<main.h>
#endif //ARDUINO_NAGA_LIB

namespace naga_libs{

#ifdef ARDUINO_NAGA_LIB
	class Arduino_Rotary_Encoder{

	};
#endif //ARDUINO_NAGA_LIB

#ifdef STM32_NAGA_LIB
	class rotary_encoder_stm32 {
		TIM_HandleTypeDef*htim;
	public:
		rotary_encoder_stm32() = delete;
		rotary_encoder_stm32(rotary_encoder_stm32& enc) {
			auto htim_ = enc.get_htim();
			HAL_TIM_Encoder_Stop(htim, TIM_CHANNEL_ALL);
			htim = htim_;
			HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);

		}
		rotary_encoder_stm32(rotary_encoder_stm32&& enc) {
					auto htim_ = enc.get_htim();
					HAL_TIM_Encoder_Stop(htim, TIM_CHANNEL_ALL);
					htim = htim_;
					HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
		}
		rotary_encoder_stm32(TIM_HandleTypeDef *_htim):htim(_htim){
			HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
		}

		void operator =(rotary_encoder_stm32& enc) {
			auto htim_ = enc.get_htim();
			HAL_TIM_Encoder_Stop(htim, TIM_CHANNEL_ALL);
			htim = htim_;
			HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
		}

		void operator =(rotary_encoder_stm32&& enc) {
			auto htim_ = enc.get_htim();
			HAL_TIM_Encoder_Stop(htim, TIM_CHANNEL_ALL);
			htim = htim_;
			HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
		}

		void operator()(TIM_HandleTypeDef *htim_) {
			HAL_TIM_Encoder_Stop(htim, TIM_CHANNEL_ALL);
			htim = htim_;
			HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
		}

		__attribute__((weak)) uint32_t operator()(){
			uint32_t counter = __HAL_TIM_GET_COUNTER(htim);
			__HAL_TIM_GET_COUNTER(htim) = 0;
			return counter;
		}

		TIM_HandleTypeDef* get_htim() {
			return htim;
		}
	};
#endif //STM32_NAGA_LIB

};
