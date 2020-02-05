uint8_t setFrequency(TIM_HandleTypeDef *timer, uint32_t freq) {
	if(freq > 500) {
		return 0;
	}
	uint32_t tempprescaler = 1;
	uint32_t prescaler = 1;
	uint32_t counterR = 1;
	uint32_t tempcounterR = 1;
	uint32_t freqint = 0;
	double tempfreq = 0;
	double temperr = 1000;
	double err = 1000;
	uint8_t end = 0;

	if(COREFREQ % freq == 0) {
		freqint = COREFREQ / freq / 2;
		counterR = freqint;
		while(counterR > MAX16) {
			while((freqint % prescaler != 0) || (freqint / prescaler > MAX16)) {
				if(prescaler < MAX16) {
					prescaler++;
				} else {
					return 0;
				}
			}
			counterR = freqint/prescaler;
		}
	} else {
		double freqdouble = (double)COREFREQ / 2;
		tempfreq = freqdouble / freq;
		while(err != 0 && end != 1) {
			while(tempfreq / tempprescaler > MAX16) {
				if(tempprescaler < MAX16) {
					tempprescaler++;
				} else {
					end = 1;
				}
			}
			tempcounterR = freqdouble / tempprescaler / freq;
			tempfreq = freqdouble / tempprescaler / tempcounterR;
			if(tempfreq > freq) {
				temperr = tempfreq - freq;
			} else {
				temperr = freq - tempfreq;
			}
			if(temperr < err) {
				prescaler = tempprescaler;
				counterR = tempcounterR;
			}

			tempcounterR++;
			tempfreq = freqdouble / tempprescaler / tempcounterR;
			temperr = abs(tempfreq - freq);
			if(temperr < err) {
				prescaler = tempprescaler;
				counterR = tempcounterR;
			}
			end = 1;
		}
	}

	timer->Instance->PSC = prescaler - 1;
	timer->Instance->ARR = counterR - 1;

	double actfreq = (double)COREFREQ / prescaler / counterR / 2;
	double acterr = (double)freq - actfreq;

	return 1;
}

void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  static uint16_t lastInt;
  int16_t tempInt;
  if(isLookup) {
	  tempInt = lookup[lookupiter];
  } else {

  }

  if(lastInt == tempInt) {
	  return;
  } else {
	  lastInt = tempInt;
  }

  if(tempInt == 0) {
	  HAL_TIM_PWM_Stop(&htim1, 0);
	  HAL_TIM_PWM_Stop(&htim1, 2);
	  HAL_TIM_PWM_Stop(&htim1, 4);
	  HAL_TIM_PWM_Stop(&htim1, 6);
  } else if (tempInt < 0) {
	  TIM1->CCR1 = tempInt;
	  if(curCha != 1) {
		  HAL_TIM_PWM_Start(&htim1, 0);
		  HAL_TIM_PWM_Start(&htim1, 2);
		  HAL_TIM_PWM_Stop(&htim1, 4);
		  HAL_TIM_PWM_Stop(&htim1, 6);
		  curCha = 1;
	  }
  } else {
	  TIM1->CCR2 = tempInt;
	  curCha = 2;
	  if(curCha != 2) {
		  HAL_TIM_PWM_Stop(&htim1, 0);
		  HAL_TIM_PWM_Stop(&htim1, 2);
		  HAL_TIM_PWM_Start(&htim1, 4);
		  HAL_TIM_PWM_Start(&htim1, 6);
		  curCha = 2;
	  }
  }
  /* USER CODE END TIM1_UP_IRQn 1 */
}
