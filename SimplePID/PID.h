/*
Ёто заголовочный файл простейшего PID-регул€тора
ƒл€ полной информации читайте Wiki: 
*/

#pragma once

#include <queue>

#define DEFAULT_I_QUEUE_MAX_SIZE 30
#define DEFAULT_FPS 1000

class Regulator {
public:
	/*
	—оздаЄт объект класса Regulator со следующими параметрами:
	minSignal Ч минимальное значение сигнала,
	maxSignal Ч максимальное значение сигнала,
	desiredValue Ч желаемое значение регулируемой величины,
	minValue Ч минимальное значение регулируемой величины,
	maxValue Ч максимальное значение регулируемой величины
	*/
	Regulator(float minSignal, float maxSignal, float desiredValue, float minValue, float maxValue);

	/*
	ћетод, передающий измеренное новое значение регулируемой величины в регул€тор.
	ѕараметр sensor должен принадлежать отрезку [minValue, maxValue]
	*/
	void step(float sensor);

	/*
	”станавливает новое значение дл€ коэффициента пропорциональной составл€ющей регул€тора.
	ѕараметр coefficient должен быть от 0 до 1
	*/
	void setKP(float coefficient);

	/*
	”станавливает новое значение дл€ коэффициента интегральной составл€ющей регул€тора.
	ѕараметр coefficient должен быть от 0 до 1
	*/
	void setKI(float coefficient);

	/*
	”станавливает новое значение дл€ коэффициента дифференциальной составл€ющей регул€тора.
	ѕараметр coefficient должен быть от 0 до 1
	*/
	void setKD(float coefficient);

	/*
	≈сли вы обновл€ете значение не через равные промежутки времени,
	то через этот метод можно вручную указать, сколько времени прошло с предыдущего измерени€.
	ѕараметр deltaT должен быть положительной величиной
	*/
	void setDeltaT(float deltaT);

	/*
	≈сли обновл€ете значение через равные промежутки времени,
	то через этот метод можно задать частоту обновлени€.
	ѕараметр fps должен быть положительной величиной
	*/
	void setFps(int fps);

	/*
	”станавливает желаемое значение измер€емой величины.
	ѕараметр desiredValue должен принадлежать отрезку [minValue, maxValue]
	*/
	void setDesiredValue(float desiredValue);

	/*
	¬озвращает вычисленный PID-сигнал.
	¬нимание! ”бедитесь, что перед получением сигнала вы вызвали метод step(),
	вычисл€ющий PID-сигнал.
	*/
	float getSignal();
private:
	float kP;
	float kI;
	float kD;
	float p;
	float prevP;
	float deltaT;
	int fps;
	float i;
	float maxI;
	int iQueueMaxSize;
	std::queue<float> iQueue;
	float d;
	float signal;
	float minSignal;
	float maxSignal;
	float minValue;
	float maxValue;
	float valueIntervalLength;
	float signalIntervalLength;
	float sensorValue;
	float desiredValue;
	void calculateP();
	void calculateI();
	void calculateD();
	float mapValue(float value);
};