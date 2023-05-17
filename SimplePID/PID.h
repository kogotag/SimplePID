/*
��� ������������ ���� ����������� PID-����������
��� ������ ���������� ������� Wiki: 
*/

#pragma once

#include <queue>

#define DEFAULT_I_QUEUE_MAX_SIZE 30
#define DEFAULT_FPS 1000

class Regulator {
public:
	/*
	������ ������ ������ Regulator �� ���������� �����������:
	minSignal � ����������� �������� �������,
	maxSignal � ������������ �������� �������,
	desiredValue � �������� �������� ������������ ��������,
	minValue � ����������� �������� ������������ ��������,
	maxValue � ������������ �������� ������������ ��������
	*/
	Regulator(float minSignal, float maxSignal, float desiredValue, float minValue, float maxValue);

	/*
	�����, ���������� ���������� ����� �������� ������������ �������� � ���������.
	�������� sensor ������ ������������ ������� [minValue, maxValue]
	*/
	void step(float sensor);

	/*
	������������� ����� �������� ��� ������������ ���������������� ������������ ����������.
	�������� coefficient ������ ���� �� 0 �� 1
	*/
	void setKP(float coefficient);

	/*
	������������� ����� �������� ��� ������������ ������������ ������������ ����������.
	�������� coefficient ������ ���� �� 0 �� 1
	*/
	void setKI(float coefficient);

	/*
	������������� ����� �������� ��� ������������ ���������������� ������������ ����������.
	�������� coefficient ������ ���� �� 0 �� 1
	*/
	void setKD(float coefficient);

	/*
	���� �� ���������� �������� �� ����� ������ ���������� �������,
	�� ����� ���� ����� ����� ������� �������, ������� ������� ������ � ����������� ���������.
	�������� deltaT ������ ���� ������������� ���������
	*/
	void setDeltaT(float deltaT);

	/*
	���� ���������� �������� ����� ������ ���������� �������,
	�� ����� ���� ����� ����� ������ ������� ����������.
	�������� fps ������ ���� ������������� ���������
	*/
	void setFps(int fps);

	/*
	������������� �������� �������� ���������� ��������.
	�������� desiredValue ������ ������������ ������� [minValue, maxValue]
	*/
	void setDesiredValue(float desiredValue);

	/*
	���������� ����������� PID-������.
	��������! ���������, ��� ����� ���������� ������� �� ������� ����� step(),
	����������� PID-������.
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