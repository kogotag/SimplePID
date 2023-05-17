#include "PID.h"
#include <stdexcept>
#include <queue>
#include <stdlib.h>

Regulator::Regulator(float minSignal, float maxSignal, float desiredValue, float minValue, float maxValue) {
	if (minSignal >= maxSignal) {
		throw std::invalid_argument("minSignal must be lower than maxSignal");
	}

	if (minValue >= maxValue) {
		throw std::invalid_argument("minValue must be lower than maxValue");
	}

	if (desiredValue < minValue || desiredValue > maxValue) {
		throw std::invalid_argument("desiredValue must be from minValue to maxValue");
	}

	this->minSignal = minSignal;
	this->maxSignal = maxSignal;
	this->minValue = minValue;
	this->maxValue = maxValue;
	this->desiredValue = desiredValue;
	this->kP = 0.0f;
	this->kI = 0.0f;
	this->kD = 0.0f;
	this->p = 0.0f;
	this->prevP = 0.0f;
	this->i = 0.0f;
	this->d = 0.0f;
	this->maxI = maxSignal;
	this->iQueueMaxSize = DEFAULT_I_QUEUE_MAX_SIZE;
	this->fps = DEFAULT_FPS;
	this->deltaT = 1.0f / fps;
	this->sensorValue = desiredValue;
	this->signal = 0.0f;
	this->valueIntervalLength = abs(maxValue - minValue);
	this->signalIntervalLength = abs(maxSignal - minSignal);
}

void Regulator::step(float sensorValue) {
	if (sensorValue < minValue || sensorValue > maxValue) {
		throw std::invalid_argument("sensorValue must be from minValue to maxValue");
	}

	this->sensorValue = sensorValue;
	calculateP();
	calculateI();
	calculateD();
	float tempSignal = (kP * p + kI * i + kD * d) * signalIntervalLength / 2.0f;
	signal = std::min(std::max(minSignal, tempSignal), maxSignal);
}

float Regulator::getSignal() {
	return this->signal;
}

void Regulator::setKP(float coefficient) {
	if (coefficient < 0.0f || coefficient > 1.0f) {
		throw std::invalid_argument("The coefficient must be from 0 to 1");
	}
	this->kP = coefficient;
}

void Regulator::setKI(float coefficient) {
	if (coefficient < 0.0f || coefficient > 1.0f) {
		throw std::invalid_argument("The coefficient must be from 0 to 1");
	}
	this->kI = coefficient;
}

void Regulator::setKD(float coefficient) {
	if (coefficient < 0.0f || coefficient > 1.0f) {
		throw std::invalid_argument("The coefficient must be from 0 to 1");
	}
	this->kD = coefficient;
}

void Regulator::setDeltaT(float deltaT) {
	if (deltaT <= 0.0f) {
		throw std::invalid_argument("deltaT must be greater than zero");
	}
	this->deltaT = deltaT;
}

void Regulator::setFps(int fps) {
	if (fps <= 0) {
		throw std::invalid_argument("fps must be positive value");
	}
	this->fps = fps;
}

void Regulator::setDesiredValue(float desiredValue) {
	if (desiredValue < minValue || desiredValue > maxValue) {
		throw std::invalid_argument("desiredValue must be from minValue to maxValue");
	}
	this->desiredValue = desiredValue;
}

void Regulator::calculateP() {
	prevP = p;
	p = mapValue(desiredValue) - mapValue(sensorValue);
}

void Regulator::calculateI() {
	i += p;
	iQueue.push(p);

	if (iQueue.size() > iQueueMaxSize) {
		i -= iQueue.front();
		iQueue.pop();
	}
}

void Regulator::calculateD() {
	d = (p - prevP) / deltaT;
}

float Regulator::mapValue(float value) {
	if (value < minValue || value > maxValue) {
		throw std::invalid_argument("value must be from minValue to maxValue");
	}
	return (value - minValue) / valueIntervalLength;
}