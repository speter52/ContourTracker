#ifndef FUNCTION_CREATOR_H
#define FUNCTION_CREATOR_H

#include <math.h>
#include <limits.h>
#include <iostream>

using namespace std;

class FunctionCreator
{
	private:

		class Equation
		{
			public:
				virtual double operator()(double input)=0;

		};
		class Ushape : public Equation
		{
			public:
				Ushape(double left, double middle, int concavity);
				double operator()(double input);
			private:
				double middle;
				double width;
				double left;
				double right;
				double concavity;
		};

		double middle;
		double left;
		double right;
		Equation* function;

	public:
	
		FunctionCreator(int type, double left, double middle, double right, int concavity);
		FunctionCreator(int type, double left, double middle, int concavity);
		double operator()(double input);


};
#endif
