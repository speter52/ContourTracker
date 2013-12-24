//Simon Peter
//speter3@illinois.edu
//ARC Lab

#include "FunctionCreator.hpp"

//Public constructor to create specific functor
//Type determines what kind of equation to use, currently the only option is 0 for -x^2/(x^2-1)
FunctionCreator::FunctionCreator(int type, double left, double middle, int concavity)
{
	if(left>=middle)
	{
		cout<<"Invalid parameters for function\n";
		return;
	}
	if(type==0) function = new Ushape(left,middle,concavity);
}
//Public operator function that returns the output value after a FunctionCreator object has already been created
double FunctionCreator::operator()(double input)
{
	return (*function)(input);		
}

//Private constructor that uses -x^2/(x^2-1) as a model equation
FunctionCreator::Ushape::Ushape(double left, double middle, int concavity)
{
	this->middle = middle;
	this->width= 2*(middle-left);
	this->left = left;
	this->right = left + 2*(middle-left);
	this->concavity = concavity;
}

//Private helper that returns the y value of the function
double FunctionCreator::Ushape::operator()(double input)
{
	if(input<=left || input>=right) return INT_MAX;
	return -concavity * ( (pow(input-middle,2)) / ( (pow(input-middle,2)) - (pow(width/2,2)) ) );
}

