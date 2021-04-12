#include "algorithm_base.h"

using namespace base_space;

AlgorithmBase::AlgorithmBase(std::string name):
    name_(name)
{

}

AlgorithmBase::~AlgorithmBase()
{

}

std::string AlgorithmBase::getName()
{
    return name_;
}

AlgorithmBase::AlgorithmBase():
    name_("dummy")
{

}

