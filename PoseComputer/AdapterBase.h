#pragma once
#ifndef ADAPTER_BASE_H_
#define ADAPTER_BASE_H_

namespace pose_lib
{

class AdapterBase
{
public:
    virtual int getNumberCorrespondences() = 0;
    typedef enum CorrespondanceType
    {
        POINT = 0,
        LINE
    } correspondance_t;
    
};  // AdapterBase

}  // namespace pose_lib

#endif  // ADAPTER_BASE_H_
