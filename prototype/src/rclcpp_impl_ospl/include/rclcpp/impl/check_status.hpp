#ifndef RCLCPP_RCLCPP_IMPL_CHECK_STATUS_HPP_
#define RCLCPP_RCLCPP_IMPL_CHECK_STATUS_HPP_
#include <iostream>

#include "ccpp_dds_dcps.h"

static const char *RetCodeName[13] = {
    "DDS_RETCODE_OK",
    "DDS_RETCODE_ERROR",
    "DDS_RETCODE_UNSUPPORTED",
    "DDS_RETCODE_BAD_PARAMETER",
    "DDS_RETCODE_PRECONDITION_NOT_MET",
    "DDS_RETCODE_OUT_OF_RESOURCES",
    "DDS_RETCODE_NOT_ENABLED",
    "DDS_RETCODE_IMMUTABLE_POLICY",
    "DDS_RETCODE_INCONSISTENT_POLICY",
    "DDS_RETCODE_ALREADY_DELETED",
    "DDS_RETCODE_TIMEOUT",
    "DDS_RETCODE_NO_DATA",
    "DDS_RETCODE_ILLEGAL_OPERATION"
};

inline const char * getErrorName(DDS::ReturnCode_t status)
{
    return RetCodeName[status];
}

inline void checkStatus(DDS::ReturnCode_t status, const char *info)
{
    if (status != DDS::RETCODE_OK && status != DDS::RETCODE_NO_DATA)
    {
        std::cerr << "Error in " << info << ": " << getErrorName(status) << std::endl;
        exit (1);
    }
}

inline void checkHandle(void *handle, const char *info)
{
    if (!handle)
    {
        std::cerr << "Error in " << info << ": Creation failed: invalid handle" << std::endl;
        exit (1);
    }
}

#endif /* RCLCPP_RCLCPP_IMPL_CHECK_STATUS_HPP_ */