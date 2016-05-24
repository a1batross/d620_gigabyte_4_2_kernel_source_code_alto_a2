#include "AudioAfeReg.h"

#define LOG_TAG "AudioAfeReg"
#ifndef ANDROID_DEFAULT_CODE
    #include <cutils/xlog.h>
    #ifdef ALOGE
    #undef ALOGE
    #endif
    #ifdef ALOGW
    #undef ALOGW
    #endif ALOGI
    #undef ALOGI
    #ifdef ALOGD
    #undef ALOGD
    #endif
    #ifdef ALOGV
    #undef ALOGV
    #endif
    #define ALOGE XLOGE
    #define ALOGW XLOGW
    #define ALOGI XLOGI
    #define ALOGD XLOGD
    #define ALOGV XLOGV
#else
    #include <utils/Log.h>
#endif

namespace android
{

AudioAfeReg *AudioAfeReg::UniqueAfeRegInstance = 0;

AudioAfeReg *AudioAfeReg::getInstance()
{
    if (UniqueAfeRegInstance == 0) {
        ALOGD("+UniqueAfeRegInstance \n");
        UniqueAfeRegInstance = new AudioAfeReg();
        ALOGD("-UniqueAfeRegInstance \n");
    }
    return UniqueAfeRegInstance;
}

AudioAfeReg::AudioAfeReg()
{
    ALOGD("AudioAfeReg contructor\n");
    mFd = 0;
    // here open audio hardware for register setting
    mFd = ::open(kAudioDeviceName, O_RDWR);
    if (mFd == 0) {
        ALOGE("AudioAfeReg open mFd fail");
    }
}

bool AudioAfeReg::CheckRegRange(uint32 offset)
{
    if (offset >= AFE_REGISTER_OFFSET) {
        ALOGW("SetAfeReg offset > AFE_REGISTER_OFFSET  = %d", AFE_REGISTER_OFFSET) ;
        return false;
    }
    return true;
}

uint32 AudioAfeReg::GetAfeFd()
{
    return mFd;
}

status_t AudioAfeReg::SetAfeReg(uint32 offset, uint32 value, uint32 mask)
{
    if (!CheckRegRange(offset)) {
        return INVALID_OPERATION;
    }
    ALOGD("SetAfeReg offset = 0x%x value = 0x%x mask = 0x%x", offset, value, mask);
    mReg_Control.offset = offset;
    mReg_Control.value = value;
    mReg_Control.mask = mask;
    ::ioctl(mFd, SET_AUDSYS_REG, &mReg_Control);

    return NO_ERROR;
}

uint32_t AudioAfeReg::GetAfeReg(uint32 address)
{
    if (!CheckRegRange(address)) {
        return 0;
    }

    // here need to get afe ref base on address
    mReg_Control.offset = address;
    mReg_Control.value = 0;
    mReg_Control.mask = 0xffffffff;
    ::ioctl(mFd, GET_AUDSYS_REG, &mReg_Control);

    return mReg_Control.value;
}

}

