/*=============================================================================
  Copyright (C) 2013 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  Please do not modify this file, because it was created automatically by a 
  code generator tool (AVT VimbaClassGenerator). So any manual modifications 
  will be lost if you run the tool again.

-------------------------------------------------------------------------------

  THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
  NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
  DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=============================================================================*/

#include "MakoCamera.h"

MakoCamera::MakoCamera (
    const char         *pCameraID,
    const char         *pCameraName,
    const char         *pCameraModel,
    const char         *pCameraSerialNumber,
    const char         *pInterfaceID,
    VmbInterfaceType    interfaceType,
    const char         *pInterfaceName,
    const char         *pInterfaceSerialNumber,
    VmbAccessModeType   interfacePermittedAccess 
    )
    : Camera (
        pCameraID, 
        pCameraName, 
        pCameraModel, 
        pCameraSerialNumber, 
        pInterfaceID, 
        interfaceType)
{
}


// Category /Acquisition
VmbErrorType MakoCamera::AcquisitionAbort ()
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetAcquisitionAbortFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->RunCommand ();
    return result;
}
VmbErrorType MakoCamera::GetAcquisitionAbortFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_AcquisitionAbortFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("AcquisitionAbort", m_AcquisitionAbortFeature);
        if (result != VmbErrorSuccess)
        {
            m_AcquisitionAbortFeature.reset();
            return result;
        }
    }
    feature = m_AcquisitionAbortFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetAcquisitionFrameCount (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetAcquisitionFrameCountFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetAcquisitionFrameCount (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetAcquisitionFrameCountFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetAcquisitionFrameCountFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_AcquisitionFrameCountFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("AcquisitionFrameCount", m_AcquisitionFrameCountFeature);
        if (result != VmbErrorSuccess)
        {
            m_AcquisitionFrameCountFeature.reset();
            return result;
        }
    }
    feature = m_AcquisitionFrameCountFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetAcquisitionFrameRateAbs (double & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetAcquisitionFrameRateAbsFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetAcquisitionFrameRateAbs (double value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetAcquisitionFrameRateAbsFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetAcquisitionFrameRateAbsFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_AcquisitionFrameRateAbsFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("AcquisitionFrameRateAbs", m_AcquisitionFrameRateAbsFeature);
        if (result != VmbErrorSuccess)
        {
            m_AcquisitionFrameRateAbsFeature.reset();
            return result;
        }
    }
    feature = m_AcquisitionFrameRateAbsFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetAcquisitionFrameRateLimit (double & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetAcquisitionFrameRateLimitFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetAcquisitionFrameRateLimitFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_AcquisitionFrameRateLimitFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("AcquisitionFrameRateLimit", m_AcquisitionFrameRateLimitFeature);
        if (result != VmbErrorSuccess)
        {
            m_AcquisitionFrameRateLimitFeature.reset();
            return result;
        }
    }
    feature = m_AcquisitionFrameRateLimitFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetAcquisitionMode (AcquisitionModeEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetAcquisitionModeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (AcquisitionModeEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetAcquisitionMode (AcquisitionModeEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetAcquisitionModeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetAcquisitionModeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_AcquisitionModeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("AcquisitionMode", m_AcquisitionModeFeature);
        if (result != VmbErrorSuccess)
        {
            m_AcquisitionModeFeature.reset();
            return result;
        }
    }
    feature = m_AcquisitionModeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::AcquisitionStart ()
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetAcquisitionStartFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->RunCommand ();
    return result;
}
VmbErrorType MakoCamera::GetAcquisitionStartFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_AcquisitionStartFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("AcquisitionStart", m_AcquisitionStartFeature);
        if (result != VmbErrorSuccess)
        {
            m_AcquisitionStartFeature.reset();
            return result;
        }
    }
    feature = m_AcquisitionStartFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::AcquisitionStop ()
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetAcquisitionStopFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->RunCommand ();
    return result;
}
VmbErrorType MakoCamera::GetAcquisitionStopFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_AcquisitionStopFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("AcquisitionStop", m_AcquisitionStopFeature);
        if (result != VmbErrorSuccess)
        {
            m_AcquisitionStopFeature.reset();
            return result;
        }
    }
    feature = m_AcquisitionStopFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetRecorderPreEventCount (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetRecorderPreEventCountFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetRecorderPreEventCount (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetRecorderPreEventCountFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetRecorderPreEventCountFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_RecorderPreEventCountFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("RecorderPreEventCount", m_RecorderPreEventCountFeature);
        if (result != VmbErrorSuccess)
        {
            m_RecorderPreEventCountFeature.reset();
            return result;
        }
    }
    feature = m_RecorderPreEventCountFeature;
    return VmbErrorSuccess;
}


// Category /Acquisition/Trigger
VmbErrorType MakoCamera::GetTriggerActivation (TriggerActivationEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetTriggerActivationFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (TriggerActivationEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetTriggerActivation (TriggerActivationEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetTriggerActivationFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetTriggerActivationFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_TriggerActivationFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("TriggerActivation", m_TriggerActivationFeature);
        if (result != VmbErrorSuccess)
        {
            m_TriggerActivationFeature.reset();
            return result;
        }
    }
    feature = m_TriggerActivationFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetTriggerDelayAbs (double & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetTriggerDelayAbsFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetTriggerDelayAbs (double value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetTriggerDelayAbsFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetTriggerDelayAbsFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_TriggerDelayAbsFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("TriggerDelayAbs", m_TriggerDelayAbsFeature);
        if (result != VmbErrorSuccess)
        {
            m_TriggerDelayAbsFeature.reset();
            return result;
        }
    }
    feature = m_TriggerDelayAbsFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetTriggerMode (TriggerModeEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetTriggerModeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (TriggerModeEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetTriggerMode (TriggerModeEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetTriggerModeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetTriggerModeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_TriggerModeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("TriggerMode", m_TriggerModeFeature);
        if (result != VmbErrorSuccess)
        {
            m_TriggerModeFeature.reset();
            return result;
        }
    }
    feature = m_TriggerModeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetTriggerOverlap (TriggerOverlapEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetTriggerOverlapFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (TriggerOverlapEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetTriggerOverlap (TriggerOverlapEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetTriggerOverlapFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetTriggerOverlapFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_TriggerOverlapFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("TriggerOverlap", m_TriggerOverlapFeature);
        if (result != VmbErrorSuccess)
        {
            m_TriggerOverlapFeature.reset();
            return result;
        }
    }
    feature = m_TriggerOverlapFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetTriggerSelector (TriggerSelectorEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetTriggerSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (TriggerSelectorEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetTriggerSelector (TriggerSelectorEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetTriggerSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetTriggerSelectorFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_TriggerSelectorFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("TriggerSelector", m_TriggerSelectorFeature);
        if (result != VmbErrorSuccess)
        {
            m_TriggerSelectorFeature.reset();
            return result;
        }
    }
    feature = m_TriggerSelectorFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::TriggerSoftware ()
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetTriggerSoftwareFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->RunCommand ();
    return result;
}
VmbErrorType MakoCamera::GetTriggerSoftwareFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_TriggerSoftwareFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("TriggerSoftware", m_TriggerSoftwareFeature);
        if (result != VmbErrorSuccess)
        {
            m_TriggerSoftwareFeature.reset();
            return result;
        }
    }
    feature = m_TriggerSoftwareFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetTriggerSource (TriggerSourceEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetTriggerSourceFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (TriggerSourceEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetTriggerSource (TriggerSourceEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetTriggerSourceFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetTriggerSourceFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_TriggerSourceFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("TriggerSource", m_TriggerSourceFeature);
        if (result != VmbErrorSuccess)
        {
            m_TriggerSourceFeature.reset();
            return result;
        }
    }
    feature = m_TriggerSourceFeature;
    return VmbErrorSuccess;
}


// Category /Controls
VmbErrorType MakoCamera::GetGamma (double & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGammaFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetGamma (double value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGammaFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGammaFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GammaFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("Gamma", m_GammaFeature);
        if (result != VmbErrorSuccess)
        {
            m_GammaFeature.reset();
            return result;
        }
    }
    feature = m_GammaFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetHue (double & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetHueFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetHue (double value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetHueFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetHueFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_HueFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("Hue", m_HueFeature);
        if (result != VmbErrorSuccess)
        {
            m_HueFeature.reset();
            return result;
        }
    }
    feature = m_HueFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetSaturation (double & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetSaturationFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetSaturation (double value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetSaturationFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetSaturationFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_SaturationFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("Saturation", m_SaturationFeature);
        if (result != VmbErrorSuccess)
        {
            m_SaturationFeature.reset();
            return result;
        }
    }
    feature = m_SaturationFeature;
    return VmbErrorSuccess;
}


// Category /Controls/BlackLevelControl
VmbErrorType MakoCamera::GetBlackLevel (double & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetBlackLevelFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetBlackLevel (double value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetBlackLevelFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetBlackLevelFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_BlackLevelFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("BlackLevel", m_BlackLevelFeature);
        if (result != VmbErrorSuccess)
        {
            m_BlackLevelFeature.reset();
            return result;
        }
    }
    feature = m_BlackLevelFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetBlackLevelSelector (BlackLevelSelectorEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetBlackLevelSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (BlackLevelSelectorEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetBlackLevelSelector (BlackLevelSelectorEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetBlackLevelSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetBlackLevelSelectorFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_BlackLevelSelectorFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("BlackLevelSelector", m_BlackLevelSelectorFeature);
        if (result != VmbErrorSuccess)
        {
            m_BlackLevelSelectorFeature.reset();
            return result;
        }
    }
    feature = m_BlackLevelSelectorFeature;
    return VmbErrorSuccess;
}


// Category /Controls/ColorTransformationControl
VmbErrorType MakoCamera::GetColorTransformationMode (ColorTransformationModeEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetColorTransformationModeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (ColorTransformationModeEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetColorTransformationMode (ColorTransformationModeEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetColorTransformationModeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetColorTransformationModeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_ColorTransformationModeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("ColorTransformationMode", m_ColorTransformationModeFeature);
        if (result != VmbErrorSuccess)
        {
            m_ColorTransformationModeFeature.reset();
            return result;
        }
    }
    feature = m_ColorTransformationModeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetColorTransformationSelector (ColorTransformationSelectorEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetColorTransformationSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (ColorTransformationSelectorEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetColorTransformationSelector (ColorTransformationSelectorEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetColorTransformationSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetColorTransformationSelectorFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_ColorTransformationSelectorFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("ColorTransformationSelector", m_ColorTransformationSelectorFeature);
        if (result != VmbErrorSuccess)
        {
            m_ColorTransformationSelectorFeature.reset();
            return result;
        }
    }
    feature = m_ColorTransformationSelectorFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetColorTransformationValue (double & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetColorTransformationValueFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetColorTransformationValue (double value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetColorTransformationValueFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetColorTransformationValueFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_ColorTransformationValueFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("ColorTransformationValue", m_ColorTransformationValueFeature);
        if (result != VmbErrorSuccess)
        {
            m_ColorTransformationValueFeature.reset();
            return result;
        }
    }
    feature = m_ColorTransformationValueFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetColorTransformationValueSelector (ColorTransformationValueSelectorEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetColorTransformationValueSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (ColorTransformationValueSelectorEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetColorTransformationValueSelector (ColorTransformationValueSelectorEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetColorTransformationValueSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetColorTransformationValueSelectorFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_ColorTransformationValueSelectorFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("ColorTransformationValueSelector", m_ColorTransformationValueSelectorFeature);
        if (result != VmbErrorSuccess)
        {
            m_ColorTransformationValueSelectorFeature.reset();
            return result;
        }
    }
    feature = m_ColorTransformationValueSelectorFeature;
    return VmbErrorSuccess;
}


// Category /Controls/DSPSubregion
VmbErrorType MakoCamera::GetDSPSubregionBottom (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetDSPSubregionBottomFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetDSPSubregionBottom (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetDSPSubregionBottomFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetDSPSubregionBottomFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_DSPSubregionBottomFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("DSPSubregionBottom", m_DSPSubregionBottomFeature);
        if (result != VmbErrorSuccess)
        {
            m_DSPSubregionBottomFeature.reset();
            return result;
        }
    }
    feature = m_DSPSubregionBottomFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetDSPSubregionLeft (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetDSPSubregionLeftFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetDSPSubregionLeft (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetDSPSubregionLeftFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetDSPSubregionLeftFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_DSPSubregionLeftFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("DSPSubregionLeft", m_DSPSubregionLeftFeature);
        if (result != VmbErrorSuccess)
        {
            m_DSPSubregionLeftFeature.reset();
            return result;
        }
    }
    feature = m_DSPSubregionLeftFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetDSPSubregionRight (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetDSPSubregionRightFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetDSPSubregionRight (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetDSPSubregionRightFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetDSPSubregionRightFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_DSPSubregionRightFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("DSPSubregionRight", m_DSPSubregionRightFeature);
        if (result != VmbErrorSuccess)
        {
            m_DSPSubregionRightFeature.reset();
            return result;
        }
    }
    feature = m_DSPSubregionRightFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetDSPSubregionTop (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetDSPSubregionTopFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetDSPSubregionTop (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetDSPSubregionTopFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetDSPSubregionTopFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_DSPSubregionTopFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("DSPSubregionTop", m_DSPSubregionTopFeature);
        if (result != VmbErrorSuccess)
        {
            m_DSPSubregionTopFeature.reset();
            return result;
        }
    }
    feature = m_DSPSubregionTopFeature;
    return VmbErrorSuccess;
}


// Category /Controls/DefectMask
VmbErrorType MakoCamera::GetDefectMaskColumnEnable (DefectMaskColumnEnableEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetDefectMaskColumnEnableFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (DefectMaskColumnEnableEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetDefectMaskColumnEnable (DefectMaskColumnEnableEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetDefectMaskColumnEnableFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetDefectMaskColumnEnableFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_DefectMaskColumnEnableFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("DefectMaskColumnEnable", m_DefectMaskColumnEnableFeature);
        if (result != VmbErrorSuccess)
        {
            m_DefectMaskColumnEnableFeature.reset();
            return result;
        }
    }
    feature = m_DefectMaskColumnEnableFeature;
    return VmbErrorSuccess;
}


// Category /Controls/Exposure
VmbErrorType MakoCamera::GetExposureAuto (ExposureAutoEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetExposureAutoFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (ExposureAutoEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetExposureAuto (ExposureAutoEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetExposureAutoFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetExposureAutoFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_ExposureAutoFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("ExposureAuto", m_ExposureAutoFeature);
        if (result != VmbErrorSuccess)
        {
            m_ExposureAutoFeature.reset();
            return result;
        }
    }
    feature = m_ExposureAutoFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetExposureMode (ExposureModeEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetExposureModeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (ExposureModeEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetExposureMode (ExposureModeEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetExposureModeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetExposureModeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_ExposureModeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("ExposureMode", m_ExposureModeFeature);
        if (result != VmbErrorSuccess)
        {
            m_ExposureModeFeature.reset();
            return result;
        }
    }
    feature = m_ExposureModeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetExposureTimeAbs (double & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetExposureTimeAbsFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetExposureTimeAbs (double value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetExposureTimeAbsFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetExposureTimeAbsFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_ExposureTimeAbsFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("ExposureTimeAbs", m_ExposureTimeAbsFeature);
        if (result != VmbErrorSuccess)
        {
            m_ExposureTimeAbsFeature.reset();
            return result;
        }
    }
    feature = m_ExposureTimeAbsFeature;
    return VmbErrorSuccess;
}


// Category /Controls/Exposure/ExposureAutoControl
VmbErrorType MakoCamera::GetExposureAutoAdjustTol (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetExposureAutoAdjustTolFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetExposureAutoAdjustTol (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetExposureAutoAdjustTolFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetExposureAutoAdjustTolFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_ExposureAutoAdjustTolFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("ExposureAutoAdjustTol", m_ExposureAutoAdjustTolFeature);
        if (result != VmbErrorSuccess)
        {
            m_ExposureAutoAdjustTolFeature.reset();
            return result;
        }
    }
    feature = m_ExposureAutoAdjustTolFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetExposureAutoAlg (ExposureAutoAlgEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetExposureAutoAlgFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (ExposureAutoAlgEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetExposureAutoAlg (ExposureAutoAlgEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetExposureAutoAlgFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetExposureAutoAlgFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_ExposureAutoAlgFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("ExposureAutoAlg", m_ExposureAutoAlgFeature);
        if (result != VmbErrorSuccess)
        {
            m_ExposureAutoAlgFeature.reset();
            return result;
        }
    }
    feature = m_ExposureAutoAlgFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetExposureAutoMax (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetExposureAutoMaxFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetExposureAutoMax (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetExposureAutoMaxFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetExposureAutoMaxFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_ExposureAutoMaxFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("ExposureAutoMax", m_ExposureAutoMaxFeature);
        if (result != VmbErrorSuccess)
        {
            m_ExposureAutoMaxFeature.reset();
            return result;
        }
    }
    feature = m_ExposureAutoMaxFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetExposureAutoMin (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetExposureAutoMinFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetExposureAutoMin (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetExposureAutoMinFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetExposureAutoMinFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_ExposureAutoMinFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("ExposureAutoMin", m_ExposureAutoMinFeature);
        if (result != VmbErrorSuccess)
        {
            m_ExposureAutoMinFeature.reset();
            return result;
        }
    }
    feature = m_ExposureAutoMinFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetExposureAutoOutliers (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetExposureAutoOutliersFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetExposureAutoOutliers (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetExposureAutoOutliersFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetExposureAutoOutliersFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_ExposureAutoOutliersFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("ExposureAutoOutliers", m_ExposureAutoOutliersFeature);
        if (result != VmbErrorSuccess)
        {
            m_ExposureAutoOutliersFeature.reset();
            return result;
        }
    }
    feature = m_ExposureAutoOutliersFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetExposureAutoRate (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetExposureAutoRateFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetExposureAutoRate (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetExposureAutoRateFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetExposureAutoRateFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_ExposureAutoRateFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("ExposureAutoRate", m_ExposureAutoRateFeature);
        if (result != VmbErrorSuccess)
        {
            m_ExposureAutoRateFeature.reset();
            return result;
        }
    }
    feature = m_ExposureAutoRateFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetExposureAutoTarget (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetExposureAutoTargetFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetExposureAutoTarget (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetExposureAutoTargetFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetExposureAutoTargetFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_ExposureAutoTargetFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("ExposureAutoTarget", m_ExposureAutoTargetFeature);
        if (result != VmbErrorSuccess)
        {
            m_ExposureAutoTargetFeature.reset();
            return result;
        }
    }
    feature = m_ExposureAutoTargetFeature;
    return VmbErrorSuccess;
}


// Category /Controls/GainControl
VmbErrorType MakoCamera::GetGain (double & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGainFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetGain (double value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGainFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGainFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GainFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("Gain", m_GainFeature);
        if (result != VmbErrorSuccess)
        {
            m_GainFeature.reset();
            return result;
        }
    }
    feature = m_GainFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGainAuto (GainAutoEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGainAutoFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (GainAutoEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetGainAuto (GainAutoEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGainAutoFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGainAutoFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GainAutoFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GainAuto", m_GainAutoFeature);
        if (result != VmbErrorSuccess)
        {
            m_GainAutoFeature.reset();
            return result;
        }
    }
    feature = m_GainAutoFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGainSelector (GainSelectorEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGainSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (GainSelectorEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetGainSelector (GainSelectorEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGainSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGainSelectorFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GainSelectorFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GainSelector", m_GainSelectorFeature);
        if (result != VmbErrorSuccess)
        {
            m_GainSelectorFeature.reset();
            return result;
        }
    }
    feature = m_GainSelectorFeature;
    return VmbErrorSuccess;
}


// Category /Controls/GainControl/GainAutoControl
VmbErrorType MakoCamera::GetGainAutoAdjustTol (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGainAutoAdjustTolFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetGainAutoAdjustTol (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGainAutoAdjustTolFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGainAutoAdjustTolFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GainAutoAdjustTolFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GainAutoAdjustTol", m_GainAutoAdjustTolFeature);
        if (result != VmbErrorSuccess)
        {
            m_GainAutoAdjustTolFeature.reset();
            return result;
        }
    }
    feature = m_GainAutoAdjustTolFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGainAutoMax (double & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGainAutoMaxFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetGainAutoMax (double value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGainAutoMaxFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGainAutoMaxFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GainAutoMaxFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GainAutoMax", m_GainAutoMaxFeature);
        if (result != VmbErrorSuccess)
        {
            m_GainAutoMaxFeature.reset();
            return result;
        }
    }
    feature = m_GainAutoMaxFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGainAutoMin (double & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGainAutoMinFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetGainAutoMin (double value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGainAutoMinFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGainAutoMinFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GainAutoMinFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GainAutoMin", m_GainAutoMinFeature);
        if (result != VmbErrorSuccess)
        {
            m_GainAutoMinFeature.reset();
            return result;
        }
    }
    feature = m_GainAutoMinFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGainAutoOutliers (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGainAutoOutliersFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetGainAutoOutliers (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGainAutoOutliersFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGainAutoOutliersFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GainAutoOutliersFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GainAutoOutliers", m_GainAutoOutliersFeature);
        if (result != VmbErrorSuccess)
        {
            m_GainAutoOutliersFeature.reset();
            return result;
        }
    }
    feature = m_GainAutoOutliersFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGainAutoRate (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGainAutoRateFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetGainAutoRate (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGainAutoRateFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGainAutoRateFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GainAutoRateFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GainAutoRate", m_GainAutoRateFeature);
        if (result != VmbErrorSuccess)
        {
            m_GainAutoRateFeature.reset();
            return result;
        }
    }
    feature = m_GainAutoRateFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGainAutoTarget (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGainAutoTargetFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetGainAutoTarget (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGainAutoTargetFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGainAutoTargetFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GainAutoTargetFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GainAutoTarget", m_GainAutoTargetFeature);
        if (result != VmbErrorSuccess)
        {
            m_GainAutoTargetFeature.reset();
            return result;
        }
    }
    feature = m_GainAutoTargetFeature;
    return VmbErrorSuccess;
}


// Category /Controls/LUTControl
VmbErrorType MakoCamera::GetLUTEnable (bool & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetLUTEnableFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetLUTEnable (bool value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetLUTEnableFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetLUTEnableFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_LUTEnableFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("LUTEnable", m_LUTEnableFeature);
        if (result != VmbErrorSuccess)
        {
            m_LUTEnableFeature.reset();
            return result;
        }
    }
    feature = m_LUTEnableFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetLUTIndex (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetLUTIndexFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetLUTIndex (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetLUTIndexFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetLUTIndexFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_LUTIndexFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("LUTIndex", m_LUTIndexFeature);
        if (result != VmbErrorSuccess)
        {
            m_LUTIndexFeature.reset();
            return result;
        }
    }
    feature = m_LUTIndexFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::LUTLoadAll ()
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetLUTLoadAllFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->RunCommand ();
    return result;
}
VmbErrorType MakoCamera::GetLUTLoadAllFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_LUTLoadAllFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("LUTLoadAll", m_LUTLoadAllFeature);
        if (result != VmbErrorSuccess)
        {
            m_LUTLoadAllFeature.reset();
            return result;
        }
    }
    feature = m_LUTLoadAllFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetLUTMode (LUTModeEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetLUTModeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (LUTModeEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetLUTMode (LUTModeEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetLUTModeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetLUTModeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_LUTModeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("LUTMode", m_LUTModeFeature);
        if (result != VmbErrorSuccess)
        {
            m_LUTModeFeature.reset();
            return result;
        }
    }
    feature = m_LUTModeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::LUTSaveAll ()
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetLUTSaveAllFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->RunCommand ();
    return result;
}
VmbErrorType MakoCamera::GetLUTSaveAllFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_LUTSaveAllFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("LUTSaveAll", m_LUTSaveAllFeature);
        if (result != VmbErrorSuccess)
        {
            m_LUTSaveAllFeature.reset();
            return result;
        }
    }
    feature = m_LUTSaveAllFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetLUTSelector (LUTSelectorEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetLUTSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (LUTSelectorEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetLUTSelector (LUTSelectorEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetLUTSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetLUTSelectorFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_LUTSelectorFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("LUTSelector", m_LUTSelectorFeature);
        if (result != VmbErrorSuccess)
        {
            m_LUTSelectorFeature.reset();
            return result;
        }
    }
    feature = m_LUTSelectorFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetLUTValue (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetLUTValueFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetLUTValue (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetLUTValueFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetLUTValueFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_LUTValueFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("LUTValue", m_LUTValueFeature);
        if (result != VmbErrorSuccess)
        {
            m_LUTValueFeature.reset();
            return result;
        }
    }
    feature = m_LUTValueFeature;
    return VmbErrorSuccess;
}


// Category /Controls/LUTControl/LUTInfo
VmbErrorType MakoCamera::GetLUTAddress (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetLUTAddressFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetLUTAddressFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_LUTAddressFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("LUTAddress", m_LUTAddressFeature);
        if (result != VmbErrorSuccess)
        {
            m_LUTAddressFeature.reset();
            return result;
        }
    }
    feature = m_LUTAddressFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetLUTBitDepthIn (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetLUTBitDepthInFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetLUTBitDepthInFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_LUTBitDepthInFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("LUTBitDepthIn", m_LUTBitDepthInFeature);
        if (result != VmbErrorSuccess)
        {
            m_LUTBitDepthInFeature.reset();
            return result;
        }
    }
    feature = m_LUTBitDepthInFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetLUTBitDepthOut (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetLUTBitDepthOutFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetLUTBitDepthOutFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_LUTBitDepthOutFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("LUTBitDepthOut", m_LUTBitDepthOutFeature);
        if (result != VmbErrorSuccess)
        {
            m_LUTBitDepthOutFeature.reset();
            return result;
        }
    }
    feature = m_LUTBitDepthOutFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetLUTSizeBytes (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetLUTSizeBytesFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetLUTSizeBytesFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_LUTSizeBytesFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("LUTSizeBytes", m_LUTSizeBytesFeature);
        if (result != VmbErrorSuccess)
        {
            m_LUTSizeBytesFeature.reset();
            return result;
        }
    }
    feature = m_LUTSizeBytesFeature;
    return VmbErrorSuccess;
}


// Category /Controls/Whitebalance
VmbErrorType MakoCamera::GetBalanceRatioAbs (double & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetBalanceRatioAbsFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetBalanceRatioAbs (double value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetBalanceRatioAbsFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetBalanceRatioAbsFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_BalanceRatioAbsFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("BalanceRatioAbs", m_BalanceRatioAbsFeature);
        if (result != VmbErrorSuccess)
        {
            m_BalanceRatioAbsFeature.reset();
            return result;
        }
    }
    feature = m_BalanceRatioAbsFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetBalanceRatioSelector (BalanceRatioSelectorEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetBalanceRatioSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (BalanceRatioSelectorEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetBalanceRatioSelector (BalanceRatioSelectorEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetBalanceRatioSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetBalanceRatioSelectorFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_BalanceRatioSelectorFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("BalanceRatioSelector", m_BalanceRatioSelectorFeature);
        if (result != VmbErrorSuccess)
        {
            m_BalanceRatioSelectorFeature.reset();
            return result;
        }
    }
    feature = m_BalanceRatioSelectorFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetBalanceWhiteAuto (BalanceWhiteAutoEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetBalanceWhiteAutoFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (BalanceWhiteAutoEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetBalanceWhiteAuto (BalanceWhiteAutoEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetBalanceWhiteAutoFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetBalanceWhiteAutoFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_BalanceWhiteAutoFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("BalanceWhiteAuto", m_BalanceWhiteAutoFeature);
        if (result != VmbErrorSuccess)
        {
            m_BalanceWhiteAutoFeature.reset();
            return result;
        }
    }
    feature = m_BalanceWhiteAutoFeature;
    return VmbErrorSuccess;
}


// Category /Controls/Whitebalance/BalanceWhiteAutoControl
VmbErrorType MakoCamera::GetBalanceWhiteAutoAdjustTol (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetBalanceWhiteAutoAdjustTolFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetBalanceWhiteAutoAdjustTol (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetBalanceWhiteAutoAdjustTolFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetBalanceWhiteAutoAdjustTolFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_BalanceWhiteAutoAdjustTolFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("BalanceWhiteAutoAdjustTol", m_BalanceWhiteAutoAdjustTolFeature);
        if (result != VmbErrorSuccess)
        {
            m_BalanceWhiteAutoAdjustTolFeature.reset();
            return result;
        }
    }
    feature = m_BalanceWhiteAutoAdjustTolFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetBalanceWhiteAutoRate (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetBalanceWhiteAutoRateFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetBalanceWhiteAutoRate (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetBalanceWhiteAutoRateFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetBalanceWhiteAutoRateFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_BalanceWhiteAutoRateFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("BalanceWhiteAutoRate", m_BalanceWhiteAutoRateFeature);
        if (result != VmbErrorSuccess)
        {
            m_BalanceWhiteAutoRateFeature.reset();
            return result;
        }
    }
    feature = m_BalanceWhiteAutoRateFeature;
    return VmbErrorSuccess;
}


// Category /DeviceStatus
VmbErrorType MakoCamera::GetDeviceTemperature (double & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetDeviceTemperatureFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetDeviceTemperatureFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_DeviceTemperatureFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("DeviceTemperature", m_DeviceTemperatureFeature);
        if (result != VmbErrorSuccess)
        {
            m_DeviceTemperatureFeature.reset();
            return result;
        }
    }
    feature = m_DeviceTemperatureFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetDeviceTemperatureSelector (DeviceTemperatureSelectorEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetDeviceTemperatureSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (DeviceTemperatureSelectorEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetDeviceTemperatureSelector (DeviceTemperatureSelectorEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetDeviceTemperatureSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetDeviceTemperatureSelectorFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_DeviceTemperatureSelectorFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("DeviceTemperatureSelector", m_DeviceTemperatureSelectorFeature);
        if (result != VmbErrorSuccess)
        {
            m_DeviceTemperatureSelectorFeature.reset();
            return result;
        }
    }
    feature = m_DeviceTemperatureSelectorFeature;
    return VmbErrorSuccess;
}


// Category /EventControl
VmbErrorType MakoCamera::GetEventNotification (EventNotificationEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventNotificationFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (EventNotificationEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetEventNotification (EventNotificationEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventNotificationFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventNotificationFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventNotificationFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventNotification", m_EventNotificationFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventNotificationFeature.reset();
            return result;
        }
    }
    feature = m_EventNotificationFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventSelector (EventSelectorEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (EventSelectorEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetEventSelector (EventSelectorEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventSelectorFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventSelectorFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventSelector", m_EventSelectorFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventSelectorFeature.reset();
            return result;
        }
    }
    feature = m_EventSelectorFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventsEnable1 (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventsEnable1Feature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetEventsEnable1 (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventsEnable1Feature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventsEnable1Feature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventsEnable1Feature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventsEnable1", m_EventsEnable1Feature);
        if (result != VmbErrorSuccess)
        {
            m_EventsEnable1Feature.reset();
            return result;
        }
    }
    feature = m_EventsEnable1Feature;
    return VmbErrorSuccess;
}


// Category /EventControl/EventData
VmbErrorType MakoCamera::GetEventOverflowTimestamp (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventOverflowTimestampFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventOverflowTimestampFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventOverflowTimestampFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventOverflowTimestamp", m_EventOverflowTimestampFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventOverflowTimestampFeature.reset();
            return result;
        }
    }
    feature = m_EventOverflowTimestampFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventOverflowFrameID (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventOverflowFrameIDFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventOverflowFrameIDFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventOverflowFrameIDFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventOverflowFrameID", m_EventOverflowFrameIDFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventOverflowFrameIDFeature.reset();
            return result;
        }
    }
    feature = m_EventOverflowFrameIDFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventLine1RisingEdgeTimestamp (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventLine1RisingEdgeTimestampFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventLine1RisingEdgeTimestampFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventLine1RisingEdgeTimestampFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventLine1RisingEdgeTimestamp", m_EventLine1RisingEdgeTimestampFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventLine1RisingEdgeTimestampFeature.reset();
            return result;
        }
    }
    feature = m_EventLine1RisingEdgeTimestampFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventLine1RisingEdgeFrameID (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventLine1RisingEdgeFrameIDFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventLine1RisingEdgeFrameIDFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventLine1RisingEdgeFrameIDFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventLine1RisingEdgeFrameID", m_EventLine1RisingEdgeFrameIDFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventLine1RisingEdgeFrameIDFeature.reset();
            return result;
        }
    }
    feature = m_EventLine1RisingEdgeFrameIDFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventLine1FallingEdgeTimestamp (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventLine1FallingEdgeTimestampFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventLine1FallingEdgeTimestampFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventLine1FallingEdgeTimestampFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventLine1FallingEdgeTimestamp", m_EventLine1FallingEdgeTimestampFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventLine1FallingEdgeTimestampFeature.reset();
            return result;
        }
    }
    feature = m_EventLine1FallingEdgeTimestampFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventLine1FallingEdgeFrameID (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventLine1FallingEdgeFrameIDFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventLine1FallingEdgeFrameIDFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventLine1FallingEdgeFrameIDFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventLine1FallingEdgeFrameID", m_EventLine1FallingEdgeFrameIDFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventLine1FallingEdgeFrameIDFeature.reset();
            return result;
        }
    }
    feature = m_EventLine1FallingEdgeFrameIDFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventFrameTriggerTimestamp (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventFrameTriggerTimestampFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventFrameTriggerTimestampFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventFrameTriggerTimestampFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventFrameTriggerTimestamp", m_EventFrameTriggerTimestampFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventFrameTriggerTimestampFeature.reset();
            return result;
        }
    }
    feature = m_EventFrameTriggerTimestampFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventFrameTriggerReadyTimestamp (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventFrameTriggerReadyTimestampFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventFrameTriggerReadyTimestampFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventFrameTriggerReadyTimestampFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventFrameTriggerReadyTimestamp", m_EventFrameTriggerReadyTimestampFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventFrameTriggerReadyTimestampFeature.reset();
            return result;
        }
    }
    feature = m_EventFrameTriggerReadyTimestampFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventFrameTriggerReadyFrameID (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventFrameTriggerReadyFrameIDFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventFrameTriggerReadyFrameIDFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventFrameTriggerReadyFrameIDFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventFrameTriggerReadyFrameID", m_EventFrameTriggerReadyFrameIDFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventFrameTriggerReadyFrameIDFeature.reset();
            return result;
        }
    }
    feature = m_EventFrameTriggerReadyFrameIDFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventFrameTriggerFrameID (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventFrameTriggerFrameIDFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventFrameTriggerFrameIDFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventFrameTriggerFrameIDFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventFrameTriggerFrameID", m_EventFrameTriggerFrameIDFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventFrameTriggerFrameIDFeature.reset();
            return result;
        }
    }
    feature = m_EventFrameTriggerFrameIDFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventExposureEndTimestamp (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventExposureEndTimestampFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventExposureEndTimestampFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventExposureEndTimestampFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventExposureEndTimestamp", m_EventExposureEndTimestampFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventExposureEndTimestampFeature.reset();
            return result;
        }
    }
    feature = m_EventExposureEndTimestampFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventExposureEndFrameID (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventExposureEndFrameIDFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventExposureEndFrameIDFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventExposureEndFrameIDFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventExposureEndFrameID", m_EventExposureEndFrameIDFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventExposureEndFrameIDFeature.reset();
            return result;
        }
    }
    feature = m_EventExposureEndFrameIDFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventErrorTimestamp (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventErrorTimestampFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventErrorTimestampFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventErrorTimestampFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventErrorTimestamp", m_EventErrorTimestampFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventErrorTimestampFeature.reset();
            return result;
        }
    }
    feature = m_EventErrorTimestampFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventErrorFrameID (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventErrorFrameIDFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventErrorFrameIDFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventErrorFrameIDFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventErrorFrameID", m_EventErrorFrameIDFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventErrorFrameIDFeature.reset();
            return result;
        }
    }
    feature = m_EventErrorFrameIDFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventAcquisitionStartTimestamp (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventAcquisitionStartTimestampFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventAcquisitionStartTimestampFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventAcquisitionStartTimestampFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventAcquisitionStartTimestamp", m_EventAcquisitionStartTimestampFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventAcquisitionStartTimestampFeature.reset();
            return result;
        }
    }
    feature = m_EventAcquisitionStartTimestampFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventAcquisitionStartFrameID (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventAcquisitionStartFrameIDFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventAcquisitionStartFrameIDFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventAcquisitionStartFrameIDFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventAcquisitionStartFrameID", m_EventAcquisitionStartFrameIDFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventAcquisitionStartFrameIDFeature.reset();
            return result;
        }
    }
    feature = m_EventAcquisitionStartFrameIDFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventAcquisitionRecordTriggerTimestamp (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventAcquisitionRecordTriggerTimestampFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventAcquisitionRecordTriggerTimestampFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventAcquisitionRecordTriggerTimestampFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventAcquisitionRecordTriggerTimestamp", m_EventAcquisitionRecordTriggerTimestampFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventAcquisitionRecordTriggerTimestampFeature.reset();
            return result;
        }
    }
    feature = m_EventAcquisitionRecordTriggerTimestampFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventAcquisitionRecordTriggerFrameID (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventAcquisitionRecordTriggerFrameIDFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventAcquisitionRecordTriggerFrameIDFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventAcquisitionRecordTriggerFrameIDFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventAcquisitionRecordTriggerFrameID", m_EventAcquisitionRecordTriggerFrameIDFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventAcquisitionRecordTriggerFrameIDFeature.reset();
            return result;
        }
    }
    feature = m_EventAcquisitionRecordTriggerFrameIDFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventAcquisitionEndTimestamp (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventAcquisitionEndTimestampFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventAcquisitionEndTimestampFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventAcquisitionEndTimestampFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventAcquisitionEndTimestamp", m_EventAcquisitionEndTimestampFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventAcquisitionEndTimestampFeature.reset();
            return result;
        }
    }
    feature = m_EventAcquisitionEndTimestampFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventAcquisitionEndFrameID (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventAcquisitionEndFrameIDFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventAcquisitionEndFrameIDFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventAcquisitionEndFrameIDFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventAcquisitionEndFrameID", m_EventAcquisitionEndFrameIDFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventAcquisitionEndFrameIDFeature.reset();
            return result;
        }
    }
    feature = m_EventAcquisitionEndFrameIDFeature;
    return VmbErrorSuccess;
}


// Category /EventControl/EventID
VmbErrorType MakoCamera::GetEventAcquisitionEnd (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventAcquisitionEndFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventAcquisitionEndFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventAcquisitionEndFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventAcquisitionEnd", m_EventAcquisitionEndFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventAcquisitionEndFeature.reset();
            return result;
        }
    }
    feature = m_EventAcquisitionEndFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventAcquisitionRecordTrigger (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventAcquisitionRecordTriggerFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventAcquisitionRecordTriggerFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventAcquisitionRecordTriggerFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventAcquisitionRecordTrigger", m_EventAcquisitionRecordTriggerFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventAcquisitionRecordTriggerFeature.reset();
            return result;
        }
    }
    feature = m_EventAcquisitionRecordTriggerFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventAcquisitionStart (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventAcquisitionStartFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventAcquisitionStartFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventAcquisitionStartFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventAcquisitionStart", m_EventAcquisitionStartFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventAcquisitionStartFeature.reset();
            return result;
        }
    }
    feature = m_EventAcquisitionStartFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventError (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventErrorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventErrorFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventErrorFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventError", m_EventErrorFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventErrorFeature.reset();
            return result;
        }
    }
    feature = m_EventErrorFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventExposureEnd (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventExposureEndFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventExposureEndFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventExposureEndFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventExposureEnd", m_EventExposureEndFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventExposureEndFeature.reset();
            return result;
        }
    }
    feature = m_EventExposureEndFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventFrameTrigger (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventFrameTriggerFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventFrameTriggerFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventFrameTriggerFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventFrameTrigger", m_EventFrameTriggerFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventFrameTriggerFeature.reset();
            return result;
        }
    }
    feature = m_EventFrameTriggerFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventFrameTriggerReady (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventFrameTriggerReadyFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventFrameTriggerReadyFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventFrameTriggerReadyFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventFrameTriggerReady", m_EventFrameTriggerReadyFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventFrameTriggerReadyFeature.reset();
            return result;
        }
    }
    feature = m_EventFrameTriggerReadyFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventLine1FallingEdge (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventLine1FallingEdgeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventLine1FallingEdgeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventLine1FallingEdgeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventLine1FallingEdge", m_EventLine1FallingEdgeFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventLine1FallingEdgeFeature.reset();
            return result;
        }
    }
    feature = m_EventLine1FallingEdgeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventLine1RisingEdge (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventLine1RisingEdgeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventLine1RisingEdgeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventLine1RisingEdgeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventLine1RisingEdge", m_EventLine1RisingEdgeFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventLine1RisingEdgeFeature.reset();
            return result;
        }
    }
    feature = m_EventLine1RisingEdgeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventLine2FallingEdge (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventLine2FallingEdgeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventLine2FallingEdgeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventLine2FallingEdgeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventLine2FallingEdge", m_EventLine2FallingEdgeFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventLine2FallingEdgeFeature.reset();
            return result;
        }
    }
    feature = m_EventLine2FallingEdgeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventLine2RisingEdge (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventLine2RisingEdgeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventLine2RisingEdgeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventLine2RisingEdgeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventLine2RisingEdge", m_EventLine2RisingEdgeFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventLine2RisingEdgeFeature.reset();
            return result;
        }
    }
    feature = m_EventLine2RisingEdgeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventLine3FallingEdge (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventLine3FallingEdgeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventLine3FallingEdgeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventLine3FallingEdgeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventLine3FallingEdge", m_EventLine3FallingEdgeFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventLine3FallingEdgeFeature.reset();
            return result;
        }
    }
    feature = m_EventLine3FallingEdgeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventLine3RisingEdge (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventLine3RisingEdgeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventLine3RisingEdgeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventLine3RisingEdgeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventLine3RisingEdge", m_EventLine3RisingEdgeFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventLine3RisingEdgeFeature.reset();
            return result;
        }
    }
    feature = m_EventLine3RisingEdgeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventLine4FallingEdge (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventLine4FallingEdgeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventLine4FallingEdgeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventLine4FallingEdgeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventLine4FallingEdge", m_EventLine4FallingEdgeFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventLine4FallingEdgeFeature.reset();
            return result;
        }
    }
    feature = m_EventLine4FallingEdgeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventLine4RisingEdge (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventLine4RisingEdgeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventLine4RisingEdgeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventLine4RisingEdgeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventLine4RisingEdge", m_EventLine4RisingEdgeFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventLine4RisingEdgeFeature.reset();
            return result;
        }
    }
    feature = m_EventLine4RisingEdgeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetEventOverflow (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetEventOverflowFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetEventOverflowFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_EventOverflowFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("EventOverflow", m_EventOverflowFeature);
        if (result != VmbErrorSuccess)
        {
            m_EventOverflowFeature.reset();
            return result;
        }
    }
    feature = m_EventOverflowFeature;
    return VmbErrorSuccess;
}


// Category /GigE
VmbErrorType MakoCamera::GetBandwidthControlMode (BandwidthControlModeEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetBandwidthControlModeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (BandwidthControlModeEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetBandwidthControlMode (BandwidthControlModeEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetBandwidthControlModeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetBandwidthControlModeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_BandwidthControlModeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("BandwidthControlMode", m_BandwidthControlModeFeature);
        if (result != VmbErrorSuccess)
        {
            m_BandwidthControlModeFeature.reset();
            return result;
        }
    }
    feature = m_BandwidthControlModeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetChunkModeActive (bool & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetChunkModeActiveFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetChunkModeActive (bool value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetChunkModeActiveFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetChunkModeActiveFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_ChunkModeActiveFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("ChunkModeActive", m_ChunkModeActiveFeature);
        if (result != VmbErrorSuccess)
        {
            m_ChunkModeActiveFeature.reset();
            return result;
        }
    }
    feature = m_ChunkModeActiveFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGevDeviceMACAddress (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGevDeviceMACAddressFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGevDeviceMACAddressFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GevDeviceMACAddressFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GevDeviceMACAddress", m_GevDeviceMACAddressFeature);
        if (result != VmbErrorSuccess)
        {
            m_GevDeviceMACAddressFeature.reset();
            return result;
        }
    }
    feature = m_GevDeviceMACAddressFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGevSCPSPacketSize (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGevSCPSPacketSizeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetGevSCPSPacketSize (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGevSCPSPacketSizeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGevSCPSPacketSizeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GevSCPSPacketSizeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GevSCPSPacketSize", m_GevSCPSPacketSizeFeature);
        if (result != VmbErrorSuccess)
        {
            m_GevSCPSPacketSizeFeature.reset();
            return result;
        }
    }
    feature = m_GevSCPSPacketSizeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetNonImagePayloadSize (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetNonImagePayloadSizeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetNonImagePayloadSizeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_NonImagePayloadSizeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("NonImagePayloadSize", m_NonImagePayloadSizeFeature);
        if (result != VmbErrorSuccess)
        {
            m_NonImagePayloadSizeFeature.reset();
            return result;
        }
    }
    feature = m_NonImagePayloadSizeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetPayloadSize (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetPayloadSizeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetPayloadSizeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_PayloadSizeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("PayloadSize", m_PayloadSizeFeature);
        if (result != VmbErrorSuccess)
        {
            m_PayloadSizeFeature.reset();
            return result;
        }
    }
    feature = m_PayloadSizeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetStreamBytesPerSecond (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStreamBytesPerSecondFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetStreamBytesPerSecond (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStreamBytesPerSecondFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetStreamBytesPerSecondFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_StreamBytesPerSecondFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("StreamBytesPerSecond", m_StreamBytesPerSecondFeature);
        if (result != VmbErrorSuccess)
        {
            m_StreamBytesPerSecondFeature.reset();
            return result;
        }
    }
    feature = m_StreamBytesPerSecondFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetStreamFrameRateConstrain (bool & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStreamFrameRateConstrainFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetStreamFrameRateConstrain (bool value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStreamFrameRateConstrainFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetStreamFrameRateConstrainFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_StreamFrameRateConstrainFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("StreamFrameRateConstrain", m_StreamFrameRateConstrainFeature);
        if (result != VmbErrorSuccess)
        {
            m_StreamFrameRateConstrainFeature.reset();
            return result;
        }
    }
    feature = m_StreamFrameRateConstrainFeature;
    return VmbErrorSuccess;
}


// Category /GigE/Configuration
VmbErrorType MakoCamera::GetGevIPConfigurationMode (GevIPConfigurationModeEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGevIPConfigurationModeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (GevIPConfigurationModeEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::GetGevIPConfigurationModeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GevIPConfigurationModeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GevIPConfigurationMode", m_GevIPConfigurationModeFeature);
        if (result != VmbErrorSuccess)
        {
            m_GevIPConfigurationModeFeature.reset();
            return result;
        }
    }
    feature = m_GevIPConfigurationModeFeature;
    return VmbErrorSuccess;
}


// Category /GigE/Current
VmbErrorType MakoCamera::GetGevCurrentDefaultGateway (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGevCurrentDefaultGatewayFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGevCurrentDefaultGatewayFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GevCurrentDefaultGatewayFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GevCurrentDefaultGateway", m_GevCurrentDefaultGatewayFeature);
        if (result != VmbErrorSuccess)
        {
            m_GevCurrentDefaultGatewayFeature.reset();
            return result;
        }
    }
    feature = m_GevCurrentDefaultGatewayFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGevCurrentIPAddress (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGevCurrentIPAddressFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGevCurrentIPAddressFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GevCurrentIPAddressFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GevCurrentIPAddress", m_GevCurrentIPAddressFeature);
        if (result != VmbErrorSuccess)
        {
            m_GevCurrentIPAddressFeature.reset();
            return result;
        }
    }
    feature = m_GevCurrentIPAddressFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGevCurrentSubnetMask (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGevCurrentSubnetMaskFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGevCurrentSubnetMaskFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GevCurrentSubnetMaskFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GevCurrentSubnetMask", m_GevCurrentSubnetMaskFeature);
        if (result != VmbErrorSuccess)
        {
            m_GevCurrentSubnetMaskFeature.reset();
            return result;
        }
    }
    feature = m_GevCurrentSubnetMaskFeature;
    return VmbErrorSuccess;
}


// Category /GigE/GVCP
VmbErrorType MakoCamera::GetGVCPCmdRetries (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVCPCmdRetriesFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetGVCPCmdRetries (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVCPCmdRetriesFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGVCPCmdRetriesFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GVCPCmdRetriesFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GVCPCmdRetries", m_GVCPCmdRetriesFeature);
        if (result != VmbErrorSuccess)
        {
            m_GVCPCmdRetriesFeature.reset();
            return result;
        }
    }
    feature = m_GVCPCmdRetriesFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGVCPCmdTimeout (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVCPCmdTimeoutFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetGVCPCmdTimeout (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVCPCmdTimeoutFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGVCPCmdTimeoutFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GVCPCmdTimeoutFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GVCPCmdTimeout", m_GVCPCmdTimeoutFeature);
        if (result != VmbErrorSuccess)
        {
            m_GVCPCmdTimeoutFeature.reset();
            return result;
        }
    }
    feature = m_GVCPCmdTimeoutFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGVCPHBInterval (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVCPHBIntervalFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetGVCPHBInterval (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVCPHBIntervalFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGVCPHBIntervalFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GVCPHBIntervalFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GVCPHBInterval", m_GVCPHBIntervalFeature);
        if (result != VmbErrorSuccess)
        {
            m_GVCPHBIntervalFeature.reset();
            return result;
        }
    }
    feature = m_GVCPHBIntervalFeature;
    return VmbErrorSuccess;
}


// Category /GigE/Persistent
VmbErrorType MakoCamera::GetGevPersistentDefaultGateway (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGevPersistentDefaultGatewayFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGevPersistentDefaultGatewayFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GevPersistentDefaultGatewayFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GevPersistentDefaultGateway", m_GevPersistentDefaultGatewayFeature);
        if (result != VmbErrorSuccess)
        {
            m_GevPersistentDefaultGatewayFeature.reset();
            return result;
        }
    }
    feature = m_GevPersistentDefaultGatewayFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGevPersistentIPAddress (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGevPersistentIPAddressFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGevPersistentIPAddressFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GevPersistentIPAddressFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GevPersistentIPAddress", m_GevPersistentIPAddressFeature);
        if (result != VmbErrorSuccess)
        {
            m_GevPersistentIPAddressFeature.reset();
            return result;
        }
    }
    feature = m_GevPersistentIPAddressFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGevPersistentSubnetMask (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGevPersistentSubnetMaskFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGevPersistentSubnetMaskFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GevPersistentSubnetMaskFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GevPersistentSubnetMask", m_GevPersistentSubnetMaskFeature);
        if (result != VmbErrorSuccess)
        {
            m_GevPersistentSubnetMaskFeature.reset();
            return result;
        }
    }
    feature = m_GevPersistentSubnetMaskFeature;
    return VmbErrorSuccess;
}


// Category /GigE/StreamHold
VmbErrorType MakoCamera::GetStreamHoldCapacity (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStreamHoldCapacityFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetStreamHoldCapacityFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_StreamHoldCapacityFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("StreamHoldCapacity", m_StreamHoldCapacityFeature);
        if (result != VmbErrorSuccess)
        {
            m_StreamHoldCapacityFeature.reset();
            return result;
        }
    }
    feature = m_StreamHoldCapacityFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetStreamHoldEnable (StreamHoldEnableEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStreamHoldEnableFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (StreamHoldEnableEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetStreamHoldEnable (StreamHoldEnableEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStreamHoldEnableFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetStreamHoldEnableFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_StreamHoldEnableFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("StreamHoldEnable", m_StreamHoldEnableFeature);
        if (result != VmbErrorSuccess)
        {
            m_StreamHoldEnableFeature.reset();
            return result;
        }
    }
    feature = m_StreamHoldEnableFeature;
    return VmbErrorSuccess;
}


// Category /GigE/Timestamp
VmbErrorType MakoCamera::GevTimestampControlLatch ()
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGevTimestampControlLatchFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->RunCommand ();
    return result;
}
VmbErrorType MakoCamera::GetGevTimestampControlLatchFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GevTimestampControlLatchFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GevTimestampControlLatch", m_GevTimestampControlLatchFeature);
        if (result != VmbErrorSuccess)
        {
            m_GevTimestampControlLatchFeature.reset();
            return result;
        }
    }
    feature = m_GevTimestampControlLatchFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GevTimestampControlReset ()
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGevTimestampControlResetFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->RunCommand ();
    return result;
}
VmbErrorType MakoCamera::GetGevTimestampControlResetFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GevTimestampControlResetFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GevTimestampControlReset", m_GevTimestampControlResetFeature);
        if (result != VmbErrorSuccess)
        {
            m_GevTimestampControlResetFeature.reset();
            return result;
        }
    }
    feature = m_GevTimestampControlResetFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGevTimestampTickFrequency (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGevTimestampTickFrequencyFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGevTimestampTickFrequencyFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GevTimestampTickFrequencyFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GevTimestampTickFrequency", m_GevTimestampTickFrequencyFeature);
        if (result != VmbErrorSuccess)
        {
            m_GevTimestampTickFrequencyFeature.reset();
            return result;
        }
    }
    feature = m_GevTimestampTickFrequencyFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGevTimestampValue (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGevTimestampValueFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGevTimestampValueFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GevTimestampValueFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GevTimestampValue", m_GevTimestampValueFeature);
        if (result != VmbErrorSuccess)
        {
            m_GevTimestampValueFeature.reset();
            return result;
        }
    }
    feature = m_GevTimestampValueFeature;
    return VmbErrorSuccess;
}


// Category /IO/Strobe
VmbErrorType MakoCamera::GetStrobeDelay (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStrobeDelayFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetStrobeDelay (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStrobeDelayFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetStrobeDelayFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_StrobeDelayFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("StrobeDelay", m_StrobeDelayFeature);
        if (result != VmbErrorSuccess)
        {
            m_StrobeDelayFeature.reset();
            return result;
        }
    }
    feature = m_StrobeDelayFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetStrobeDuration (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStrobeDurationFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetStrobeDuration (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStrobeDurationFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetStrobeDurationFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_StrobeDurationFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("StrobeDuration", m_StrobeDurationFeature);
        if (result != VmbErrorSuccess)
        {
            m_StrobeDurationFeature.reset();
            return result;
        }
    }
    feature = m_StrobeDurationFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetStrobeDurationMode (StrobeDurationModeEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStrobeDurationModeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (StrobeDurationModeEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetStrobeDurationMode (StrobeDurationModeEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStrobeDurationModeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetStrobeDurationModeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_StrobeDurationModeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("StrobeDurationMode", m_StrobeDurationModeFeature);
        if (result != VmbErrorSuccess)
        {
            m_StrobeDurationModeFeature.reset();
            return result;
        }
    }
    feature = m_StrobeDurationModeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetStrobeSource (StrobeSourceEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStrobeSourceFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (StrobeSourceEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetStrobeSource (StrobeSourceEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStrobeSourceFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetStrobeSourceFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_StrobeSourceFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("StrobeSource", m_StrobeSourceFeature);
        if (result != VmbErrorSuccess)
        {
            m_StrobeSourceFeature.reset();
            return result;
        }
    }
    feature = m_StrobeSourceFeature;
    return VmbErrorSuccess;
}


// Category /IO/SyncIn
VmbErrorType MakoCamera::GetSyncInGlitchFilter (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetSyncInGlitchFilterFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetSyncInGlitchFilter (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetSyncInGlitchFilterFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetSyncInGlitchFilterFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_SyncInGlitchFilterFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("SyncInGlitchFilter", m_SyncInGlitchFilterFeature);
        if (result != VmbErrorSuccess)
        {
            m_SyncInGlitchFilterFeature.reset();
            return result;
        }
    }
    feature = m_SyncInGlitchFilterFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetSyncInLevels (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetSyncInLevelsFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetSyncInLevelsFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_SyncInLevelsFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("SyncInLevels", m_SyncInLevelsFeature);
        if (result != VmbErrorSuccess)
        {
            m_SyncInLevelsFeature.reset();
            return result;
        }
    }
    feature = m_SyncInLevelsFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetSyncInSelector (SyncInSelectorEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetSyncInSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (SyncInSelectorEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetSyncInSelector (SyncInSelectorEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetSyncInSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetSyncInSelectorFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_SyncInSelectorFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("SyncInSelector", m_SyncInSelectorFeature);
        if (result != VmbErrorSuccess)
        {
            m_SyncInSelectorFeature.reset();
            return result;
        }
    }
    feature = m_SyncInSelectorFeature;
    return VmbErrorSuccess;
}


// Category /IO/SyncOut
VmbErrorType MakoCamera::GetSyncOutLevels (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetSyncOutLevelsFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetSyncOutLevels (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetSyncOutLevelsFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetSyncOutLevelsFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_SyncOutLevelsFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("SyncOutLevels", m_SyncOutLevelsFeature);
        if (result != VmbErrorSuccess)
        {
            m_SyncOutLevelsFeature.reset();
            return result;
        }
    }
    feature = m_SyncOutLevelsFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetSyncOutPolarity (SyncOutPolarityEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetSyncOutPolarityFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (SyncOutPolarityEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetSyncOutPolarity (SyncOutPolarityEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetSyncOutPolarityFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetSyncOutPolarityFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_SyncOutPolarityFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("SyncOutPolarity", m_SyncOutPolarityFeature);
        if (result != VmbErrorSuccess)
        {
            m_SyncOutPolarityFeature.reset();
            return result;
        }
    }
    feature = m_SyncOutPolarityFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetSyncOutSelector (SyncOutSelectorEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetSyncOutSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (SyncOutSelectorEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetSyncOutSelector (SyncOutSelectorEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetSyncOutSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetSyncOutSelectorFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_SyncOutSelectorFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("SyncOutSelector", m_SyncOutSelectorFeature);
        if (result != VmbErrorSuccess)
        {
            m_SyncOutSelectorFeature.reset();
            return result;
        }
    }
    feature = m_SyncOutSelectorFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetSyncOutSource (SyncOutSourceEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetSyncOutSourceFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (SyncOutSourceEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetSyncOutSource (SyncOutSourceEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetSyncOutSourceFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetSyncOutSourceFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_SyncOutSourceFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("SyncOutSource", m_SyncOutSourceFeature);
        if (result != VmbErrorSuccess)
        {
            m_SyncOutSourceFeature.reset();
            return result;
        }
    }
    feature = m_SyncOutSourceFeature;
    return VmbErrorSuccess;
}


// Category /ImageFormat
VmbErrorType MakoCamera::GetHeight (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetHeightFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetHeight (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetHeightFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetHeightFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_HeightFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("Height", m_HeightFeature);
        if (result != VmbErrorSuccess)
        {
            m_HeightFeature.reset();
            return result;
        }
    }
    feature = m_HeightFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetHeightMax (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetHeightMaxFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetHeightMaxFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_HeightMaxFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("HeightMax", m_HeightMaxFeature);
        if (result != VmbErrorSuccess)
        {
            m_HeightMaxFeature.reset();
            return result;
        }
    }
    feature = m_HeightMaxFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetImageSize (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetImageSizeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetImageSizeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_ImageSizeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("ImageSize", m_ImageSizeFeature);
        if (result != VmbErrorSuccess)
        {
            m_ImageSizeFeature.reset();
            return result;
        }
    }
    feature = m_ImageSizeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetOffsetX (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetOffsetXFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetOffsetX (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetOffsetXFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetOffsetXFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_OffsetXFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("OffsetX", m_OffsetXFeature);
        if (result != VmbErrorSuccess)
        {
            m_OffsetXFeature.reset();
            return result;
        }
    }
    feature = m_OffsetXFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetOffsetY (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetOffsetYFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetOffsetY (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetOffsetYFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetOffsetYFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_OffsetYFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("OffsetY", m_OffsetYFeature);
        if (result != VmbErrorSuccess)
        {
            m_OffsetYFeature.reset();
            return result;
        }
    }
    feature = m_OffsetYFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetPixelFormat (PixelFormatEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetPixelFormatFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (PixelFormatEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetPixelFormat (PixelFormatEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetPixelFormatFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetPixelFormatFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_PixelFormatFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("PixelFormat", m_PixelFormatFeature);
        if (result != VmbErrorSuccess)
        {
            m_PixelFormatFeature.reset();
            return result;
        }
    }
    feature = m_PixelFormatFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetWidth (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetWidthFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetWidth (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetWidthFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetWidthFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_WidthFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("Width", m_WidthFeature);
        if (result != VmbErrorSuccess)
        {
            m_WidthFeature.reset();
            return result;
        }
    }
    feature = m_WidthFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetWidthMax (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetWidthMaxFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetWidthMaxFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_WidthMaxFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("WidthMax", m_WidthMaxFeature);
        if (result != VmbErrorSuccess)
        {
            m_WidthMaxFeature.reset();
            return result;
        }
    }
    feature = m_WidthMaxFeature;
    return VmbErrorSuccess;
}


// Category /ImageMode
VmbErrorType MakoCamera::GetSensorHeight (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetSensorHeightFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetSensorHeightFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_SensorHeightFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("SensorHeight", m_SensorHeightFeature);
        if (result != VmbErrorSuccess)
        {
            m_SensorHeightFeature.reset();
            return result;
        }
    }
    feature = m_SensorHeightFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetSensorWidth (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetSensorWidthFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetSensorWidthFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_SensorWidthFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("SensorWidth", m_SensorWidthFeature);
        if (result != VmbErrorSuccess)
        {
            m_SensorWidthFeature.reset();
            return result;
        }
    }
    feature = m_SensorWidthFeature;
    return VmbErrorSuccess;
}


// Category /Info
VmbErrorType MakoCamera::GetDeviceFirmwareVersion (std::string & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetDeviceFirmwareVersionFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetDeviceFirmwareVersionFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_DeviceFirmwareVersionFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("DeviceFirmwareVersion", m_DeviceFirmwareVersionFeature);
        if (result != VmbErrorSuccess)
        {
            m_DeviceFirmwareVersionFeature.reset();
            return result;
        }
    }
    feature = m_DeviceFirmwareVersionFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetDeviceID (std::string & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetDeviceIDFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetDeviceIDFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_DeviceIDFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("DeviceID", m_DeviceIDFeature);
        if (result != VmbErrorSuccess)
        {
            m_DeviceIDFeature.reset();
            return result;
        }
    }
    feature = m_DeviceIDFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetDeviceModelName (std::string & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetDeviceModelNameFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetDeviceModelNameFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_DeviceModelNameFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("DeviceModelName", m_DeviceModelNameFeature);
        if (result != VmbErrorSuccess)
        {
            m_DeviceModelNameFeature.reset();
            return result;
        }
    }
    feature = m_DeviceModelNameFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetDevicePartNumber (std::string & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetDevicePartNumberFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetDevicePartNumberFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_DevicePartNumberFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("DevicePartNumber", m_DevicePartNumberFeature);
        if (result != VmbErrorSuccess)
        {
            m_DevicePartNumberFeature.reset();
            return result;
        }
    }
    feature = m_DevicePartNumberFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetDeviceScanType (DeviceScanTypeEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetDeviceScanTypeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (DeviceScanTypeEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::GetDeviceScanTypeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_DeviceScanTypeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("DeviceScanType", m_DeviceScanTypeFeature);
        if (result != VmbErrorSuccess)
        {
            m_DeviceScanTypeFeature.reset();
            return result;
        }
    }
    feature = m_DeviceScanTypeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetDeviceVendorName (std::string & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetDeviceVendorNameFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetDeviceVendorNameFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_DeviceVendorNameFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("DeviceVendorName", m_DeviceVendorNameFeature);
        if (result != VmbErrorSuccess)
        {
            m_DeviceVendorNameFeature.reset();
            return result;
        }
    }
    feature = m_DeviceVendorNameFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetFirmwareVerBuild (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetFirmwareVerBuildFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetFirmwareVerBuildFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_FirmwareVerBuildFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("FirmwareVerBuild", m_FirmwareVerBuildFeature);
        if (result != VmbErrorSuccess)
        {
            m_FirmwareVerBuildFeature.reset();
            return result;
        }
    }
    feature = m_FirmwareVerBuildFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetFirmwareVerMajor (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetFirmwareVerMajorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetFirmwareVerMajorFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_FirmwareVerMajorFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("FirmwareVerMajor", m_FirmwareVerMajorFeature);
        if (result != VmbErrorSuccess)
        {
            m_FirmwareVerMajorFeature.reset();
            return result;
        }
    }
    feature = m_FirmwareVerMajorFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetFirmwareVerMinor (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetFirmwareVerMinorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetFirmwareVerMinorFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_FirmwareVerMinorFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("FirmwareVerMinor", m_FirmwareVerMinorFeature);
        if (result != VmbErrorSuccess)
        {
            m_FirmwareVerMinorFeature.reset();
            return result;
        }
    }
    feature = m_FirmwareVerMinorFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetSensorBits (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetSensorBitsFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetSensorBitsFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_SensorBitsFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("SensorBits", m_SensorBitsFeature);
        if (result != VmbErrorSuccess)
        {
            m_SensorBitsFeature.reset();
            return result;
        }
    }
    feature = m_SensorBitsFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetSensorType (SensorTypeEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetSensorTypeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (SensorTypeEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::GetSensorTypeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_SensorTypeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("SensorType", m_SensorTypeFeature);
        if (result != VmbErrorSuccess)
        {
            m_SensorTypeFeature.reset();
            return result;
        }
    }
    feature = m_SensorTypeFeature;
    return VmbErrorSuccess;
}


// Category /SavedUserSets
VmbErrorType MakoCamera::GetUserSetDefaultSelector (UserSetDefaultSelectorEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetUserSetDefaultSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (UserSetDefaultSelectorEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetUserSetDefaultSelector (UserSetDefaultSelectorEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetUserSetDefaultSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetUserSetDefaultSelectorFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_UserSetDefaultSelectorFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("UserSetDefaultSelector", m_UserSetDefaultSelectorFeature);
        if (result != VmbErrorSuccess)
        {
            m_UserSetDefaultSelectorFeature.reset();
            return result;
        }
    }
    feature = m_UserSetDefaultSelectorFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::UserSetLoad ()
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetUserSetLoadFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->RunCommand ();
    return result;
}
VmbErrorType MakoCamera::GetUserSetLoadFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_UserSetLoadFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("UserSetLoad", m_UserSetLoadFeature);
        if (result != VmbErrorSuccess)
        {
            m_UserSetLoadFeature.reset();
            return result;
        }
    }
    feature = m_UserSetLoadFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::UserSetSave ()
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetUserSetSaveFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->RunCommand ();
    return result;
}
VmbErrorType MakoCamera::GetUserSetSaveFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_UserSetSaveFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("UserSetSave", m_UserSetSaveFeature);
        if (result != VmbErrorSuccess)
        {
            m_UserSetSaveFeature.reset();
            return result;
        }
    }
    feature = m_UserSetSaveFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetUserSetSelector (UserSetSelectorEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetUserSetSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (UserSetSelectorEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetUserSetSelector (UserSetSelectorEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetUserSetSelectorFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetUserSetSelectorFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_UserSetSelectorFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("UserSetSelector", m_UserSetSelectorFeature);
        if (result != VmbErrorSuccess)
        {
            m_UserSetSelectorFeature.reset();
            return result;
        }
    }
    feature = m_UserSetSelectorFeature;
    return VmbErrorSuccess;
}


// Category /Stream/Info
VmbErrorType MakoCamera::GetGVSPFilterVersion (std::string & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPFilterVersionFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGVSPFilterVersionFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GVSPFilterVersionFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GVSPFilterVersion", m_GVSPFilterVersionFeature);
        if (result != VmbErrorSuccess)
        {
            m_GVSPFilterVersionFeature.reset();
            return result;
        }
    }
    feature = m_GVSPFilterVersionFeature;
    return VmbErrorSuccess;
}


// Category /Stream/Multicast
VmbErrorType MakoCamera::GetMulticastEnable (bool & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetMulticastEnableFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetMulticastEnable (bool value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetMulticastEnableFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetMulticastEnableFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_MulticastEnableFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("MulticastEnable", m_MulticastEnableFeature);
        if (result != VmbErrorSuccess)
        {
            m_MulticastEnableFeature.reset();
            return result;
        }
    }
    feature = m_MulticastEnableFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetMulticastIPAddress (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetMulticastIPAddressFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetMulticastIPAddress (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetMulticastIPAddressFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetMulticastIPAddressFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_MulticastIPAddressFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("MulticastIPAddress", m_MulticastIPAddressFeature);
        if (result != VmbErrorSuccess)
        {
            m_MulticastIPAddressFeature.reset();
            return result;
        }
    }
    feature = m_MulticastIPAddressFeature;
    return VmbErrorSuccess;
}


// Category /Stream/Settings
VmbErrorType MakoCamera::GVSPAdjustPacketSize ()
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPAdjustPacketSizeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->RunCommand ();
    return result;
}
VmbErrorType MakoCamera::GetGVSPAdjustPacketSizeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GVSPAdjustPacketSizeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GVSPAdjustPacketSize", m_GVSPAdjustPacketSizeFeature);
        if (result != VmbErrorSuccess)
        {
            m_GVSPAdjustPacketSizeFeature.reset();
            return result;
        }
    }
    feature = m_GVSPAdjustPacketSizeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGVSPBurstSize (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPBurstSizeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetGVSPBurstSize (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPBurstSizeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGVSPBurstSizeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GVSPBurstSizeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GVSPBurstSize", m_GVSPBurstSizeFeature);
        if (result != VmbErrorSuccess)
        {
            m_GVSPBurstSizeFeature.reset();
            return result;
        }
    }
    feature = m_GVSPBurstSizeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGVSPDriver (GVSPDriverEnum & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPDriverFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    VmbInt64_t nValue;
    result = pFeature->GetValue (nValue);
    value = (GVSPDriverEnum) nValue;
    return result;
}
VmbErrorType MakoCamera::SetGVSPDriver (GVSPDriverEnum value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPDriverFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGVSPDriverFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GVSPDriverFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GVSPDriver", m_GVSPDriverFeature);
        if (result != VmbErrorSuccess)
        {
            m_GVSPDriverFeature.reset();
            return result;
        }
    }
    feature = m_GVSPDriverFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGVSPHostReceiveBuffers (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPHostReceiveBuffersFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetGVSPHostReceiveBuffers (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPHostReceiveBuffersFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGVSPHostReceiveBuffersFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GVSPHostReceiveBuffersFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GVSPHostReceiveBuffers", m_GVSPHostReceiveBuffersFeature);
        if (result != VmbErrorSuccess)
        {
            m_GVSPHostReceiveBuffersFeature.reset();
            return result;
        }
    }
    feature = m_GVSPHostReceiveBuffersFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGVSPMaxLookBack (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPMaxLookBackFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetGVSPMaxLookBack (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPMaxLookBackFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGVSPMaxLookBackFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GVSPMaxLookBackFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GVSPMaxLookBack", m_GVSPMaxLookBackFeature);
        if (result != VmbErrorSuccess)
        {
            m_GVSPMaxLookBackFeature.reset();
            return result;
        }
    }
    feature = m_GVSPMaxLookBackFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGVSPMaxRequests (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPMaxRequestsFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetGVSPMaxRequests (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPMaxRequestsFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGVSPMaxRequestsFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GVSPMaxRequestsFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GVSPMaxRequests", m_GVSPMaxRequestsFeature);
        if (result != VmbErrorSuccess)
        {
            m_GVSPMaxRequestsFeature.reset();
            return result;
        }
    }
    feature = m_GVSPMaxRequestsFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGVSPMaxWaitSize (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPMaxWaitSizeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetGVSPMaxWaitSize (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPMaxWaitSizeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGVSPMaxWaitSizeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GVSPMaxWaitSizeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GVSPMaxWaitSize", m_GVSPMaxWaitSizeFeature);
        if (result != VmbErrorSuccess)
        {
            m_GVSPMaxWaitSizeFeature.reset();
            return result;
        }
    }
    feature = m_GVSPMaxWaitSizeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGVSPMissingSize (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPMissingSizeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetGVSPMissingSize (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPMissingSizeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGVSPMissingSizeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GVSPMissingSizeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GVSPMissingSize", m_GVSPMissingSizeFeature);
        if (result != VmbErrorSuccess)
        {
            m_GVSPMissingSizeFeature.reset();
            return result;
        }
    }
    feature = m_GVSPMissingSizeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGVSPPacketSize (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPPacketSizeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetGVSPPacketSize (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPPacketSizeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGVSPPacketSizeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GVSPPacketSizeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GVSPPacketSize", m_GVSPPacketSizeFeature);
        if (result != VmbErrorSuccess)
        {
            m_GVSPPacketSizeFeature.reset();
            return result;
        }
    }
    feature = m_GVSPPacketSizeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGVSPTiltingSize (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPTiltingSizeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetGVSPTiltingSize (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPTiltingSizeFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGVSPTiltingSizeFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GVSPTiltingSizeFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GVSPTiltingSize", m_GVSPTiltingSizeFeature);
        if (result != VmbErrorSuccess)
        {
            m_GVSPTiltingSizeFeature.reset();
            return result;
        }
    }
    feature = m_GVSPTiltingSizeFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetGVSPTimeout (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPTimeoutFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::SetGVSPTimeout (VmbInt64_t value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetGVSPTimeoutFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->SetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetGVSPTimeoutFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_GVSPTimeoutFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("GVSPTimeout", m_GVSPTimeoutFeature);
        if (result != VmbErrorSuccess)
        {
            m_GVSPTimeoutFeature.reset();
            return result;
        }
    }
    feature = m_GVSPTimeoutFeature;
    return VmbErrorSuccess;
}


// Category /Stream/Statistics
VmbErrorType MakoCamera::GetStatFrameDelivered (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStatFrameDeliveredFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetStatFrameDeliveredFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_StatFrameDeliveredFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("StatFrameDelivered", m_StatFrameDeliveredFeature);
        if (result != VmbErrorSuccess)
        {
            m_StatFrameDeliveredFeature.reset();
            return result;
        }
    }
    feature = m_StatFrameDeliveredFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetStatFrameDropped (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStatFrameDroppedFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetStatFrameDroppedFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_StatFrameDroppedFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("StatFrameDropped", m_StatFrameDroppedFeature);
        if (result != VmbErrorSuccess)
        {
            m_StatFrameDroppedFeature.reset();
            return result;
        }
    }
    feature = m_StatFrameDroppedFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetStatFrameRate (double & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStatFrameRateFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetStatFrameRateFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_StatFrameRateFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("StatFrameRate", m_StatFrameRateFeature);
        if (result != VmbErrorSuccess)
        {
            m_StatFrameRateFeature.reset();
            return result;
        }
    }
    feature = m_StatFrameRateFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetStatFrameRescued (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStatFrameRescuedFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetStatFrameRescuedFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_StatFrameRescuedFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("StatFrameRescued", m_StatFrameRescuedFeature);
        if (result != VmbErrorSuccess)
        {
            m_StatFrameRescuedFeature.reset();
            return result;
        }
    }
    feature = m_StatFrameRescuedFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetStatFrameShoved (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStatFrameShovedFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetStatFrameShovedFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_StatFrameShovedFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("StatFrameShoved", m_StatFrameShovedFeature);
        if (result != VmbErrorSuccess)
        {
            m_StatFrameShovedFeature.reset();
            return result;
        }
    }
    feature = m_StatFrameShovedFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetStatFrameUnderrun (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStatFrameUnderrunFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetStatFrameUnderrunFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_StatFrameUnderrunFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("StatFrameUnderrun", m_StatFrameUnderrunFeature);
        if (result != VmbErrorSuccess)
        {
            m_StatFrameUnderrunFeature.reset();
            return result;
        }
    }
    feature = m_StatFrameUnderrunFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetStatLocalRate (double & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStatLocalRateFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetStatLocalRateFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_StatLocalRateFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("StatLocalRate", m_StatLocalRateFeature);
        if (result != VmbErrorSuccess)
        {
            m_StatLocalRateFeature.reset();
            return result;
        }
    }
    feature = m_StatLocalRateFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetStatPacketErrors (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStatPacketErrorsFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetStatPacketErrorsFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_StatPacketErrorsFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("StatPacketErrors", m_StatPacketErrorsFeature);
        if (result != VmbErrorSuccess)
        {
            m_StatPacketErrorsFeature.reset();
            return result;
        }
    }
    feature = m_StatPacketErrorsFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetStatPacketMissed (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStatPacketMissedFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetStatPacketMissedFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_StatPacketMissedFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("StatPacketMissed", m_StatPacketMissedFeature);
        if (result != VmbErrorSuccess)
        {
            m_StatPacketMissedFeature.reset();
            return result;
        }
    }
    feature = m_StatPacketMissedFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetStatPacketReceived (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStatPacketReceivedFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetStatPacketReceivedFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_StatPacketReceivedFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("StatPacketReceived", m_StatPacketReceivedFeature);
        if (result != VmbErrorSuccess)
        {
            m_StatPacketReceivedFeature.reset();
            return result;
        }
    }
    feature = m_StatPacketReceivedFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetStatPacketRequested (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStatPacketRequestedFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetStatPacketRequestedFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_StatPacketRequestedFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("StatPacketRequested", m_StatPacketRequestedFeature);
        if (result != VmbErrorSuccess)
        {
            m_StatPacketRequestedFeature.reset();
            return result;
        }
    }
    feature = m_StatPacketRequestedFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetStatPacketResent (VmbInt64_t & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStatPacketResentFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetStatPacketResentFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_StatPacketResentFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("StatPacketResent", m_StatPacketResentFeature);
        if (result != VmbErrorSuccess)
        {
            m_StatPacketResentFeature.reset();
            return result;
        }
    }
    feature = m_StatPacketResentFeature;
    return VmbErrorSuccess;
}

VmbErrorType MakoCamera::GetStatTimeElapsed (double & value)
{
    VmbErrorType result;
    AVT::VmbAPI::FeaturePtr pFeature;
    result = GetStatTimeElapsedFeature (pFeature);
    if (result != VmbErrorSuccess)
        return result;
    result = pFeature->GetValue (value);
    return result;
}
VmbErrorType MakoCamera::GetStatTimeElapsedFeature (AVT::VmbAPI::FeaturePtr & feature)
{
    if (m_StatTimeElapsedFeature.get() == NULL)
    {
        VmbErrorType result;
        result = GetFeatureByName ("StatTimeElapsed", m_StatTimeElapsedFeature);
        if (result != VmbErrorSuccess)
        {
            m_StatTimeElapsedFeature.reset();
            return result;
        }
    }
    feature = m_StatTimeElapsedFeature;
    return VmbErrorSuccess;
}

