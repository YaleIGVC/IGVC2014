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

#ifndef MAKOCAMERA_H
#define MAKOCAMERA_H

#include "../VimbaCPP/Include/VimbaCPP.h"

class MakoCamera : public AVT::VmbAPI::Camera
{
    public:
        typedef SP_DECL( MakoCamera ) Ptr;

        typedef enum _AcquisitionModeEnum
        {
            AcquisitionMode_Continuous = 1,
            AcquisitionMode_SingleFrame = 2,
            AcquisitionMode_MultiFrame = 3,
            AcquisitionMode_Recorder = 4
        } AcquisitionModeEnum;

        typedef enum _BalanceRatioSelectorEnum
        {
            BalanceRatioSelector_Red = 0,
            BalanceRatioSelector_Blue = 1
        } BalanceRatioSelectorEnum;

        typedef enum _BalanceWhiteAutoEnum
        {
            BalanceWhiteAuto_Off = 1,
            BalanceWhiteAuto_Continuous = 2,
            BalanceWhiteAuto_Once = 3
        } BalanceWhiteAutoEnum;

        typedef enum _BandwidthControlModeEnum
        {
            BandwidthControlMode_StreamBytesPerSecond = 0,
            BandwidthControlMode_SCPD = 1,
            BandwidthControlMode_Both = 2
        } BandwidthControlModeEnum;

        typedef enum _BlackLevelSelectorEnum
        {
            BlackLevelSelector_All = 0
        } BlackLevelSelectorEnum;

        typedef enum _ColorTransformationModeEnum
        {
            ColorTransformationMode_Off = 0,
            ColorTransformationMode_Manual = 1,
            ColorTransformationMode_Temp6500K = 2
        } ColorTransformationModeEnum;

        typedef enum _ColorTransformationSelectorEnum
        {
            ColorTransformationSelector_RGBtoRGB = 0
        } ColorTransformationSelectorEnum;

        typedef enum _ColorTransformationValueSelectorEnum
        {
            ColorTransformationValueSelector_Gain00 = 0,
            ColorTransformationValueSelector_Gain01 = 1,
            ColorTransformationValueSelector_Gain02 = 2,
            ColorTransformationValueSelector_Gain10 = 3,
            ColorTransformationValueSelector_Gain11 = 4,
            ColorTransformationValueSelector_Gain12 = 5,
            ColorTransformationValueSelector_Gain20 = 6,
            ColorTransformationValueSelector_Gain21 = 7,
            ColorTransformationValueSelector_Gain22 = 8
        } ColorTransformationValueSelectorEnum;

        typedef enum _DefectMaskColumnEnableEnum
        {
            DefectMaskColumnEnable_Disabled = 0,
            DefectMaskColumnEnable_Enabled = 1
        } DefectMaskColumnEnableEnum;

        typedef enum _DeviceScanTypeEnum
        {
            DeviceScanType_Areascan = 0
        } DeviceScanTypeEnum;

        typedef enum _DeviceTemperatureSelectorEnum
        {
            DeviceTemperatureSelector_Sensor = 0,
            DeviceTemperatureSelector_Main = 1,
            DeviceTemperatureSelector_Power = 2
        } DeviceTemperatureSelectorEnum;

        typedef enum _EventNotificationEnum
        {
            EventNotification_Off = 0,
            EventNotification_On = 1
        } EventNotificationEnum;

        typedef enum _EventSelectorEnum
        {
            EventSelector_AcquisitionStart = 40000,
            EventSelector_AcquisitionEnd = 40001,
            EventSelector_FrameTrigger = 40002,
            EventSelector_ExposureEnd = 40003,
            EventSelector_AcquisitionRecordTrigger = 40004,
            EventSelector_PtpSyncLost = 40005,
            EventSelector_PtpSyncLocked = 40006,
            EventSelector_Line1RisingEdge = 40010,
            EventSelector_Line1FallingEdge = 40011,
            EventSelector_Line2RisingEdge = 40012,
            EventSelector_Line2FallingEdge = 40013,
            EventSelector_Line3RisingEdge = 40014,
            EventSelector_Line3FallingEdge = 40015,
            EventSelector_Line4RisingEdge = 40016,
            EventSelector_Line4FallingEdge = 40017,
            EventSelector_FrameTriggerReady = 40018
        } EventSelectorEnum;

        typedef enum _ExposureAutoEnum
        {
            ExposureAuto_Off = 1,
            ExposureAuto_Continuous = 2,
            ExposureAuto_Once = 3,
            ExposureAuto_other = 4
        } ExposureAutoEnum;

        typedef enum _ExposureAutoAlgEnum
        {
            ExposureAutoAlg_Mean = 0,
            ExposureAutoAlg_FitRange = 1
        } ExposureAutoAlgEnum;

        typedef enum _ExposureModeEnum
        {
            ExposureMode_Timed = 1
        } ExposureModeEnum;

        typedef enum _GVSPDriverEnum
        {
            GVSPDriver_Socket = 0,
            GVSPDriver_Filter = 1
        } GVSPDriverEnum;

        typedef enum _GainAutoEnum
        {
            GainAuto_Off = 1,
            GainAuto_Continuous = 2,
            GainAuto_Once = 3
        } GainAutoEnum;

        typedef enum _GainSelectorEnum
        {
            GainSelector_All = 0
        } GainSelectorEnum;

        typedef enum _GevIPConfigurationModeEnum
        {
            GevIPConfigurationMode_LLA = 4,
            GevIPConfigurationMode_Persistent = 5,
            GevIPConfigurationMode_DHCP = 6
        } GevIPConfigurationModeEnum;

        typedef enum _LUTModeEnum
        {
            LUTMode_Luminance = 0,
            LUTMode_Red = 1,
            LUTMode_Green = 2,
            LUTMode_Blue = 3
        } LUTModeEnum;

        typedef enum _LUTSelectorEnum
        {
            LUTSelector_LUT1 = 0,
            LUTSelector_LUT2 = 1,
            LUTSelector_LUT3 = 2,
            LUTSelector_LUT4 = 3,
            LUTSelector_LUT5 = 4
        } LUTSelectorEnum;

        typedef enum _PixelFormatEnum
        {
            PixelFormat_Mono8 = 17301505,
            PixelFormat_BayerGR8 = 17301512,
            PixelFormat_BayerRG8 = 17301513,
            PixelFormat_BayerGB8 = 17301514,
            PixelFormat_BayerBG8 = 17301515,
            PixelFormat_Mono12Packed = 17563654,
            PixelFormat_BayerGR12Packed = 17563690,
            PixelFormat_BayerRG12Packed = 17563691,
            PixelFormat_BayerGB12Packed = 17563692,
            PixelFormat_Mono10 = 17825795,
            PixelFormat_Mono12 = 17825797,
            PixelFormat_BayerBG10 = 17825807,
            PixelFormat_BayerGR12 = 17825808,
            PixelFormat_BayerRG12 = 17825809,
            PixelFormat_BayerGB12 = 17825810,
            PixelFormat_Mono14 = 17825829,
            PixelFormat_YUV411Packed = 34340894,
            PixelFormat_YUV422Packed = 34603039,
            PixelFormat_RGB8Packed = 35127316,
            PixelFormat_BGR8Packed = 35127317,
            PixelFormat_YUV444Packed = 35127328,
            PixelFormat_RGBA8Packed = 35651606,
            PixelFormat_BGRA8Packed = 35651607,
            PixelFormat_RGB10Packed = 36700184,
            PixelFormat_RGB12Packed = 36700186
        } PixelFormatEnum;

        typedef enum _SensorTypeEnum
        {
            SensorType_Mono = 0,
            SensorType_Bayer = 1
        } SensorTypeEnum;

        typedef enum _StreamHoldEnableEnum
        {
            StreamHoldEnable_Off = 0,
            StreamHoldEnable_On = 1
        } StreamHoldEnableEnum;

        typedef enum _StrobeDurationModeEnum
        {
            StrobeDurationMode_Source = 0,
            StrobeDurationMode_Controlled = 1
        } StrobeDurationModeEnum;

        typedef enum _StrobeSourceEnum
        {
            StrobeSource_AcquisitionTriggerReady = 1,
            StrobeSource_FrameTriggerReady = 2,
            StrobeSource_FrameTrigger = 3,
            StrobeSource_Exposing = 4,
            StrobeSource_FrameReadout = 5,
            StrobeSource_Acquiring = 7,
            StrobeSource_LineIn1 = 8,
            StrobeSource_LineIn2 = 9
        } StrobeSourceEnum;

        typedef enum _SyncInSelectorEnum
        {
            SyncInSelector_SyncIn1 = 0,
            SyncInSelector_SyncIn2 = 1,
            SyncInSelector_SyncIn3 = 2,
            SyncInSelector_SyncIn4 = 3
        } SyncInSelectorEnum;

        typedef enum _SyncOutPolarityEnum
        {
            SyncOutPolarity_Normal = 0,
            SyncOutPolarity_Invert = 1
        } SyncOutPolarityEnum;

        typedef enum _SyncOutSelectorEnum
        {
            SyncOutSelector_SyncOut1 = 0,
            SyncOutSelector_SyncOut2 = 1,
            SyncOutSelector_SyncOut3 = 2,
            SyncOutSelector_SyncOut4 = 3
        } SyncOutSelectorEnum;

        typedef enum _SyncOutSourceEnum
        {
            SyncOutSource_GPO = 0,
            SyncOutSource_AcquisitionTriggerReady = 1,
            SyncOutSource_FrameTriggerReady = 2,
            SyncOutSource_Exposing = 4,
            SyncOutSource_FrameReadout = 5,
            SyncOutSource_Imaging = 6,
            SyncOutSource_Acquiring = 7,
            SyncOutSource_LineIn1 = 8,
            SyncOutSource_LineIn2 = 9,
            SyncOutSource_Strobe1 = 12
        } SyncOutSourceEnum;

        typedef enum _TriggerActivationEnum
        {
            TriggerActivation_RisingEdge = 0,
            TriggerActivation_FallingEdge = 1,
            TriggerActivation_AnyEdge = 2,
            TriggerActivation_LevelHigh = 3,
            TriggerActivation_LevelLow = 4
        } TriggerActivationEnum;

        typedef enum _TriggerModeEnum
        {
            TriggerMode_Off = 0,
            TriggerMode_On = 1
        } TriggerModeEnum;

        typedef enum _TriggerOverlapEnum
        {
            TriggerOverlap_Off = 0,
            TriggerOverlap_ReadOut = 1,
            TriggerOverlap_PreviousFrame = 2
        } TriggerOverlapEnum;

        typedef enum _TriggerSelectorEnum
        {
            TriggerSelector_FrameStart = 0,
            TriggerSelector_AcquisitionStart = 3,
            TriggerSelector_AcquisitionEnd = 4,
            TriggerSelector_AcquisitionRecord = 6
        } TriggerSelectorEnum;

        typedef enum _TriggerSourceEnum
        {
            TriggerSource_Freerun = 0,
            TriggerSource_Line1 = 1,
            TriggerSource_Line2 = 2,
            TriggerSource_Line3 = 3,
            TriggerSource_Line4 = 4,
            TriggerSource_FixedRate = 5,
            TriggerSource_Software = 6
        } TriggerSourceEnum;

        typedef enum _UserSetDefaultSelectorEnum
        {
            UserSetDefaultSelector_Default = 0,
            UserSetDefaultSelector_UserSet1 = 1,
            UserSetDefaultSelector_UserSet2 = 2,
            UserSetDefaultSelector_UserSet3 = 3,
            UserSetDefaultSelector_UserSet4 = 4,
            UserSetDefaultSelector_UserSet5 = 5
        } UserSetDefaultSelectorEnum;

        typedef enum _UserSetSelectorEnum
        {
            UserSetSelector_Default = 0,
            UserSetSelector_UserSet1 = 1,
            UserSetSelector_UserSet2 = 2,
            UserSetSelector_UserSet3 = 3,
            UserSetSelector_UserSet4 = 4,
            UserSetSelector_UserSet5 = 5
        } UserSetSelectorEnum;

        
        MakoCamera (
            const char         *pCameraID,
            const char         *pCameraName,
            const char         *pCameraModel,
            const char         *pCameraSerialNumber,
            const char         *pInterfaceID,
            VmbInterfaceType    interfaceType,
            const char         *pInterfaceName,
            const char         *pInterfaceSerialNumber,
            VmbAccessModeType   interfacePermittedAccess );


        // Category /Acquisition
        VmbErrorType AcquisitionAbort ();
        VmbErrorType GetAcquisitionAbortFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetAcquisitionFrameCount (VmbInt64_t & value);
        VmbErrorType SetAcquisitionFrameCount (VmbInt64_t value);
        VmbErrorType GetAcquisitionFrameCountFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetAcquisitionFrameRateAbs (double & value);
        VmbErrorType SetAcquisitionFrameRateAbs (double value);
        VmbErrorType GetAcquisitionFrameRateAbsFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetAcquisitionFrameRateLimit (double & value);
        VmbErrorType GetAcquisitionFrameRateLimitFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetAcquisitionMode (AcquisitionModeEnum & value);
        VmbErrorType SetAcquisitionMode (AcquisitionModeEnum value);
        VmbErrorType GetAcquisitionModeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType AcquisitionStart ();
        VmbErrorType GetAcquisitionStartFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType AcquisitionStop ();
        VmbErrorType GetAcquisitionStopFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetRecorderPreEventCount (VmbInt64_t & value);
        VmbErrorType SetRecorderPreEventCount (VmbInt64_t value);
        VmbErrorType GetRecorderPreEventCountFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /Acquisition/Trigger
        VmbErrorType GetTriggerActivation (TriggerActivationEnum & value);
        VmbErrorType SetTriggerActivation (TriggerActivationEnum value);
        VmbErrorType GetTriggerActivationFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetTriggerDelayAbs (double & value);
        VmbErrorType SetTriggerDelayAbs (double value);
        VmbErrorType GetTriggerDelayAbsFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetTriggerMode (TriggerModeEnum & value);
        VmbErrorType SetTriggerMode (TriggerModeEnum value);
        VmbErrorType GetTriggerModeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetTriggerOverlap (TriggerOverlapEnum & value);
        VmbErrorType SetTriggerOverlap (TriggerOverlapEnum value);
        VmbErrorType GetTriggerOverlapFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetTriggerSelector (TriggerSelectorEnum & value);
        VmbErrorType SetTriggerSelector (TriggerSelectorEnum value);
        VmbErrorType GetTriggerSelectorFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType TriggerSoftware ();
        VmbErrorType GetTriggerSoftwareFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetTriggerSource (TriggerSourceEnum & value);
        VmbErrorType SetTriggerSource (TriggerSourceEnum value);
        VmbErrorType GetTriggerSourceFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /Controls
        VmbErrorType GetGamma (double & value);
        VmbErrorType SetGamma (double value);
        VmbErrorType GetGammaFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetHue (double & value);
        VmbErrorType SetHue (double value);
        VmbErrorType GetHueFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetSaturation (double & value);
        VmbErrorType SetSaturation (double value);
        VmbErrorType GetSaturationFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /Controls/BlackLevelControl
        VmbErrorType GetBlackLevel (double & value);
        VmbErrorType SetBlackLevel (double value);
        VmbErrorType GetBlackLevelFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetBlackLevelSelector (BlackLevelSelectorEnum & value);
        VmbErrorType SetBlackLevelSelector (BlackLevelSelectorEnum value);
        VmbErrorType GetBlackLevelSelectorFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /Controls/ColorTransformationControl
        VmbErrorType GetColorTransformationMode (ColorTransformationModeEnum & value);
        VmbErrorType SetColorTransformationMode (ColorTransformationModeEnum value);
        VmbErrorType GetColorTransformationModeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetColorTransformationSelector (ColorTransformationSelectorEnum & value);
        VmbErrorType SetColorTransformationSelector (ColorTransformationSelectorEnum value);
        VmbErrorType GetColorTransformationSelectorFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetColorTransformationValue (double & value);
        VmbErrorType SetColorTransformationValue (double value);
        VmbErrorType GetColorTransformationValueFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetColorTransformationValueSelector (ColorTransformationValueSelectorEnum & value);
        VmbErrorType SetColorTransformationValueSelector (ColorTransformationValueSelectorEnum value);
        VmbErrorType GetColorTransformationValueSelectorFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /Controls/DSPSubregion
        VmbErrorType GetDSPSubregionBottom (VmbInt64_t & value);
        VmbErrorType SetDSPSubregionBottom (VmbInt64_t value);
        VmbErrorType GetDSPSubregionBottomFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetDSPSubregionLeft (VmbInt64_t & value);
        VmbErrorType SetDSPSubregionLeft (VmbInt64_t value);
        VmbErrorType GetDSPSubregionLeftFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetDSPSubregionRight (VmbInt64_t & value);
        VmbErrorType SetDSPSubregionRight (VmbInt64_t value);
        VmbErrorType GetDSPSubregionRightFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetDSPSubregionTop (VmbInt64_t & value);
        VmbErrorType SetDSPSubregionTop (VmbInt64_t value);
        VmbErrorType GetDSPSubregionTopFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /Controls/DefectMask
        VmbErrorType GetDefectMaskColumnEnable (DefectMaskColumnEnableEnum & value);
        VmbErrorType SetDefectMaskColumnEnable (DefectMaskColumnEnableEnum value);
        VmbErrorType GetDefectMaskColumnEnableFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /Controls/Exposure
        VmbErrorType GetExposureAuto (ExposureAutoEnum & value);
        VmbErrorType SetExposureAuto (ExposureAutoEnum value);
        VmbErrorType GetExposureAutoFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetExposureMode (ExposureModeEnum & value);
        VmbErrorType SetExposureMode (ExposureModeEnum value);
        VmbErrorType GetExposureModeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetExposureTimeAbs (double & value);
        VmbErrorType SetExposureTimeAbs (double value);
        VmbErrorType GetExposureTimeAbsFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /Controls/Exposure/ExposureAutoControl
        VmbErrorType GetExposureAutoAdjustTol (VmbInt64_t & value);
        VmbErrorType SetExposureAutoAdjustTol (VmbInt64_t value);
        VmbErrorType GetExposureAutoAdjustTolFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetExposureAutoAlg (ExposureAutoAlgEnum & value);
        VmbErrorType SetExposureAutoAlg (ExposureAutoAlgEnum value);
        VmbErrorType GetExposureAutoAlgFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetExposureAutoMax (VmbInt64_t & value);
        VmbErrorType SetExposureAutoMax (VmbInt64_t value);
        VmbErrorType GetExposureAutoMaxFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetExposureAutoMin (VmbInt64_t & value);
        VmbErrorType SetExposureAutoMin (VmbInt64_t value);
        VmbErrorType GetExposureAutoMinFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetExposureAutoOutliers (VmbInt64_t & value);
        VmbErrorType SetExposureAutoOutliers (VmbInt64_t value);
        VmbErrorType GetExposureAutoOutliersFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetExposureAutoRate (VmbInt64_t & value);
        VmbErrorType SetExposureAutoRate (VmbInt64_t value);
        VmbErrorType GetExposureAutoRateFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetExposureAutoTarget (VmbInt64_t & value);
        VmbErrorType SetExposureAutoTarget (VmbInt64_t value);
        VmbErrorType GetExposureAutoTargetFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /Controls/GainControl
        VmbErrorType GetGain (double & value);
        VmbErrorType SetGain (double value);
        VmbErrorType GetGainFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGainAuto (GainAutoEnum & value);
        VmbErrorType SetGainAuto (GainAutoEnum value);
        VmbErrorType GetGainAutoFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGainSelector (GainSelectorEnum & value);
        VmbErrorType SetGainSelector (GainSelectorEnum value);
        VmbErrorType GetGainSelectorFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /Controls/GainControl/GainAutoControl
        VmbErrorType GetGainAutoAdjustTol (VmbInt64_t & value);
        VmbErrorType SetGainAutoAdjustTol (VmbInt64_t value);
        VmbErrorType GetGainAutoAdjustTolFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGainAutoMax (double & value);
        VmbErrorType SetGainAutoMax (double value);
        VmbErrorType GetGainAutoMaxFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGainAutoMin (double & value);
        VmbErrorType SetGainAutoMin (double value);
        VmbErrorType GetGainAutoMinFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGainAutoOutliers (VmbInt64_t & value);
        VmbErrorType SetGainAutoOutliers (VmbInt64_t value);
        VmbErrorType GetGainAutoOutliersFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGainAutoRate (VmbInt64_t & value);
        VmbErrorType SetGainAutoRate (VmbInt64_t value);
        VmbErrorType GetGainAutoRateFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGainAutoTarget (VmbInt64_t & value);
        VmbErrorType SetGainAutoTarget (VmbInt64_t value);
        VmbErrorType GetGainAutoTargetFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /Controls/LUTControl
        VmbErrorType GetLUTEnable (bool & value);
        VmbErrorType SetLUTEnable (bool value);
        VmbErrorType GetLUTEnableFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetLUTIndex (VmbInt64_t & value);
        VmbErrorType SetLUTIndex (VmbInt64_t value);
        VmbErrorType GetLUTIndexFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType LUTLoadAll ();
        VmbErrorType GetLUTLoadAllFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetLUTMode (LUTModeEnum & value);
        VmbErrorType SetLUTMode (LUTModeEnum value);
        VmbErrorType GetLUTModeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType LUTSaveAll ();
        VmbErrorType GetLUTSaveAllFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetLUTSelector (LUTSelectorEnum & value);
        VmbErrorType SetLUTSelector (LUTSelectorEnum value);
        VmbErrorType GetLUTSelectorFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetLUTValue (VmbInt64_t & value);
        VmbErrorType SetLUTValue (VmbInt64_t value);
        VmbErrorType GetLUTValueFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /Controls/LUTControl/LUTInfo
        VmbErrorType GetLUTAddress (VmbInt64_t & value);
        VmbErrorType GetLUTAddressFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetLUTBitDepthIn (VmbInt64_t & value);
        VmbErrorType GetLUTBitDepthInFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetLUTBitDepthOut (VmbInt64_t & value);
        VmbErrorType GetLUTBitDepthOutFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetLUTSizeBytes (VmbInt64_t & value);
        VmbErrorType GetLUTSizeBytesFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /Controls/Whitebalance
        VmbErrorType GetBalanceRatioAbs (double & value);
        VmbErrorType SetBalanceRatioAbs (double value);
        VmbErrorType GetBalanceRatioAbsFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetBalanceRatioSelector (BalanceRatioSelectorEnum & value);
        VmbErrorType SetBalanceRatioSelector (BalanceRatioSelectorEnum value);
        VmbErrorType GetBalanceRatioSelectorFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetBalanceWhiteAuto (BalanceWhiteAutoEnum & value);
        VmbErrorType SetBalanceWhiteAuto (BalanceWhiteAutoEnum value);
        VmbErrorType GetBalanceWhiteAutoFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /Controls/Whitebalance/BalanceWhiteAutoControl
        VmbErrorType GetBalanceWhiteAutoAdjustTol (VmbInt64_t & value);
        VmbErrorType SetBalanceWhiteAutoAdjustTol (VmbInt64_t value);
        VmbErrorType GetBalanceWhiteAutoAdjustTolFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetBalanceWhiteAutoRate (VmbInt64_t & value);
        VmbErrorType SetBalanceWhiteAutoRate (VmbInt64_t value);
        VmbErrorType GetBalanceWhiteAutoRateFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /DeviceStatus
        VmbErrorType GetDeviceTemperature (double & value);
        VmbErrorType GetDeviceTemperatureFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetDeviceTemperatureSelector (DeviceTemperatureSelectorEnum & value);
        VmbErrorType SetDeviceTemperatureSelector (DeviceTemperatureSelectorEnum value);
        VmbErrorType GetDeviceTemperatureSelectorFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /EventControl
        VmbErrorType GetEventNotification (EventNotificationEnum & value);
        VmbErrorType SetEventNotification (EventNotificationEnum value);
        VmbErrorType GetEventNotificationFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventSelector (EventSelectorEnum & value);
        VmbErrorType SetEventSelector (EventSelectorEnum value);
        VmbErrorType GetEventSelectorFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventsEnable1 (VmbInt64_t & value);
        VmbErrorType SetEventsEnable1 (VmbInt64_t value);
        VmbErrorType GetEventsEnable1Feature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /EventControl/EventData
        VmbErrorType GetEventOverflowTimestamp (VmbInt64_t & value);
        VmbErrorType GetEventOverflowTimestampFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventOverflowFrameID (VmbInt64_t & value);
        VmbErrorType GetEventOverflowFrameIDFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventLine1RisingEdgeTimestamp (VmbInt64_t & value);
        VmbErrorType GetEventLine1RisingEdgeTimestampFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventLine1RisingEdgeFrameID (VmbInt64_t & value);
        VmbErrorType GetEventLine1RisingEdgeFrameIDFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventLine1FallingEdgeTimestamp (VmbInt64_t & value);
        VmbErrorType GetEventLine1FallingEdgeTimestampFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventLine1FallingEdgeFrameID (VmbInt64_t & value);
        VmbErrorType GetEventLine1FallingEdgeFrameIDFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventFrameTriggerTimestamp (VmbInt64_t & value);
        VmbErrorType GetEventFrameTriggerTimestampFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventFrameTriggerReadyTimestamp (VmbInt64_t & value);
        VmbErrorType GetEventFrameTriggerReadyTimestampFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventFrameTriggerReadyFrameID (VmbInt64_t & value);
        VmbErrorType GetEventFrameTriggerReadyFrameIDFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventFrameTriggerFrameID (VmbInt64_t & value);
        VmbErrorType GetEventFrameTriggerFrameIDFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventExposureEndTimestamp (VmbInt64_t & value);
        VmbErrorType GetEventExposureEndTimestampFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventExposureEndFrameID (VmbInt64_t & value);
        VmbErrorType GetEventExposureEndFrameIDFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventErrorTimestamp (VmbInt64_t & value);
        VmbErrorType GetEventErrorTimestampFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventErrorFrameID (VmbInt64_t & value);
        VmbErrorType GetEventErrorFrameIDFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventAcquisitionStartTimestamp (VmbInt64_t & value);
        VmbErrorType GetEventAcquisitionStartTimestampFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventAcquisitionStartFrameID (VmbInt64_t & value);
        VmbErrorType GetEventAcquisitionStartFrameIDFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventAcquisitionRecordTriggerTimestamp (VmbInt64_t & value);
        VmbErrorType GetEventAcquisitionRecordTriggerTimestampFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventAcquisitionRecordTriggerFrameID (VmbInt64_t & value);
        VmbErrorType GetEventAcquisitionRecordTriggerFrameIDFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventAcquisitionEndTimestamp (VmbInt64_t & value);
        VmbErrorType GetEventAcquisitionEndTimestampFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventAcquisitionEndFrameID (VmbInt64_t & value);
        VmbErrorType GetEventAcquisitionEndFrameIDFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /EventControl/EventID
        VmbErrorType GetEventAcquisitionEnd (VmbInt64_t & value);
        VmbErrorType GetEventAcquisitionEndFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventAcquisitionRecordTrigger (VmbInt64_t & value);
        VmbErrorType GetEventAcquisitionRecordTriggerFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventAcquisitionStart (VmbInt64_t & value);
        VmbErrorType GetEventAcquisitionStartFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventError (VmbInt64_t & value);
        VmbErrorType GetEventErrorFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventExposureEnd (VmbInt64_t & value);
        VmbErrorType GetEventExposureEndFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventFrameTrigger (VmbInt64_t & value);
        VmbErrorType GetEventFrameTriggerFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventFrameTriggerReady (VmbInt64_t & value);
        VmbErrorType GetEventFrameTriggerReadyFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventLine1FallingEdge (VmbInt64_t & value);
        VmbErrorType GetEventLine1FallingEdgeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventLine1RisingEdge (VmbInt64_t & value);
        VmbErrorType GetEventLine1RisingEdgeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventLine2FallingEdge (VmbInt64_t & value);
        VmbErrorType GetEventLine2FallingEdgeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventLine2RisingEdge (VmbInt64_t & value);
        VmbErrorType GetEventLine2RisingEdgeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventLine3FallingEdge (VmbInt64_t & value);
        VmbErrorType GetEventLine3FallingEdgeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventLine3RisingEdge (VmbInt64_t & value);
        VmbErrorType GetEventLine3RisingEdgeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventLine4FallingEdge (VmbInt64_t & value);
        VmbErrorType GetEventLine4FallingEdgeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventLine4RisingEdge (VmbInt64_t & value);
        VmbErrorType GetEventLine4RisingEdgeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetEventOverflow (VmbInt64_t & value);
        VmbErrorType GetEventOverflowFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /GigE
        VmbErrorType GetBandwidthControlMode (BandwidthControlModeEnum & value);
        VmbErrorType SetBandwidthControlMode (BandwidthControlModeEnum value);
        VmbErrorType GetBandwidthControlModeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetChunkModeActive (bool & value);
        VmbErrorType SetChunkModeActive (bool value);
        VmbErrorType GetChunkModeActiveFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGevDeviceMACAddress (VmbInt64_t & value);
        VmbErrorType GetGevDeviceMACAddressFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGevSCPSPacketSize (VmbInt64_t & value);
        VmbErrorType SetGevSCPSPacketSize (VmbInt64_t value);
        VmbErrorType GetGevSCPSPacketSizeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetNonImagePayloadSize (VmbInt64_t & value);
        VmbErrorType GetNonImagePayloadSizeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetPayloadSize (VmbInt64_t & value);
        VmbErrorType GetPayloadSizeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetStreamBytesPerSecond (VmbInt64_t & value);
        VmbErrorType SetStreamBytesPerSecond (VmbInt64_t value);
        VmbErrorType GetStreamBytesPerSecondFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetStreamFrameRateConstrain (bool & value);
        VmbErrorType SetStreamFrameRateConstrain (bool value);
        VmbErrorType GetStreamFrameRateConstrainFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /GigE/Configuration
        VmbErrorType GetGevIPConfigurationMode (GevIPConfigurationModeEnum & value);
        VmbErrorType GetGevIPConfigurationModeFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /GigE/Current
        VmbErrorType GetGevCurrentDefaultGateway (VmbInt64_t & value);
        VmbErrorType GetGevCurrentDefaultGatewayFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGevCurrentIPAddress (VmbInt64_t & value);
        VmbErrorType GetGevCurrentIPAddressFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGevCurrentSubnetMask (VmbInt64_t & value);
        VmbErrorType GetGevCurrentSubnetMaskFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /GigE/GVCP
        VmbErrorType GetGVCPCmdRetries (VmbInt64_t & value);
        VmbErrorType SetGVCPCmdRetries (VmbInt64_t value);
        VmbErrorType GetGVCPCmdRetriesFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGVCPCmdTimeout (VmbInt64_t & value);
        VmbErrorType SetGVCPCmdTimeout (VmbInt64_t value);
        VmbErrorType GetGVCPCmdTimeoutFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGVCPHBInterval (VmbInt64_t & value);
        VmbErrorType SetGVCPHBInterval (VmbInt64_t value);
        VmbErrorType GetGVCPHBIntervalFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /GigE/Persistent
        VmbErrorType GetGevPersistentDefaultGateway (VmbInt64_t & value);
        VmbErrorType GetGevPersistentDefaultGatewayFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGevPersistentIPAddress (VmbInt64_t & value);
        VmbErrorType GetGevPersistentIPAddressFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGevPersistentSubnetMask (VmbInt64_t & value);
        VmbErrorType GetGevPersistentSubnetMaskFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /GigE/StreamHold
        VmbErrorType GetStreamHoldCapacity (VmbInt64_t & value);
        VmbErrorType GetStreamHoldCapacityFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetStreamHoldEnable (StreamHoldEnableEnum & value);
        VmbErrorType SetStreamHoldEnable (StreamHoldEnableEnum value);
        VmbErrorType GetStreamHoldEnableFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /GigE/Timestamp
        VmbErrorType GevTimestampControlLatch ();
        VmbErrorType GetGevTimestampControlLatchFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GevTimestampControlReset ();
        VmbErrorType GetGevTimestampControlResetFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGevTimestampTickFrequency (VmbInt64_t & value);
        VmbErrorType GetGevTimestampTickFrequencyFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGevTimestampValue (VmbInt64_t & value);
        VmbErrorType GetGevTimestampValueFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /IO/Strobe
        VmbErrorType GetStrobeDelay (VmbInt64_t & value);
        VmbErrorType SetStrobeDelay (VmbInt64_t value);
        VmbErrorType GetStrobeDelayFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetStrobeDuration (VmbInt64_t & value);
        VmbErrorType SetStrobeDuration (VmbInt64_t value);
        VmbErrorType GetStrobeDurationFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetStrobeDurationMode (StrobeDurationModeEnum & value);
        VmbErrorType SetStrobeDurationMode (StrobeDurationModeEnum value);
        VmbErrorType GetStrobeDurationModeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetStrobeSource (StrobeSourceEnum & value);
        VmbErrorType SetStrobeSource (StrobeSourceEnum value);
        VmbErrorType GetStrobeSourceFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /IO/SyncIn
        VmbErrorType GetSyncInGlitchFilter (VmbInt64_t & value);
        VmbErrorType SetSyncInGlitchFilter (VmbInt64_t value);
        VmbErrorType GetSyncInGlitchFilterFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetSyncInLevels (VmbInt64_t & value);
        VmbErrorType GetSyncInLevelsFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetSyncInSelector (SyncInSelectorEnum & value);
        VmbErrorType SetSyncInSelector (SyncInSelectorEnum value);
        VmbErrorType GetSyncInSelectorFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /IO/SyncOut
        VmbErrorType GetSyncOutLevels (VmbInt64_t & value);
        VmbErrorType SetSyncOutLevels (VmbInt64_t value);
        VmbErrorType GetSyncOutLevelsFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetSyncOutPolarity (SyncOutPolarityEnum & value);
        VmbErrorType SetSyncOutPolarity (SyncOutPolarityEnum value);
        VmbErrorType GetSyncOutPolarityFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetSyncOutSelector (SyncOutSelectorEnum & value);
        VmbErrorType SetSyncOutSelector (SyncOutSelectorEnum value);
        VmbErrorType GetSyncOutSelectorFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetSyncOutSource (SyncOutSourceEnum & value);
        VmbErrorType SetSyncOutSource (SyncOutSourceEnum value);
        VmbErrorType GetSyncOutSourceFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /ImageFormat
        VmbErrorType GetHeight (VmbInt64_t & value);
        VmbErrorType SetHeight (VmbInt64_t value);
        VmbErrorType GetHeightFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetHeightMax (VmbInt64_t & value);
        VmbErrorType GetHeightMaxFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetImageSize (VmbInt64_t & value);
        VmbErrorType GetImageSizeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetOffsetX (VmbInt64_t & value);
        VmbErrorType SetOffsetX (VmbInt64_t value);
        VmbErrorType GetOffsetXFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetOffsetY (VmbInt64_t & value);
        VmbErrorType SetOffsetY (VmbInt64_t value);
        VmbErrorType GetOffsetYFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetPixelFormat (PixelFormatEnum & value);
        VmbErrorType SetPixelFormat (PixelFormatEnum value);
        VmbErrorType GetPixelFormatFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetWidth (VmbInt64_t & value);
        VmbErrorType SetWidth (VmbInt64_t value);
        VmbErrorType GetWidthFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetWidthMax (VmbInt64_t & value);
        VmbErrorType GetWidthMaxFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /ImageMode
        VmbErrorType GetSensorHeight (VmbInt64_t & value);
        VmbErrorType GetSensorHeightFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetSensorWidth (VmbInt64_t & value);
        VmbErrorType GetSensorWidthFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /Info
        VmbErrorType GetDeviceFirmwareVersion (std::string & value);
        VmbErrorType GetDeviceFirmwareVersionFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetDeviceID (std::string & value);
        VmbErrorType GetDeviceIDFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetDeviceModelName (std::string & value);
        VmbErrorType GetDeviceModelNameFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetDevicePartNumber (std::string & value);
        VmbErrorType GetDevicePartNumberFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetDeviceScanType (DeviceScanTypeEnum & value);
        VmbErrorType GetDeviceScanTypeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetDeviceVendorName (std::string & value);
        VmbErrorType GetDeviceVendorNameFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetFirmwareVerBuild (VmbInt64_t & value);
        VmbErrorType GetFirmwareVerBuildFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetFirmwareVerMajor (VmbInt64_t & value);
        VmbErrorType GetFirmwareVerMajorFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetFirmwareVerMinor (VmbInt64_t & value);
        VmbErrorType GetFirmwareVerMinorFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetSensorBits (VmbInt64_t & value);
        VmbErrorType GetSensorBitsFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetSensorType (SensorTypeEnum & value);
        VmbErrorType GetSensorTypeFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /SavedUserSets
        VmbErrorType GetUserSetDefaultSelector (UserSetDefaultSelectorEnum & value);
        VmbErrorType SetUserSetDefaultSelector (UserSetDefaultSelectorEnum value);
        VmbErrorType GetUserSetDefaultSelectorFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType UserSetLoad ();
        VmbErrorType GetUserSetLoadFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType UserSetSave ();
        VmbErrorType GetUserSetSaveFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetUserSetSelector (UserSetSelectorEnum & value);
        VmbErrorType SetUserSetSelector (UserSetSelectorEnum value);
        VmbErrorType GetUserSetSelectorFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /Stream/Info
        VmbErrorType GetGVSPFilterVersion (std::string & value);
        VmbErrorType GetGVSPFilterVersionFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /Stream/Multicast
        VmbErrorType GetMulticastEnable (bool & value);
        VmbErrorType SetMulticastEnable (bool value);
        VmbErrorType GetMulticastEnableFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetMulticastIPAddress (VmbInt64_t & value);
        VmbErrorType SetMulticastIPAddress (VmbInt64_t value);
        VmbErrorType GetMulticastIPAddressFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /Stream/Settings
        VmbErrorType GVSPAdjustPacketSize ();
        VmbErrorType GetGVSPAdjustPacketSizeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGVSPBurstSize (VmbInt64_t & value);
        VmbErrorType SetGVSPBurstSize (VmbInt64_t value);
        VmbErrorType GetGVSPBurstSizeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGVSPDriver (GVSPDriverEnum & value);
        VmbErrorType SetGVSPDriver (GVSPDriverEnum value);
        VmbErrorType GetGVSPDriverFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGVSPHostReceiveBuffers (VmbInt64_t & value);
        VmbErrorType SetGVSPHostReceiveBuffers (VmbInt64_t value);
        VmbErrorType GetGVSPHostReceiveBuffersFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGVSPMaxLookBack (VmbInt64_t & value);
        VmbErrorType SetGVSPMaxLookBack (VmbInt64_t value);
        VmbErrorType GetGVSPMaxLookBackFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGVSPMaxRequests (VmbInt64_t & value);
        VmbErrorType SetGVSPMaxRequests (VmbInt64_t value);
        VmbErrorType GetGVSPMaxRequestsFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGVSPMaxWaitSize (VmbInt64_t & value);
        VmbErrorType SetGVSPMaxWaitSize (VmbInt64_t value);
        VmbErrorType GetGVSPMaxWaitSizeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGVSPMissingSize (VmbInt64_t & value);
        VmbErrorType SetGVSPMissingSize (VmbInt64_t value);
        VmbErrorType GetGVSPMissingSizeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGVSPPacketSize (VmbInt64_t & value);
        VmbErrorType SetGVSPPacketSize (VmbInt64_t value);
        VmbErrorType GetGVSPPacketSizeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGVSPTiltingSize (VmbInt64_t & value);
        VmbErrorType SetGVSPTiltingSize (VmbInt64_t value);
        VmbErrorType GetGVSPTiltingSizeFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetGVSPTimeout (VmbInt64_t & value);
        VmbErrorType SetGVSPTimeout (VmbInt64_t value);
        VmbErrorType GetGVSPTimeoutFeature (AVT::VmbAPI::FeaturePtr & feature);


        // Category /Stream/Statistics
        VmbErrorType GetStatFrameDelivered (VmbInt64_t & value);
        VmbErrorType GetStatFrameDeliveredFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetStatFrameDropped (VmbInt64_t & value);
        VmbErrorType GetStatFrameDroppedFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetStatFrameRate (double & value);
        VmbErrorType GetStatFrameRateFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetStatFrameRescued (VmbInt64_t & value);
        VmbErrorType GetStatFrameRescuedFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetStatFrameShoved (VmbInt64_t & value);
        VmbErrorType GetStatFrameShovedFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetStatFrameUnderrun (VmbInt64_t & value);
        VmbErrorType GetStatFrameUnderrunFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetStatLocalRate (double & value);
        VmbErrorType GetStatLocalRateFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetStatPacketErrors (VmbInt64_t & value);
        VmbErrorType GetStatPacketErrorsFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetStatPacketMissed (VmbInt64_t & value);
        VmbErrorType GetStatPacketMissedFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetStatPacketReceived (VmbInt64_t & value);
        VmbErrorType GetStatPacketReceivedFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetStatPacketRequested (VmbInt64_t & value);
        VmbErrorType GetStatPacketRequestedFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetStatPacketResent (VmbInt64_t & value);
        VmbErrorType GetStatPacketResentFeature (AVT::VmbAPI::FeaturePtr & feature);

        VmbErrorType GetStatTimeElapsed (double & value);
        VmbErrorType GetStatTimeElapsedFeature (AVT::VmbAPI::FeaturePtr & feature);


    private:
        AVT::VmbAPI::FeaturePtr m_AcquisitionAbortFeature;
        AVT::VmbAPI::FeaturePtr m_AcquisitionFrameCountFeature;
        AVT::VmbAPI::FeaturePtr m_AcquisitionFrameRateAbsFeature;
        AVT::VmbAPI::FeaturePtr m_AcquisitionFrameRateLimitFeature;
        AVT::VmbAPI::FeaturePtr m_AcquisitionModeFeature;
        AVT::VmbAPI::FeaturePtr m_AcquisitionStartFeature;
        AVT::VmbAPI::FeaturePtr m_AcquisitionStopFeature;
        AVT::VmbAPI::FeaturePtr m_RecorderPreEventCountFeature;
        AVT::VmbAPI::FeaturePtr m_TriggerActivationFeature;
        AVT::VmbAPI::FeaturePtr m_TriggerDelayAbsFeature;
        AVT::VmbAPI::FeaturePtr m_TriggerModeFeature;
        AVT::VmbAPI::FeaturePtr m_TriggerOverlapFeature;
        AVT::VmbAPI::FeaturePtr m_TriggerSelectorFeature;
        AVT::VmbAPI::FeaturePtr m_TriggerSoftwareFeature;
        AVT::VmbAPI::FeaturePtr m_TriggerSourceFeature;
        AVT::VmbAPI::FeaturePtr m_GammaFeature;
        AVT::VmbAPI::FeaturePtr m_HueFeature;
        AVT::VmbAPI::FeaturePtr m_SaturationFeature;
        AVT::VmbAPI::FeaturePtr m_BlackLevelFeature;
        AVT::VmbAPI::FeaturePtr m_BlackLevelSelectorFeature;
        AVT::VmbAPI::FeaturePtr m_ColorTransformationModeFeature;
        AVT::VmbAPI::FeaturePtr m_ColorTransformationSelectorFeature;
        AVT::VmbAPI::FeaturePtr m_ColorTransformationValueFeature;
        AVT::VmbAPI::FeaturePtr m_ColorTransformationValueSelectorFeature;
        AVT::VmbAPI::FeaturePtr m_DSPSubregionBottomFeature;
        AVT::VmbAPI::FeaturePtr m_DSPSubregionLeftFeature;
        AVT::VmbAPI::FeaturePtr m_DSPSubregionRightFeature;
        AVT::VmbAPI::FeaturePtr m_DSPSubregionTopFeature;
        AVT::VmbAPI::FeaturePtr m_DefectMaskColumnEnableFeature;
        AVT::VmbAPI::FeaturePtr m_ExposureAutoFeature;
        AVT::VmbAPI::FeaturePtr m_ExposureModeFeature;
        AVT::VmbAPI::FeaturePtr m_ExposureTimeAbsFeature;
        AVT::VmbAPI::FeaturePtr m_ExposureAutoAdjustTolFeature;
        AVT::VmbAPI::FeaturePtr m_ExposureAutoAlgFeature;
        AVT::VmbAPI::FeaturePtr m_ExposureAutoMaxFeature;
        AVT::VmbAPI::FeaturePtr m_ExposureAutoMinFeature;
        AVT::VmbAPI::FeaturePtr m_ExposureAutoOutliersFeature;
        AVT::VmbAPI::FeaturePtr m_ExposureAutoRateFeature;
        AVT::VmbAPI::FeaturePtr m_ExposureAutoTargetFeature;
        AVT::VmbAPI::FeaturePtr m_GainFeature;
        AVT::VmbAPI::FeaturePtr m_GainAutoFeature;
        AVT::VmbAPI::FeaturePtr m_GainSelectorFeature;
        AVT::VmbAPI::FeaturePtr m_GainAutoAdjustTolFeature;
        AVT::VmbAPI::FeaturePtr m_GainAutoMaxFeature;
        AVT::VmbAPI::FeaturePtr m_GainAutoMinFeature;
        AVT::VmbAPI::FeaturePtr m_GainAutoOutliersFeature;
        AVT::VmbAPI::FeaturePtr m_GainAutoRateFeature;
        AVT::VmbAPI::FeaturePtr m_GainAutoTargetFeature;
        AVT::VmbAPI::FeaturePtr m_LUTEnableFeature;
        AVT::VmbAPI::FeaturePtr m_LUTIndexFeature;
        AVT::VmbAPI::FeaturePtr m_LUTLoadAllFeature;
        AVT::VmbAPI::FeaturePtr m_LUTModeFeature;
        AVT::VmbAPI::FeaturePtr m_LUTSaveAllFeature;
        AVT::VmbAPI::FeaturePtr m_LUTSelectorFeature;
        AVT::VmbAPI::FeaturePtr m_LUTValueFeature;
        AVT::VmbAPI::FeaturePtr m_LUTAddressFeature;
        AVT::VmbAPI::FeaturePtr m_LUTBitDepthInFeature;
        AVT::VmbAPI::FeaturePtr m_LUTBitDepthOutFeature;
        AVT::VmbAPI::FeaturePtr m_LUTSizeBytesFeature;
        AVT::VmbAPI::FeaturePtr m_BalanceRatioAbsFeature;
        AVT::VmbAPI::FeaturePtr m_BalanceRatioSelectorFeature;
        AVT::VmbAPI::FeaturePtr m_BalanceWhiteAutoFeature;
        AVT::VmbAPI::FeaturePtr m_BalanceWhiteAutoAdjustTolFeature;
        AVT::VmbAPI::FeaturePtr m_BalanceWhiteAutoRateFeature;
        AVT::VmbAPI::FeaturePtr m_DeviceTemperatureFeature;
        AVT::VmbAPI::FeaturePtr m_DeviceTemperatureSelectorFeature;
        AVT::VmbAPI::FeaturePtr m_EventNotificationFeature;
        AVT::VmbAPI::FeaturePtr m_EventSelectorFeature;
        AVT::VmbAPI::FeaturePtr m_EventsEnable1Feature;
        AVT::VmbAPI::FeaturePtr m_EventOverflowTimestampFeature;
        AVT::VmbAPI::FeaturePtr m_EventOverflowFrameIDFeature;
        AVT::VmbAPI::FeaturePtr m_EventLine1RisingEdgeTimestampFeature;
        AVT::VmbAPI::FeaturePtr m_EventLine1RisingEdgeFrameIDFeature;
        AVT::VmbAPI::FeaturePtr m_EventLine1FallingEdgeTimestampFeature;
        AVT::VmbAPI::FeaturePtr m_EventLine1FallingEdgeFrameIDFeature;
        AVT::VmbAPI::FeaturePtr m_EventFrameTriggerTimestampFeature;
        AVT::VmbAPI::FeaturePtr m_EventFrameTriggerReadyTimestampFeature;
        AVT::VmbAPI::FeaturePtr m_EventFrameTriggerReadyFrameIDFeature;
        AVT::VmbAPI::FeaturePtr m_EventFrameTriggerFrameIDFeature;
        AVT::VmbAPI::FeaturePtr m_EventExposureEndTimestampFeature;
        AVT::VmbAPI::FeaturePtr m_EventExposureEndFrameIDFeature;
        AVT::VmbAPI::FeaturePtr m_EventErrorTimestampFeature;
        AVT::VmbAPI::FeaturePtr m_EventErrorFrameIDFeature;
        AVT::VmbAPI::FeaturePtr m_EventAcquisitionStartTimestampFeature;
        AVT::VmbAPI::FeaturePtr m_EventAcquisitionStartFrameIDFeature;
        AVT::VmbAPI::FeaturePtr m_EventAcquisitionRecordTriggerTimestampFeature;
        AVT::VmbAPI::FeaturePtr m_EventAcquisitionRecordTriggerFrameIDFeature;
        AVT::VmbAPI::FeaturePtr m_EventAcquisitionEndTimestampFeature;
        AVT::VmbAPI::FeaturePtr m_EventAcquisitionEndFrameIDFeature;
        AVT::VmbAPI::FeaturePtr m_EventAcquisitionEndFeature;
        AVT::VmbAPI::FeaturePtr m_EventAcquisitionRecordTriggerFeature;
        AVT::VmbAPI::FeaturePtr m_EventAcquisitionStartFeature;
        AVT::VmbAPI::FeaturePtr m_EventErrorFeature;
        AVT::VmbAPI::FeaturePtr m_EventExposureEndFeature;
        AVT::VmbAPI::FeaturePtr m_EventFrameTriggerFeature;
        AVT::VmbAPI::FeaturePtr m_EventFrameTriggerReadyFeature;
        AVT::VmbAPI::FeaturePtr m_EventLine1FallingEdgeFeature;
        AVT::VmbAPI::FeaturePtr m_EventLine1RisingEdgeFeature;
        AVT::VmbAPI::FeaturePtr m_EventLine2FallingEdgeFeature;
        AVT::VmbAPI::FeaturePtr m_EventLine2RisingEdgeFeature;
        AVT::VmbAPI::FeaturePtr m_EventLine3FallingEdgeFeature;
        AVT::VmbAPI::FeaturePtr m_EventLine3RisingEdgeFeature;
        AVT::VmbAPI::FeaturePtr m_EventLine4FallingEdgeFeature;
        AVT::VmbAPI::FeaturePtr m_EventLine4RisingEdgeFeature;
        AVT::VmbAPI::FeaturePtr m_EventOverflowFeature;
        AVT::VmbAPI::FeaturePtr m_BandwidthControlModeFeature;
        AVT::VmbAPI::FeaturePtr m_ChunkModeActiveFeature;
        AVT::VmbAPI::FeaturePtr m_GevDeviceMACAddressFeature;
        AVT::VmbAPI::FeaturePtr m_GevSCPSPacketSizeFeature;
        AVT::VmbAPI::FeaturePtr m_NonImagePayloadSizeFeature;
        AVT::VmbAPI::FeaturePtr m_PayloadSizeFeature;
        AVT::VmbAPI::FeaturePtr m_StreamBytesPerSecondFeature;
        AVT::VmbAPI::FeaturePtr m_StreamFrameRateConstrainFeature;
        AVT::VmbAPI::FeaturePtr m_GevIPConfigurationModeFeature;
        AVT::VmbAPI::FeaturePtr m_GevCurrentDefaultGatewayFeature;
        AVT::VmbAPI::FeaturePtr m_GevCurrentIPAddressFeature;
        AVT::VmbAPI::FeaturePtr m_GevCurrentSubnetMaskFeature;
        AVT::VmbAPI::FeaturePtr m_GVCPCmdRetriesFeature;
        AVT::VmbAPI::FeaturePtr m_GVCPCmdTimeoutFeature;
        AVT::VmbAPI::FeaturePtr m_GVCPHBIntervalFeature;
        AVT::VmbAPI::FeaturePtr m_GevPersistentDefaultGatewayFeature;
        AVT::VmbAPI::FeaturePtr m_GevPersistentIPAddressFeature;
        AVT::VmbAPI::FeaturePtr m_GevPersistentSubnetMaskFeature;
        AVT::VmbAPI::FeaturePtr m_StreamHoldCapacityFeature;
        AVT::VmbAPI::FeaturePtr m_StreamHoldEnableFeature;
        AVT::VmbAPI::FeaturePtr m_GevTimestampControlLatchFeature;
        AVT::VmbAPI::FeaturePtr m_GevTimestampControlResetFeature;
        AVT::VmbAPI::FeaturePtr m_GevTimestampTickFrequencyFeature;
        AVT::VmbAPI::FeaturePtr m_GevTimestampValueFeature;
        AVT::VmbAPI::FeaturePtr m_StrobeDelayFeature;
        AVT::VmbAPI::FeaturePtr m_StrobeDurationFeature;
        AVT::VmbAPI::FeaturePtr m_StrobeDurationModeFeature;
        AVT::VmbAPI::FeaturePtr m_StrobeSourceFeature;
        AVT::VmbAPI::FeaturePtr m_SyncInGlitchFilterFeature;
        AVT::VmbAPI::FeaturePtr m_SyncInLevelsFeature;
        AVT::VmbAPI::FeaturePtr m_SyncInSelectorFeature;
        AVT::VmbAPI::FeaturePtr m_SyncOutLevelsFeature;
        AVT::VmbAPI::FeaturePtr m_SyncOutPolarityFeature;
        AVT::VmbAPI::FeaturePtr m_SyncOutSelectorFeature;
        AVT::VmbAPI::FeaturePtr m_SyncOutSourceFeature;
        AVT::VmbAPI::FeaturePtr m_HeightFeature;
        AVT::VmbAPI::FeaturePtr m_HeightMaxFeature;
        AVT::VmbAPI::FeaturePtr m_ImageSizeFeature;
        AVT::VmbAPI::FeaturePtr m_OffsetXFeature;
        AVT::VmbAPI::FeaturePtr m_OffsetYFeature;
        AVT::VmbAPI::FeaturePtr m_PixelFormatFeature;
        AVT::VmbAPI::FeaturePtr m_WidthFeature;
        AVT::VmbAPI::FeaturePtr m_WidthMaxFeature;
        AVT::VmbAPI::FeaturePtr m_SensorHeightFeature;
        AVT::VmbAPI::FeaturePtr m_SensorWidthFeature;
        AVT::VmbAPI::FeaturePtr m_DeviceFirmwareVersionFeature;
        AVT::VmbAPI::FeaturePtr m_DeviceIDFeature;
        AVT::VmbAPI::FeaturePtr m_DeviceModelNameFeature;
        AVT::VmbAPI::FeaturePtr m_DevicePartNumberFeature;
        AVT::VmbAPI::FeaturePtr m_DeviceScanTypeFeature;
        AVT::VmbAPI::FeaturePtr m_DeviceVendorNameFeature;
        AVT::VmbAPI::FeaturePtr m_FirmwareVerBuildFeature;
        AVT::VmbAPI::FeaturePtr m_FirmwareVerMajorFeature;
        AVT::VmbAPI::FeaturePtr m_FirmwareVerMinorFeature;
        AVT::VmbAPI::FeaturePtr m_SensorBitsFeature;
        AVT::VmbAPI::FeaturePtr m_SensorTypeFeature;
        AVT::VmbAPI::FeaturePtr m_UserSetDefaultSelectorFeature;
        AVT::VmbAPI::FeaturePtr m_UserSetLoadFeature;
        AVT::VmbAPI::FeaturePtr m_UserSetSaveFeature;
        AVT::VmbAPI::FeaturePtr m_UserSetSelectorFeature;
        AVT::VmbAPI::FeaturePtr m_GVSPFilterVersionFeature;
        AVT::VmbAPI::FeaturePtr m_MulticastEnableFeature;
        AVT::VmbAPI::FeaturePtr m_MulticastIPAddressFeature;
        AVT::VmbAPI::FeaturePtr m_GVSPAdjustPacketSizeFeature;
        AVT::VmbAPI::FeaturePtr m_GVSPBurstSizeFeature;
        AVT::VmbAPI::FeaturePtr m_GVSPDriverFeature;
        AVT::VmbAPI::FeaturePtr m_GVSPHostReceiveBuffersFeature;
        AVT::VmbAPI::FeaturePtr m_GVSPMaxLookBackFeature;
        AVT::VmbAPI::FeaturePtr m_GVSPMaxRequestsFeature;
        AVT::VmbAPI::FeaturePtr m_GVSPMaxWaitSizeFeature;
        AVT::VmbAPI::FeaturePtr m_GVSPMissingSizeFeature;
        AVT::VmbAPI::FeaturePtr m_GVSPPacketSizeFeature;
        AVT::VmbAPI::FeaturePtr m_GVSPTiltingSizeFeature;
        AVT::VmbAPI::FeaturePtr m_GVSPTimeoutFeature;
        AVT::VmbAPI::FeaturePtr m_StatFrameDeliveredFeature;
        AVT::VmbAPI::FeaturePtr m_StatFrameDroppedFeature;
        AVT::VmbAPI::FeaturePtr m_StatFrameRateFeature;
        AVT::VmbAPI::FeaturePtr m_StatFrameRescuedFeature;
        AVT::VmbAPI::FeaturePtr m_StatFrameShovedFeature;
        AVT::VmbAPI::FeaturePtr m_StatFrameUnderrunFeature;
        AVT::VmbAPI::FeaturePtr m_StatLocalRateFeature;
        AVT::VmbAPI::FeaturePtr m_StatPacketErrorsFeature;
        AVT::VmbAPI::FeaturePtr m_StatPacketMissedFeature;
        AVT::VmbAPI::FeaturePtr m_StatPacketReceivedFeature;
        AVT::VmbAPI::FeaturePtr m_StatPacketRequestedFeature;
        AVT::VmbAPI::FeaturePtr m_StatPacketResentFeature;
        AVT::VmbAPI::FeaturePtr m_StatTimeElapsedFeature;
};

#endif
