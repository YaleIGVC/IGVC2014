﻿/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        Helper.cpp

  Description: A Common helper class. Mostly used to define any constants and common methods.
               

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


#include "Helper.h"

const QString Helper::m_GIGE_MULTICAST_IP_ADDRESS       = "MulticastIPAddress";

const QString Helper::m_GIGE_CURRENT_IP_ADDRESS         = "GevCurrentIPAddress";      
const QString Helper::m_GIGE_CURRENT_SUBNET_MASK        = "GevCurrentSubnetMask";
const QString Helper::m_GIGE_CURRENT_DEFAULT_GATEWAY    = "GevCurrentDefaultGateway";
const QString Helper::m_GIGE_PERSISTENT_DEFAULT_GATEWAY = "GevPersistentDefaultGateway";
const QString Helper::m_GIGE_PERSISTENT_IP_ADDRESS      = "GevPersistentIPAddress";
const QString Helper::m_GIGE_PERSISTENT_SUBNET_MASK     = "GevPersistentSubnetMask";
const QString Helper::m_GIGE_DEVICE_MAC_ADDRESS         = "GevDeviceMACAddress";

const QString Helper::m_GIGE_STAT_FRAME_DELIVERED       = "Stat Frames Delivered";
const QString Helper::m_GIGE_STAT_FRAME_DROPPED         = "Stat Frames Dropped";
const QString Helper::m_GIGE_STAT_FRAME_RATE            = "Stat Frame Rate";
const QString Helper::m_GIGE_STAT_FRAME_RESCUED         = "Stat Frames Rescued";
const QString Helper::m_GIGE_STAT_FRAME_SHOVED          = "Stat Frames Shoved";
const QString Helper::m_GIGE_STAT_FRAME_UNDERRUN        = "Stat Frames Underrun";
const QString Helper::m_GIGE_STAT_LOCAL_RATE            = "Stat Local Rate";
const QString Helper::m_GIGE_STAT_PACKET_ERRORS         = "Stat Packets Errors";
const QString Helper::m_GIGE_STAT_PACKET_MISSED         = "Stat Packets Missed";
const QString Helper::m_GIGE_STAT_PACKET_RECEIVED       = "Stat Packets Received";
const QString Helper::m_GIGE_STAT_PACKET_REQUESTED      = "Stat Packets Requested";
const QString Helper::m_GIGE_STAT_PACKET_RESENT         = "Stat Packets Resent";
const QString Helper::m_GIGE_STAT_TIME_ELAPSED          = "Stat Time Elapsed";

const QString Helper::m_EXPOSURE_AUTO                   = "ExposureAuto";
const QString Helper::m_GAIN_AUTO                       = "GainAuto";
const QString Helper::m_BALANCE_WHITE_AUTO              = "Balance White Auto";

Helper::Helper ( void )
{ 

}

Helper::~Helper ( void )
{

}

/*
*  The value sent by VmbAPI is big endian, e.g:
*  Multicast IP Address: 823132143 = 0x 31 0F FF EF = 049.015.255.239
*  and should be converted to little endian for GUI:  239.255.015.049
*/
QString Helper::parseToIPAddressLittleEndian( const QString sHexValue )
{
    QString sIP("");
	QString sValue = sHexValue;

	int nDeltaLength = 8 - sValue.length();
	for(int n=0; n < nDeltaLength; n++)
	{
		sValue.push_front("0");
	}

	for(int i=0; i < 8; i+=2)
	{
		QString sByteValue = sValue.mid(i, 2);
		
		bool bOK;
		int nByteValue = sByteValue.toUInt(&bOK,16); 
		
		if(100 > nByteValue)
		{
			if(10 > nByteValue)
			{
				sIP.push_front("00" + QString::number(nByteValue));
			}
			else
			{
                sIP.push_front("0" + QString::number(nByteValue));
			}
		}
		else
		{
            sIP.push_front(QString::number(nByteValue));
		}
	}

	return sIP;
}

QString Helper::displayValueToIPv4( const QString sDecValue )
{
    VmbInt64_t	nValue64  = sDecValue.toLongLong();
    QString sLittleEndian = parseToIPAddressLittleEndian(QString::number(nValue64, 16));

    QString sIPv4;
    for( unsigned int i = 0; i < 12; i+=3)
    {
      sIPv4.append( sLittleEndian.mid( i , 3) );
      if( 9 > i)
      {
        sIPv4.append(".");
      }
    }

    return sIPv4; 
}

// maps return codes to explanation strings for output
QString Helper::mapReturnCodeToString(const VmbError_t &nReturnCode)
{
	QString sResult = "(Return code undefined)";

	switch (nReturnCode)
	{
	case VmbErrorSuccess: sResult = "(No error)"; break;
	case VmbErrorInternalFault: sResult = "(Unexpected fault in VmbApi or driver)"; break;
	case VmbErrorApiNotStarted: sResult = "(VmbStartup() was not called before the current command)"; break;
	case VmbErrorNotFound: sResult = "(The designated instance (camera, feature etc.) cannot be found)"; break;
	case VmbErrorBadHandle: sResult = "(The given handle is not valid)"; break;
	case VmbErrorDeviceNotOpen: sResult = "(Device was not opened for usage)"; break;
	case VmbErrorInvalidAccess: sResult = "(Operation is invalid with the current access mode)"; break;
	case VmbErrorBadParameter: sResult = "(One of the parameters was invalid (usually an illegal pointer))"; break;
	case VmbErrorStructSize: sResult = "(The given struct size is not valid for this version of the API)"; break;
	case VmbErrorMoreData: sResult = "(More data was returned in a string/list than space was provided)"; break;
	case VmbErrorWrongType: sResult = "(The feature type for this access function was wrong)"; break;
	case VmbErrorInvalidValue: sResult = "(The value was not valid; either out of bounds or not an increment of the minimum)"; break;
	case VmbErrorTimeout: sResult = "(Timeout during wait)"; break;
	case VmbErrorOther: sResult = "(Other error)"; break;
	case VmbErrorResources: sResult = "(Resources not available (e.g memory))"; break;
	case VmbErrorInvalidCall: sResult = "(Call is invalid in the current context (e.g callback))"; break;
	case VmbErrorNoTL: sResult = "(No transport layers were found)"; break;
	case VmbErrorNotImplemented: sResult = "(API feature is not implemented)"; break;
	case VmbErrorNotSupported: sResult = "(API feature is not supported)"; break;
	case VmbErrorIncomplete: sResult = "(A multiple registers reads or writes was partially completed)"; break;
    case 7000: sResult = "<Viewer Timeout>";
	default: break;
	      
	}

	return sResult;
}
