﻿/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        MainWindow.h

  Description: The main window framework. This contains of camera tree, a toolbar and logging
               

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


#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ui_MainWindow.h"
#include "ui_StartOption.h"
#include <QSettings>

#include "UI/LoggerWindow.h"
#include "UI/CameraTreeWindow.h"
#include "ViewerWindow.h"

#include <VimbaCPP/Include/VimbaSystem.h>

using namespace AVT::VmbAPI;

class MainWindow : public QMainWindow
{
	Q_OBJECT

    public: 
	            
	protected:
					  
	private:
				Ui::mainWindow				m_MainWindow;
				LoggerWindow				*m_Logger;
				CameraTreeWindow			*m_CameraTree;
				QVector <ViewerWindow*>		m_Viewer;
				QVector<QTreeWidgetItem*>	m_GigE;
                QVector<QTreeWidgetItem*>	m_1394;
				QVector<QTreeWidgetItem*>	m_USB;

				QMap <QString, QStringList> m_CamerasPermittedAccessMap; // <model, access>	
				QMap <QString, QStringList> m_PermittedAccessStateListMap;//<model, state>
				QString                     m_sOpenAccessType;
				QList <QAction *>           m_RightMouseAction;

				unsigned int                m_nOpenState;

				AVT::VmbAPI::VimbaSystem    &m_VimbaSystem;
				CameraPtrVector				m_rCameras;
				Helper                      m_Helper;
				QString                     m_sAPIVersion;
				bool                        m_bIsRightMouseClicked;
				
				QString                     m_sCurrentModel;
				bool                        m_bIsCurrentModelChecked;
				bool                        m_bIsOpenByRightMouseClick;

				/*Start Option (auto adjust packet size) */
				Ui::StartOptionsDialog              m_StartOption;
				QDialog                            *m_StartOptionDialog;
				bool                                m_bIsAutoAdjustPacketSize;
					
	public:
				 MainWindow ( QWidget *parent = 0, Qt::WFlags flags = 0 );
				~MainWindow ( void );

	protected:
								
	private:
				void searchCameras		( void );
				void openViewer			( const QString &sCamID );
				void closeViewer		( const QString &sCamID );
				QString getBestAccess ( const QString &sCamID );
				QStringList getStringListInfo ( const QMap<QString, QStringList> mInfoMap, const QString &sCamID );
				unsigned int getAccessListPosition ( const QString &sAccessName );
				void updateAccessStateListMap ( const unsigned int &nPosition, const QString sStatus, const bool &bIsResetAll);

				virtual void closeEvent ( QCloseEvent *event );	

	private slots:
				/* when you use this std convention, you don't need any "connect..." */
				void on_ActionDiscover_triggered ( void ); 
				void on_ActionClear_triggered    ( void ); 
				void on_ActionStartOptions_triggered  ( void );
				
				/* Custom */
				void onCameraClicked     ( const QString &sModel, const bool &bIsChecked);
				void onRightMouseClicked ( const bool &bIsClicked );
				void onCloseFromViewer   ( const QString &sModel );
				void onUpdateDeviceList  ( void );
				void about ( void );
				void rightMouseOpenCamera ( bool bOpenAccesState );

};

#endif 
