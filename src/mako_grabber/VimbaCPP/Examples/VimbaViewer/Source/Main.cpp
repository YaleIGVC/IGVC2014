﻿/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        Main.cpp

  Description: Main application
 
  Qt: http://qt.nokia.com/
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



#include "MainWindow.h"
#include <QtGui/QApplication>
#include "Version.h"


class myApplication : public QApplication
{

public: 

	myApplication(int &_argc, char *_argv[]): QApplication(_argc, _argv)
	{ 

	}

	bool notify(QObject *receiver_, QEvent *event_)
	{
		try
		{
			return QApplication::notify(receiver_, event_);
		}
		catch ( std::exception const &e )
		{
			qDebug() << "std exception was caught";
			QMessageBox msgBox;
			QString sMsg = e.what();
			msgBox.setText("<Sample Viewer> A std exception occurred: " + sMsg);
			msgBox.exec();
		}
		catch (...)
		{
			qDebug() << "unknown exception was caught";
			QMessageBox msgBox;
			msgBox.setText("An unknown exception occurred.");
			msgBox.exec();
		}

		return false;
	} 
};

int main(int argc, char *argv[])
{
	myApplication a(argc, argv);
	a.setApplicationName("AVT VimbaViewer");
	a.setApplicationVersion(VIMBAVIEWER_VERSION);
	QIcon icon(":/VimbaViewer/Images/stripes_256.png");
	a.setWindowIcon(icon);
	MainWindow Application;
    Application.setWindowTitle(QString("VimbaViewer ") + QString(VIMBAVIEWER_VERSION));

	Application.show();
	return a.exec();
}
