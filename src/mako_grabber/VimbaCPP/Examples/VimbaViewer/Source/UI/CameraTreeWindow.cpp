﻿/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        CameraTreeWindow.cpp

  Description: This is the main window of the VimbaViewer that lists all connected cameras. 

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


#include "CameraTreeWindow.h"
#include <QtGui/QHeaderView>
#include <QMenu>

CameraTreeWindow::CameraTreeWindow ( QWidget *parent ): QTreeWidget ( parent ), m_bIsChecked(true), m_bIsCheckboxClicked(false), m_bIsRightMouseClicked(false)
{
	this->setHeaderLabel("Detected Cameras");
	/* you need these three lines to show a H-scrollbar */
	header()->setHorizontalScrollMode( QAbstractItemView::ScrollPerPixel );
	header()->setResizeMode( 0, QHeaderView::ResizeToContents );
	header()->setStretchLastSection( false );
	connect(this, SIGNAL( itemClicked(QTreeWidgetItem *, int) ), this, SLOT( clickOnCamera(QTreeWidgetItem *, int)));	
}

CameraTreeWindow::~CameraTreeWindow ( void )
{

}

QTreeWidgetItem *CameraTreeWindow::createItem ( void )
{
	QTreeWidgetItem *item = new QTreeWidgetItem(this);
	return item;
}

QTreeWidgetItem *CameraTreeWindow::createItem ( QTreeWidgetItem *itemRef, const bool &bIsCheckable )
{
	QTreeWidgetItem *item = new QTreeWidgetItem(itemRef);

	if(bIsCheckable)
	{
		item->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
		item->setCheckState( 0, Qt::Unchecked ); 
		item->setWhatsThis(0, "camera");
	}

	return item;
}

void CameraTreeWindow::setText ( QTreeWidgetItem *itemRef, const QString &text )
{
	itemRef->setText(0, text);
}

void CameraTreeWindow::setCheckCurrentItem ( const bool &bIsChecked )
{
	if(bIsChecked)
		m_CurrentItem->setCheckState(0, Qt::Checked);
	else
		m_CurrentItem->setCheckState(0, Qt::Unchecked);
}

void CameraTreeWindow::clickOnCamera ( QTreeWidgetItem *Item, int Column )
{
	if(!this->isEnabled())
		return;

	m_CurrentItem = Item;

	Qt::CheckState state = Item->checkState(0);
	if(m_bIsCheckboxClicked)
	{
		this->setDisabled(true);
		(Qt::Checked == state) ? emit cameraClicked(Item->text(0), true) : emit cameraClicked(Item->text(0), false);
	}
	else
	{   
		if(0 == Item->whatsThis(0).compare("camera"))
			if(Qt::Checked == state)
			{
				if(!m_bIsRightMouseClicked)
					Item->setCheckState(0, Qt::Unchecked);
				
				emit cameraClicked(Item->text(0), false);
			}
			else
			{
				if(!m_bIsRightMouseClicked)
					Item->setCheckState(0, Qt::Checked);
				
				emit cameraClicked(Item->text(0), true);
		    }	
	}
}	

/* check where the click is going on 
 * By overwriting the mousePressEvent the slot clickOnCamera will be called afterwards 
*/
void CameraTreeWindow::mousePressEvent ( QMouseEvent *event )
{
	m_bIsCheckboxClicked = false;
	QModelIndex clickedIndex = indexAt(event->pos());
	/* make sure the event was on a valid item */
	if (clickedIndex.isValid() == false)
		return;

	Qt::MouseButton mouseBtn = event->button();
	if( Qt::RightButton == mouseBtn )
	{
		m_bIsRightMouseClicked = true;
		emit rightMouseClicked (true);
	}
	else
	{
		m_bIsRightMouseClicked = false;
		emit rightMouseClicked (false);
	}

    /* Get the tree widget's x position */
    int treeX = header()->sectionViewportPosition(0);

    /* Get the x coordinate of the root item. It is required in order to calculate
       the identation of the item */
    int rootX = visualRect(rootIndex()).x();

    /* Get the rectangle of the viewport occupied by the pressed item */
    QRect vrect = visualRect(clickedIndex);

    /* Now we can easily calculate the x coordinate of the item */
    int itemX = treeX + vrect.x() - rootX; 

    /* The item is a checkbox, then an icon and finally the text. */

    /* 1. Get the rect surrounding the checkbox */
    QRect checkboxRect = QRect(itemX, 
                               vrect.y(), 
                               style()->pixelMetric(QStyle::PM_IndicatorWidth)+3,
                               vrect.height()+2); 

    /* 2. Get the rect surrounding the icon */
    QRect iconRect = QRect(itemX + checkboxRect.width(),
                           vrect.y(),
                           iconSize().width(),
                           vrect.height());

    /* 3. Finally get the rect surrounding the text */
    QRect textRect = QRect(itemX + checkboxRect.width() + iconRect.width(),
                           vrect.y(),
                           vrect.width() - checkboxRect.width() - iconRect.width(),
                           vrect.height());       

    /* Now check where the press event took place and handle it correspondingly */

    if(checkboxRect.contains(event->pos())) 
    {
	    m_bIsCheckboxClicked = true;
    } 
   
    QTreeWidget::mousePressEvent(event);
    return;
}
