﻿/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        IntSpinBox.cpp

  Description: Spinbox to handle integer feature

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


#include "IntSpinBox.h"


IntSpinBox::IntSpinBox ( QWidget *parent ): QSpinBox ( parent )
{

}

IntSpinBox::~IntSpinBox ( void )
{

}
/*nMax = nMax - (nMax % nInc);
nMax = nMax - ((nMax-nMin) % nInc);
*/

void IntSpinBox::stepBy( int steps )
{
	int nInterval =  singleStep();
	int nValue    = value();

	if(0 < steps ) //stepUp
	{
		{
			if( maximum() >= (nValue+nInterval))
			{
				setValue( nValue+nInterval);
			}	
		}
		
	}
	else   //stepDown
	{
		if( 0 != (nValue-nInterval) % nInterval ) 
		{
			/* value % ninterval should be 0 */
			nValue -= ( (nValue-nInterval) + (nInterval - ((nValue-nInterval) % nInterval)) );

			if( minimum() <= nValue)
			{
				setValue( nValue);
			}
		}
		else
		{
			setValue( nValue-nInterval);
		}
	}

	editingFinished();
}
