/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        Csv.h

  Description: Helper to access a CSV file using VimbaCPP.
               

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

#ifndef AVT_VMBAPI_EXAMPLES_LOOKUPTABLE_CSV
#define AVT_VMBAPI_EXAMPLES_LOOKUPTABLE_CSV

#include <iostream>
#include <fstream>

#include <VimbaCPP/Include/VimbaCPP.h>

using namespace std;

namespace AVT {
namespace VmbAPI {
namespace Examples {

class Csv
{
private:
    char m_VD;
    char m_RD;
    bool m_bHasHeader;

public:
    //ctor
    Csv();

    void GetVD ( char& VD );

    void SetVD ( char VD );

    void GetRD ( char& RD );

    void GetHasHeader ( bool& bHasHeader );

    void SetHasHeader ( bool bHasHeader );

    class CsvRow : public vector<string>
    {
    private:
        string m_lineText;
    public:
        void GetLineText ( string& lineText );
        void SetLineText ( string lineText );
    };

    class CsvSave : ofstream
    {
    private:
        Csv& m_instance;
    public:
        //ctor
        CsvSave ( string filename, Csv& instance );

        bool Row ( CsvRow row );
    };

    class CsvLoad : ifstream
    {
    private:
        Csv& m_instance;
    public:
        //ctor
        CsvLoad ( string filename, Csv& instance );

        bool Row ( CsvRow* row );
    };
};

}}} // namespace AVT::Vimba::Examples

#endif