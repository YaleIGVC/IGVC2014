///////////////////////////////////////////////////////////////////////////////
// (C) 2008-2013 IXXAT Automation GmbH, all rights reserved
///////////////////////////////////////////////////////////////////////////////
/**
  ECI API Demo Program

  @author Michael Ummenhofer (ummenhofer@ixxat.de)
  @file LinuxEciDemo.c
*/


//////////////////////////////////////////////////////////////////////////
// compiler directives

//////////////////////////////////////////////////////////////////////////
// include files
#include <EciDemo002.h>
#include <EciDemo003.h>
#include <EciDemo005.h>
#include <EciDemo101.h>
#include <EciDemo102.h>
#include <EciDemo105.h>
#include <EciDemo109.h>
#include <EciDemo10A.h>
#include <EciDemo10E.h>
#include <EciDemo111.h>


//////////////////////////////////////////////////////////////////////////
// static constants, types, macros, variables

//////////////////////////////////////////////////////////////////////////
// global variables

//////////////////////////////////////////////////////////////////////////
// static function prototypes

//////////////////////////////////////////////////////////////////////////
// global functions

//////////////////////////////////////////////////////////////////////////
// static functions


///////////////////////////////////////////////////////////////////////////////
/**
  main function

  @param argc Argument count
  @param argv Array of arguments
  @param envp environment variables

*/
int   main( int        argc,
            char**     argv,
            char**     envp )
{
  ECI_RESULT hResult = ECI_OK;

  OS_Printf(">> Linux ECI API Demo program <<\n\n");

  //*** ECI Demo for PC-I 04 / PCI
  hResult = EciDemo002();

  //*** ECI Demo for PC-I 04 / 104
  hResult = EciDemo003();

  //*** ECI Demo for CAN-IB1x0 / PCIe (Mini), (104)
  hResult = EciDemo005();

  //*** ECI Demo for iPC-I 320 / PCI
  hResult = EciDemo101();

  //*** ECI Demo for iPC-I 320 / 104
  hResult = EciDemo102();

  //*** ECI Demo for iPC-I XC16 / PCI (PMC)
  hResult = EciDemo105();

  //*** ECI Demo for USB-to-CAN compact
  hResult = EciDemo109();

  //*** ECI Demo for USB-to-CAN II
  hResult = EciDemo10A();

  //*** ECI Demo for iPC-I XC16 / PCIe
  hResult = EciDemo10E();

  //*** ECI Demo for CAN-IB2x0 / PCIe (104)
  hResult = EciDemo111();

  OS_Printf("-> Closing Linux ECI API Demo program <-\n");

  return hResult;
}
