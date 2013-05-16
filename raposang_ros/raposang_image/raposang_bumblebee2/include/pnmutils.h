//=============================================================================
// Copyright © 2000 Point Grey Research, Inc. All Rights Reserved.
// 
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with PGR.
// 
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// $Id: pnmutils.h,v 1.11 2003/12/11 19:51:06 jbeis Exp $
//=============================================================================
#ifndef _PNMUTILS_H_
#define _PNMUTILS_H_

//=============================================================================
//
// pnmutils:
//
// This file provides functions for reading and writing a PGM and PPM
// files from and to a TriclopsImage.
// Users are encouraged to modify this source to suit their own needs.
//=============================================================================


//=============================================================================
// System Includes
//=============================================================================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


//=============================================================================
// PGR Includes
//=============================================================================
#include "triclops.h"


#ifdef __cplusplus
extern "C"
{
#endif


//=============================================================================
// pgmReadToTriclopsImage()
//
//	This function reads an 8-bit pgm file into a TriclopsImage 
// 	structure.  It allocates the data within the TriclopsImage to contain
//	the image data.  This data must be freed after it is used by calling
//	'freeImage()'
//
// return codes:
//	True	- the file was successfully read and memory allocated
//	False	- the file read failed (no memory was allocated)
//
TriclopsBool
pgmReadToTriclopsImage(	const char* 	filename,
			TriclopsImage*	image );

//=============================================================================
// pgmReadToTriclopsImage16()
//
//	This function reads an 16-bit pgm file into a TriclopsImage16
// 	structure.  It allocates the data within the TriclopsImage16 to 
//	contain the image data.  This data must be freed after it is used 
//	by calling 'freeImage16()'
//
// return codes:
//	True	- the file was successfully read and memory allocated
//	False	- the file read failed (no memory was allocated)
//
TriclopsBool
pgmReadToTriclopsImage16(  const char* 	     filename,
			   TriclopsImage16*  image );


//=============================================================================
//
//
//
//
TriclopsBool
pgmRead3ToTriclopsInput(   const char*	  redFileName,
			   const char*	  bluFileName,
			   const char*	  greFileName,
			   TriclopsInput* pInput );


//=============================================================================
// ppmReadToTriclopsInput()
//
//	This function reads a ppm file into a TriclopsInput
// 	structure (of type TriInp_RGB_32BIT_PACKED).  
//	It allocates the data within the TriclopsInput to 
//	contain the image data.  This data must be freed after it is used 
//	by calling 'freeInput()'
//
//	This data is suitable to be be input into the Triclops stereo
//	library with a call to 'triclopsPreprocess()'
//
// return codes:
//	True	- the file was successfully read and memory allocated
//	False	- the file read failed (no memory was allocated)
//
TriclopsBool
ppmReadToTriclopsInput(	const char*    filename,
			TriclopsInput* input );


//=============================================================================
// ppmReadToTriclopsInputRGB()
//
//	This function reads a ppm file into a TriclopsInput
// 	structure (of type TriInp_RGB).  
//	It allocates the data within the TriclopsInput to 
//	contain the image data.  This data must be freed after it is used 
//	by calling 'freeInput()'
//
//	This data is suitable to be be input into the Triclops stereo
//	library with a call to 'triclopsPreprocess()'
//
// return codes:
//	True	- the file was successfully read and memory allocated
//	False	- the file read failed (no memory was allocated)
//
TriclopsBool
ppmReadToTriclopsInputRGB(	const char*    filename,
				TriclopsInput* input );


//=============================================================================
// freeInput()
//
//	This function frees the image memory associated with a TriclopsInput.
//	This needs to be called after the user is finished with a 
//	TriclopsInput structure that was created from a called to
// 	'ppmReadToTriclopsInput()'
//
// return codes:
//	True	- the memory was freed
//	False	- the TriclopsInput was not in a supported format
//
TriclopsBool
freeInput( TriclopsInput* input );


//=============================================================================
// freeImage()
//
//	This function frees the image memory associated with a TriclopsImage.
//	This needs to be called after the user is finished with a 
//	TriclopsImage structure that was created from a called to
// 	'pgmReadToTriclopsImage()'
//
// return codes:
//	True	- the memory was freed
//
TriclopsBool
freeImage( TriclopsImage* pimage );


//=============================================================================
// freeImage16()
//
//	This function frees the image memory associated with a 
//	TriclopsImage16.
//	This needs to be called after the user is finished with a 
//	TriclopsImage structure that was created from a called to
// 	'pgmReadToTriclopsImage16()'
//
// return codes:
//	True	- the memory was freed
//
TriclopsBool
freeImage16( TriclopsImage16* pimage );


//=============================================================================
// pgmWriteFromTriclopsImage()
//
//	This function writes an 8-bit pgm file from a TriclopsImage 
//	structure.
//
// return codes:
//	True	- the file was successfully written
//	False	- 'filename' could not be opened for writing
//
TriclopsBool
pgmWriteFromTriclopsImage( const char* 	     filename,
			   TriclopsImage*    image );


//=============================================================================
// ppmWriteFromTriclopsColorImage()
//
//	This function writes an 24-bit ppm file from a TriclopsColorImage
//	structure.
//
// return codes:
//	True	- the file was successfully written
//	False	- 'filename' could not be opened for writing
//
TriclopsBool
ppmWriteFromTriclopsColorImage(	 const char*	      filename,
				 TriclopsColorImage*  image );

//=============================================================================
//
//
//
//
TriclopsBool
pgmWrite3FromTriclopsInput(   const char*    redFilename,
			      const char*    greFilename,
			      const char*    bluFilename,
			      TriclopsInput* input );


//=============================================================================
//
//
//
//
TriclopsBool
pgmWrite3FromTriclopsInputWithComment( const char*    redFilename,
				       const char*    greFilename,
				       const char*    bluFilename,
				       const char*    comment,
				       TriclopsInput* input );


//=============================================================================
// pgmWriteFromTriclopsImage16()
//
//	This function writes an 16-bit pgm file from a TriclopsImage 
//	structure.
//
// return codes:
//	True	- the file was successfully written
//	False	- 'filename' could not be opened for writing
//
TriclopsBool
pgmWriteFromTriclopsImage16(  const char*	filename,
			      TriclopsImage16*	image );


//=============================================================================
// ppmWriteFromTriclopsInput()
//
//	This function writes an 24-bit ppm file from a TriclopsInput 
//	structure.
//
// return codes:
//	True	- the file was successfully written
//	False	- 'filename' could not be opened for writing
//		- or the TriclopsInput was not in a supported format
//
TriclopsBool
ppmWriteFromTriclopsInput( const char*	  filename,
			   TriclopsInput* input );


//=============================================================================
//
//
//
//
TriclopsBool
ppmWriteFromTriclopsInputWithComment(  const char*    filename,
				       const char*    comment,
				       TriclopsInput* input );



#ifdef __cplusplus
}
#endif


#endif //#ifndef _PNMUTILS_H_
