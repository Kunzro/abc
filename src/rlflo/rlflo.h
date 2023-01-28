/**CFile****************************************************************

  FileName    [rlflo.h]

  SystemName  [ABC: Logic synthesis and verification system.]

  PackageName []

  Synopsis    []

  Author      [Roma Kunz]
  
  Affiliation [ETH Zurich]

  Date        [Ver. 1.0. Started - August 20, 2022.]

  Revision    [$Id: .h,v 1.00 2005/06/20 00:00:00 alanmi Exp $]

***********************************************************************/
 
#ifndef ABC__rlflo_h
#define ABC__rlflo_h


////////////////////////////////////////////////////////////////////////
///                          INCLUDES                                ///
////////////////////////////////////////////////////////////////////////

#include "map/scl/sclSize.h"
#include "base/main/mainInt.h"
#include "base/abc/abc.h"
#include "opt/rwr/rwr.h"
#include "bool/dec/dec.h"

////////////////////////////////////////////////////////////////////////
///                         PARAMETERS                               ///
////////////////////////////////////////////////////////////////////////

ABC_NAMESPACE_HEADER_START

////////////////////////////////////////////////////////////////////////
///                         BASIC TYPES                              ///
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
///                      MACRO DEFINITIONS                           ///
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
///                    FUNCTION DECLARATIONS                         ///
////////////////////////////////////////////////////////////////////////

extern int Abc_AigReplaceSkipNew( Abc_Aig_t * pMan, Abc_Obj_t * pOld, Abc_Obj_t * pNew, int fUpdateLevel );
extern void Abc_RLfLOMapGetAreaDelay( Abc_Frame_t * pAbc, float * pArea, float * pDelay, int fAreaOnly, int useDelayTarget, double DelayTargetArg, int nTreeCRatio, int fUseWireLoads, int getDelays, float * pDelays );
extern void Abc_RLfLOGetNodeFeatures( Abc_Frame_t * pAbc, float * type, size_t type_size, float * num_inverted_inputs, size_t inv_size);
extern void Abc_RLfLOGetMaxDelayTotalArea( Abc_Frame_t * pAbc, float * pMaxDelay, float * pTotalArea, int nTreeCRatio, int fUseWireLoads);
extern void Abc_RLfLOGetNumNodesAndLevels( Abc_Frame_t * pAbc, int * pNumNodes, int * pNumLevels );
extern void Abc_RLfLOGetNumObjs( Abc_Frame_t * pAbc, int* pObjNum );
extern void Abc_RLfLOGetObjTypes( Abc_Frame_t * pAbc, int * x);
extern void Abc_RLfLOGetNumEdges( Abc_Frame_t * pAbc, int * pNumEdges );
extern void Abc_RLfLOGetEdges( Abc_Frame_t * pAbc, long * pEgdes, int nEdges, float * pEdgeFeatures);
extern void Abc_RLfLOPrintNodeIds( Abc_Frame_t * pAbc );
extern void Abc_RLfLONtkDesub( Abc_Frame_t * pAbc, int Id );
extern int Abc_RLfLONtkRefactor( Abc_Frame_t * pAbc, int Id, int nNodeSizeMax, int nConeSizeMax, int fUpdateLevel, int fUseZeros, int fUseDcs, int fVerbose );
extern int Abc_RLfLONtkRewrite( Abc_Frame_t * pAbc, int Id, int fUpdateLevel, int fUseZeros, int fVerbose, int fVeryVerbose, int fPlaceEnable );
extern int Abc_RLfLONtkResubstitute( Abc_Frame_t * pAbc, int Id ,int nCutMax, int nStepsMax, int nLevelsOdc, int fUpdateLevel, int fVerbose, int fVeryVerbose );
extern void Abc_RLfLOBalanceNode( Abc_Frame_t * pAbc, int Id, int fUpdateLevel, int fSelective);
/*=== zzz.c ==========================================================*/
 


ABC_NAMESPACE_HEADER_END



#endif

////////////////////////////////////////////////////////////////////////
///                       END OF FILE                                ///
////////////////////////////////////////////////////////////////////////

