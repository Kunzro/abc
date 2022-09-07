/**CFile****************************************************************

  FileName    [rlflo.c]

  SystemName  [ABC: Logic synthesis and verification system.]

  PackageName [Custom Functions for Pythin API for Reinforcement Learning]

  Synopsis    []

  Author      [Roman Kunz]
  
  Affiliation [ETH Zurich]

  Date        [Ver. 1.0. Started - August 20, 2022.]

  Revision    [$Id: abc_.c,v 1.00 2005/06/20 00:00:00 alanmi Exp $]

***********************************************************************/

#include "rlflo.h"

ABC_NAMESPACE_IMPL_START


////////////////////////////////////////////////////////////////////////
///                        DECLARATIONS                              ///
////////////////////////////////////////////////////////////////////////
 
////////////////////////////////////////////////////////////////////////
///                     FUNCTION DEFINITIONS                         ///
////////////////////////////////////////////////////////////////////////

void Abc_RLfLOGetMaxDelayTotalArea( Abc_Frame_t * pAbc, float * pMaxDelay, float * pTotalArea, int nTreeCRatio, int fUseWireLoads, int fShowAll, int fPrintPath, int fDumpStats )
{
    SC_Lib * pLib = (SC_Lib *)pAbc->pLibScl;
    Abc_Ntk_t * pNtk = Abc_FrameReadNtk(pAbc);
    SC_Man * p;
    Abc_Ntk_t * pNtkNew = pNtk;
    if ( pNtk->nBarBufs2 > 0 ){
        pNtkNew = Abc_NtkDupDfsNoBarBufs( pNtk );
    }
    p = Abc_SclManStart( pLib, pNtkNew, fUseWireLoads, 1, 0, nTreeCRatio);
    *pMaxDelay = p->MaxDelay;
    *pTotalArea = Abc_SclGetTotalArea(pNtk);
    Abc_SclManFree( p );
    if ( pNtk->nBarBufs2 > 0 )
        Abc_NtkDelete( pNtkNew );
}

void Abc_RLfLOGetNumNodesAndLevels( Abc_Frame_t * pAbc, int * pNumNodes, int * pNumLevels )
{
    Abc_Ntk_t * pNtk = Abc_FrameReadNtk(pAbc);
    *pNumNodes = Abc_NtkNodeNum(pNtk);
    if ( Abc_NtkIsStrash(pNtk) )
    {
        *pNumLevels = Abc_AigLevel(pNtk);
    }
    else
        *pNumLevels = Abc_NtkLevel(pNtk);
}

/**Function*************************************************************

  Synopsis    []

  Description []
               
  SideEffects []

  SeeAlso     []

***********************************************************************/

////////////////////////////////////////////////////////////////////////
///                       END OF FILE                                ///
////////////////////////////////////////////////////////////////////////


ABC_NAMESPACE_IMPL_END

