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

extern void Abc_RLfLOGetMaxDelayTotalArea( Abc_Frame_t * pAbc, float * pMaxDelay, float * pTotalArea, int nTreeCRatio, int fUseWireLoads, int fShowAll, int fPrintPath, int fDumpStats )
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

void Abc_RLfLOMapGetAreaDelay( Abc_Frame_t * pAbc, float * pArea, float * pDelay, int fAreaOnly, int useDelayTarget, double DelayTargetArg, int nTreeCRatio, int fUseWireLoads )
{
    Abc_Ntk_t * pNtk, * pNtkRes;
    double DelayTarget;
    double AreaMulti;
    double DelayMulti;
    float LogFan = 0;
    float Slew = 0; // choose based on the library
    float Gain = 250;
    int nGatesMin = 0;
    int fRecovery;
    int fSweep;
    int fSwitching;
    int fSkipFanout;
    int fUseProfile;
    int fUseBuffs;
    int fVerbose;
    extern Abc_Ntk_t * Abc_NtkMap( Abc_Ntk_t * pNtk, double DelayTarget, double AreaMulti, double DelayMulti, float LogFan, float Slew, float Gain, int nGatesMin, int fRecovery, int fSwitching, int fSkipFanout, int fUseProfile, int fUseBuffs, int fVerbose );
    extern int Abc_NtkFraigSweep( Abc_Ntk_t * pNtk, int fUseInv, int fExdc, int fVerbose, int fVeryVerbose );

    pNtk = Abc_FrameReadNtk(pAbc);
    // set defaults
    DelayTarget =-1;
    AreaMulti   = 0;
    DelayMulti  = 0;
    fRecovery   = 1;
    fSweep      = 0;
    fSwitching  = 0;
    fSkipFanout = 0;
    fUseProfile = 0;
    fUseBuffs   = 0;
    fVerbose    = 0;
    Extra_UtilGetoptReset();

    if ( useDelayTarget )
    {
        DelayTarget = DelayTargetArg;
    }

    if ( pNtk == NULL )
    {
        Abc_Print( -1, "Empty network.\n" );
        return;
    }

    if ( fAreaOnly )
        DelayTarget = ABC_INFINITY;

    if ( !Abc_NtkIsStrash(pNtk) )
    {
        pNtk = Abc_NtkStrash( pNtk, 0, 0, 0 );
        if ( pNtk == NULL )
        {
            Abc_Print( -1, "Strashing before mapping has failed.\n" );
            return;
        }
        pNtk = Abc_NtkBalance( pNtkRes = pNtk, 0, 0, 1 );
        Abc_NtkDelete( pNtkRes );
        if ( pNtk == NULL )
        {
            Abc_Print( -1, "Balancing before mapping has failed.\n" );
            return;
        }
        Abc_Print( 0, "The network was strashed and balanced before mapping.\n" );
        // get the new network
        pNtkRes = Abc_NtkMap( pNtk, DelayTarget, AreaMulti, DelayMulti, LogFan, Slew, Gain, nGatesMin, fRecovery, fSwitching, fSkipFanout, fUseProfile, fUseBuffs, fVerbose );
        if ( pNtkRes == NULL )
        {
            Abc_NtkDelete( pNtk );
            Abc_Print( -1, "Mapping has failed.\n" );
            return;
        }
        Abc_NtkDelete( pNtk );
    }
    else
    {
        // get the new network
        pNtkRes = Abc_NtkMap( pNtk, DelayTarget, AreaMulti, DelayMulti, LogFan, Slew, Gain, nGatesMin, fRecovery, fSwitching, fSkipFanout, fUseProfile, fUseBuffs, fVerbose );
        if ( pNtkRes == NULL )
        {
            Abc_Print( -1, "Mapping has failed.\n" );
            return;
        }
    }

    if ( fSweep )
    {
        Abc_NtkFraigSweep( pNtkRes, 0, 0, 0, 0 );
        if ( Abc_NtkHasMapping(pNtkRes) )
        {
            pNtkRes = Abc_NtkDupDfs( pNtk = pNtkRes );
            Abc_NtkDelete( pNtk );
        }
    }

    // get the area and delay from the mapped network
    SC_Lib * pLib = (SC_Lib *)pAbc->pLibScl;
    SC_Man * p;
    Abc_Ntk_t * pNtkNew = pNtkRes;
    if ( pNtkRes->nBarBufs2 > 0 ){
        pNtkNew = Abc_NtkDupDfsNoBarBufs( pNtkRes );
    }
    p = Abc_SclManStart( pLib, pNtkNew, fUseWireLoads, 1, 0, nTreeCRatio);
    *pDelay = p->MaxDelay;
    *pArea = Abc_SclGetTotalArea(pNtkRes);
    Abc_SclManFree( p );
    if ( pNtkRes->nBarBufs2 > 0 )
        Abc_NtkDelete( pNtkNew );

    // delete the mapped Network
    Abc_NtkDelete(pNtkRes);
    return;
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

void Abc_RLfLOPrintNodeIds( Abc_Frame_t * pAbc )
{
    Abc_Ntk_t * pNtk;
    Abc_Obj_t * pObj;
    int i;
    pNtk = Abc_FrameReadNtk(pAbc);
    Vec_PtrForEachEntry(Abc_Obj_t * , pNtk->vObjs, pObj, i){
        if (pObj){
            printf( "The id is: %d ; the type is: %d     ", pObj->Id, pObj->Type );
        }
        if (!pObj){
            printf("got here iteration is: %d", i);
        }
    }
}

void Abc_RLfLOGetNumObjs( Abc_Frame_t * pAbc, int * pObjNum ){
    Abc_Ntk_t * pNtk;
    Abc_Obj_t * pObj;
    int i;
    (*pObjNum) = 0;
    // printf("Got into func \n");
    pNtk = Abc_FrameReadNtk(pAbc);
    // if (Abc_NtkIsStrash(pNtk)){
    //     printf("Ntk is Strash \n");
    // }
    // printf("got network pointer \n");
    // printf("The network type is: %d", pNtk->ntkType);
    // printf("The network func is: %d", pNtk->ntkFunc);
    Vec_PtrForEachEntry(Abc_Obj_t * , pNtk->vObjs, pObj, i){
        if ( Abc_ObjFaninNum(pObj)>0 || Abc_ObjFanoutNum(pObj)>0 ){
        if (!pObj){
            printf("pObj is null pointer! \n");
        }
            (*pObjNum)++;
        }
    }
}

void Abc_RLfLOGetObjTypes( Abc_Frame_t * pAbc, int * x)
{
    Abc_Ntk_t * pNtk;
    Abc_Obj_t * pObj;
    int i;
    int j = 0;
    pNtk = Abc_FrameReadNtk(pAbc);
    Vec_PtrForEachEntry(Abc_Obj_t * , pNtk->vObjs, pObj, i){
        if ( Abc_ObjFaninNum(pObj)>0 || Abc_ObjFanoutNum(pObj)>0 ){
            x[j++] = pObj->Type;
        }
    }
}

void Abc_RLfLOGetNodeFeatures( Abc_Frame_t * pAbc, float * x, size_t n, size_t p)
{
    Abc_Ntk_t * pNtk;
    Abc_Obj_t * pObj;
    int i;
    int j = 0;
    pNtk = Abc_FrameReadNtk(pAbc);
    Vec_PtrForEachEntry(Abc_Obj_t * , pNtk->vObjs, pObj, i){
        if ( Abc_ObjFaninNum(pObj)>0 || Abc_ObjFanoutNum(pObj)>0 ){
            switch(pObj->Type)
            {
                case ABC_OBJ_PI:
                    x[j*p] = 0;
                    break;
                case ABC_OBJ_PO:
                    x[j*p] = 1;
                    break;
                case ABC_OBJ_NODE:
                    x[j*p] = 2;
                    break;
                default:
                    assert(false); // It has to be one of the cases!
            }
            x[j*p+1] = pObj->fCompl0 + pObj->fCompl1;
            assert( j*p+1 < n*p ); // make sure we are within the array
            j++;
        }
    }
}

void Abc_RLfLOGetNumEdges( Abc_Frame_t * pAbc, int * pNumEdges ){
    Abc_Ntk_t * pNtk;
    Abc_Obj_t * pObj;
    int i;
    *pNumEdges = 0;
    pNtk = Abc_FrameReadNtk(pAbc);
    Vec_PtrForEachEntry(Abc_Obj_t * , pNtk->vObjs, pObj, i){
        if (pObj){
            *pNumEdges += Abc_ObjFaninNum(pObj);
        }
    }
}

void Abc_RLfLOGetEdges( Abc_Frame_t * pAbc, long * pEdges, int nEdges, float * pEdgeFeatures ){
    Abc_Ntk_t * pNtk;
    Abc_Obj_t * pObj;
    int i;
    int j;
    int y = 0;
    int Entry;
    int numDisconnected = 0;
    pNtk = Abc_FrameReadNtk(pAbc);
    assert( Abc_NtkIsStrash(pNtk) );
    Vec_PtrForEachEntry(Abc_Obj_t * , pNtk->vObjs, pObj, i){
        if (Abc_ObjFanoutNum(pObj)==0 && Abc_ObjFaninNum(pObj)==0){
            assert( Abc_AigNodeIsConst(pObj) );
            assert( numDisconnected==0);
            numDisconnected++;
        }
        Vec_IntForEachEntry( Abc_ObjFaninVec(pObj), Entry, j ){
            pEdges[y] = Entry - numDisconnected;
            pEdges[nEdges + y] = pObj->Id - numDisconnected;
            if (Abc_ObjIsNode(pObj)){
                if (j == 0){
                    pEdgeFeatures[y] = Abc_NtkObj(pNtk, Entry)->fCompl0;
                } else{
                    assert(j == 1);
                    pEdgeFeatures[y] = Abc_NtkObj(pNtk, Entry)->fCompl1;
                }
            } else{
                pEdgeFeatures[y] = 0;
            }
            y++;
            assert(j <= 1);
        }
    }
    assert(y==nEdges);
}

void Abc_RLfLOPrintObjNum2x(Abc_Frame_t * pAbc){
    Abc_Ntk_t * pNtk;
    pNtk = Abc_FrameReadNtk(pAbc);
    printf("the Number of objects according to the Ntk_t: %d", pNtk->nObjs);
    printf("the number of objects according to the vObjs: %d", pNtk->vObjs->nSize);
}

void Abc_RLfLOSizeofInt(size_t * size){
    *size = sizeof(int);
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

