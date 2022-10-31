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

void Abc_RLfLOGetEdges( Abc_Frame_t * pAbc, int * pEdges, int nEdges, int * pEdgeFeatures ){
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

