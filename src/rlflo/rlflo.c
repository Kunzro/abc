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


void Abc_RLfLOGetMaxDelayTotalArea( Abc_Frame_t * pAbc, float * pMaxDelay, float * pTotalArea, int nTreeCRatio, int fUseWireLoads )
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

void Abc_RLfLOMapGetAreaDelay( Abc_Frame_t * pAbc, float * pArea, float * pDelay, int fAreaOnly, int useDelayTarget, double DelayTargetArg, int nTreeCRatio, int fUseWireLoads, int getDelays, double * pDelays )
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

    assert (pNtk); // the Network should not be empty
    assert (Abc_NtkIsStrash(pNtk)); // the network is supposed to be strashed ie an aig graph

    if ( useDelayTarget )
    {
        DelayTarget = DelayTargetArg;
    }

    if ( fAreaOnly )
        DelayTarget = ABC_INFINITY;

    // get the new network
    pNtkRes = Abc_NtkMap( pNtk, DelayTarget, AreaMulti, DelayMulti, LogFan, Slew, Gain, nGatesMin, fRecovery, fSwitching, fSkipFanout, fUseProfile, fUseBuffs, fVerbose );
    if ( pNtkRes == NULL )
    {
        Abc_Print( -1, "Mapping has failed.\n" );
        return;
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
    Abc_Obj_t * pNode, * pFanin;
    int i, k, idshift = 0;
    Abc_Ntk_t * pNtkNew = pNtkRes;
    if ( pNtkRes->nBarBufs2 > 0 ){
        pNtkNew = Abc_NtkDupDfsNoBarBufs( pNtkRes );
    }
    p = Abc_SclManStart( pLib, pNtkNew, fUseWireLoads, 1, 0, nTreeCRatio);
    *pDelay = p->MaxDelay;
    *pArea = p->SumArea; //Abc_SclGetTotalArea(pNtkRes); // already done in Abc_SclManStart
    // get the delays if requested
    // get the nodes ordered by level
    if (getDelays)
    {
        Vec_Vec_t * pNtkLevelized;
        pNtkLevelized = Abc_NtkLevelize(pNtk);  // only includes the nodes and not the pi/po
        // if (Abc_NtkObj(pNtk, 0))
        // {
        //     printf("The const 1 node exists, Type: %d, Level: %d, Id: %d, FanoutNum: %d\n", Abc_NtkObj(pNtk, 0)->Type, Abc_NtkObj(pNtk, 0)->Level, Abc_NtkObj(pNtk, 0)->Id , Abc_ObjFanoutNum(Abc_NtkObj(pNtk, 0)));
        // }
        // Vec_VecForEachEntryReverseReverse(Abc_Obj_t *, pNtkLevelized, pObj, i, k)
        // {
        //     printf("The Node Id: %d, the Level %d, the Type %d\n", pObj->Id, pObj->Level, pObj->Type);
        // }
        // float Entry;
        // Vec_FltForEachEntry(p->vTimesOut, Entry, i)
        // {
        //     printf("The delay is: %f\n", Entry);
        // }
        // Abc_NtkForEachObj(pNtkNew, pObj, i)
        // {
        //     printf("Node Id: %d, Level: %d, Type: %d, Delay rise: %lf, fall %lf\n", pObj->Id, pObj->Level, pObj->Type, Abc_SclObjTime(p, pObj)->rise, Abc_SclObjTime(p, pObj)->fall );
        // }
        if (Abc_ObjFanoutNum(Abc_NtkObj(pNtk, 0)) == 0 && Abc_ObjFaninNum(Abc_NtkObj(pNtk, 0)) == 0 )
            idshift = -1; // shift everything -1 if const node is unused which always had id 0.
        Abc_NtkForEachPo(pNtk, pNode, i)
        {
            pDelays[pNode->Id+idshift] =  Abc_SclObjTimeMax(p, pNode);
            pFanin = Abc_ObjFanin0(pNode);
            pDelays[pFanin->Id+idshift] = Abc_MaxDouble( pDelays[pFanin->Id+idshift], pDelays[pNode->Id+idshift] );
        }
        Vec_VecForEachEntryReverseReverse(Abc_Obj_t * , pNtkLevelized, pNode, i, k )
        {
            assert (Abc_ObjIsNode(pNode)); // pNode is supposed to be of type Node
            pFanin = Abc_ObjFanin0(pNode);
            pDelays[pFanin->Id+idshift] = Abc_MaxDouble( pDelays[pNode->Id+idshift], pDelays[pFanin->Id+idshift] );
            pFanin = Abc_ObjFanin1(pNode);
            pDelays[pFanin->Id+idshift] = Abc_MaxDouble( pDelays[pNode->Id+idshift], pDelays[pFanin->Id+idshift] );
        }
        Abc_NtkForEachPi(pNtk, pNode, i)
        {
            pDelays[pNode->Id+idshift] = Abc_MaxDouble( pDelays[Abc_ObjFanout0(pNode)->Id+idshift], pDelays[pNode->Id+idshift] );
        }
    }

    // Abc_NtkForEachObj(pNtk, pObj, i)
    // {
    //     name = Nm_ManFindNameById(pNtk->pManName, pObj->Id);
    //     if (name)
    //     {
    //         printf( "The Id is: %d, the type is: %d, the name is %s\n",pObj->Id, pObj->Type, name );
    //     }
    //     else
    //         printf( "The Id is: %d, the type is: %d, the NumFanins: %d, the NumFanouts: %d\n",pObj->Id, pObj->Type, Abc_ObjFaninNum(pObj), Abc_ObjFanoutNum(pObj) );
    // }
    // Abc_NtkForEachObj(pNtkNew, pObj, i)
    // {
    //     name = Nm_ManFindNameById(pNtkNew->pManName, pObj->Id);
    //     if (name)
    //     {
    //         printf( "The Id is: %d, the type is: %d, the name is %s\n",pObj->Id, pObj->Type, name );
    //     }
    // }
    // if (pNtkNew->pManName == pNtk->pManName)
    //     printf( "The the name managers are the same! %p\n", (void*) pNtkNew->pManName);
    // else
    //     printf("The name managers are different! NtkNew: %p, Ntk: %p\n", (void*) pNtkNew->pManName, (void*) pNtk->pManName);

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
    char * name;
    pNtk = Abc_FrameReadNtk(pAbc);
    int LevelMax = Abc_NtkLevelReverse(pNtk);
    printf("The max level is %d", LevelMax);
    Abc_NtkForEachObj(pNtk, pObj, i){
        if (pObj){
            printf( "The id is: %d; the type is: %d, the level is: %d", pObj->Id, pObj->Type, pObj->Level );
            name = Nm_ManFindNameById(pNtk->pManName, pObj->Id);
            if (name){
                printf( " The name is: %s\n", name);
            }else {
                printf("\n");
            }
        }
        if (!pObj){
            printf("got here iteration is: %d", i);
        }
    }
}

void Abc_RLfLOGetNumObjs( Abc_Frame_t * pAbc, int * pObjNum ){
    Abc_Ntk_t * pNtk;
    pNtk = Abc_FrameReadNtk(pAbc);
    *pObjNum = Abc_NtkObjNum(pNtk);
    if ( Abc_ObjFaninNum(Abc_NtkObj(pNtk, 0))==0 && Abc_ObjFanoutNum(Abc_NtkObj(pNtk, 0))==0 )  // don't count the const 1 node if it isn't used
        *pObjNum -= 1;
    return;
}

void Abc_RLfLOGetObjTypes( Abc_Frame_t * pAbc, int * x)
{
    Abc_Ntk_t * pNtk;
    Abc_Obj_t * pObj;
    int i;
    int j = 0;
    pNtk = Abc_FrameReadNtk(pAbc);
    Abc_NtkForEachObj(pNtk, pObj, i){
        if ( Abc_ObjFaninNum(pObj)>0 || Abc_ObjFanoutNum(pObj)>0 ){
            x[j++] = pObj->Type;
        }
    }
    return;
}

void Abc_RLfLOGetNodeFeatures( Abc_Frame_t * pAbc, float * x, size_t n, size_t p)
{
    Abc_Ntk_t * pNtk;
    Abc_Obj_t * pObj;
    int i;
    int idshift = 0;
    int j = 0;
    assert(p==2); // wrong shape
    pNtk = Abc_FrameReadNtk(pAbc);
    if (Abc_ObjFanoutNum(Abc_NtkObj(pNtk, 0)) == 0 && Abc_ObjFaninNum(Abc_NtkObj(pNtk, 0)) == 0 )
        idshift = -1; // shift everything -1 if const node is unused which always has id 0.
    Abc_NtkForEachObj(pNtk, pObj, i){
        if ( Abc_ObjFaninNum(pObj)>0 || Abc_ObjFanoutNum(pObj)>0 ){
            assert(pObj->Id+idshift==j); // make sure the index corresponds to the ID including the idshift
            switch(pObj->Type)
            {   
                case ABC_OBJ_CONST1:
                    x[j*p] = 0;
                    break;
                case ABC_OBJ_PI:
                    x[j*p] = 1;
                    break;
                case ABC_OBJ_PO:
                    x[j*p] = 2;
                    break;
                case ABC_OBJ_NODE:
                    x[j*p] = 3;
                    break;
                default:
                    printf("The Object ID is: %d", pObj->Type);
                    x[j*p] = -pObj->Type;
                    assert(false); // It has to be one of the cases!
                    break;
            }
            x[j*p+1] = pObj->fCompl0 + pObj->fCompl1;
            assert( j*p+1 < n*p ); // make sure we are within the array
            j++;
        }
    }
    return;
}

void Abc_RLfLOGetNumEdges( Abc_Frame_t * pAbc, int * pNumEdges ){
    Abc_Ntk_t * pNtk;
    Abc_Obj_t * pObj;
    int i;
    *pNumEdges = 0;
    pNtk = Abc_FrameReadNtk(pAbc);
    Abc_NtkForEachObj(pNtk, pObj, i){
        if (pObj){
            *pNumEdges += Abc_ObjFaninNum(pObj);
        }
    }
    return;
}

void Abc_RLfLOGetEdges( Abc_Frame_t * pAbc, long * pEdges, int nEdges, float * pEdgeFeatures ){
    Abc_Ntk_t * pNtk;
    Abc_Obj_t * pObj, * pFanin;
    int i;
    int j;
    int y = 0;
    int idshift = 0;
    pNtk = Abc_FrameReadNtk(pAbc);
    assert( Abc_NtkIsStrash(pNtk) );
    Abc_NtkForEachObj(pNtk, pObj, i){
        if (Abc_ObjFanoutNum(pObj)==0 && Abc_ObjFaninNum(pObj)==0){
            assert( Abc_AigNodeIsConst(pObj) );
            assert( idshift==0);
            assert( i == 0 );
            idshift--;
        }
        Abc_ObjForEachFanin(pObj, pFanin, j){
            pEdges[y] = pFanin->Id + idshift;
            pEdges[nEdges + y] = pObj->Id + idshift;
            if (j == 0){
                pEdgeFeatures[y] = pObj->fCompl0;
            } else{
                assert(j == 1);
                pEdgeFeatures[y] = pObj->fCompl1;
            }
            y++;
            assert(j <= 1);
        }
    }
    assert(y==nEdges);
    return;
}

void Abc_RLfLONtkDesub( Abc_Frame_t * pAbc, int Id )
{
    Abc_Ntk_t * pNtk;
    Abc_Obj_t * pNode, * pFaninA, * pFaninB, * pFaninC, * pChild, *pABAC, *pABBC, * pTemp, *pAC, *pBC;
    Abc_Aig_t * pMan;
    pNtk = Abc_FrameReadNtk(pAbc);
    pNode = Abc_NtkObj(pNtk, Id);
    assert(Abc_NtkIsStrash(pNtk));
    if ( Abc_ObjLevel(pNode) >=2 ){
        pMan = (Abc_Aig_t *)pNtk->pManFunc;
        assert(Abc_ObjIsNode(pNode));
        pChild = Abc_ObjChild0(pNode);
        pFaninC = Abc_ObjChild1(pNode);
        if (!Abc_ObjIsNode(Abc_ObjRegular(pChild))) // If child0 is an Input child1 must be a node
        {
            pChild = Abc_ObjChild1(pNode);
            pFaninC = Abc_ObjChild0(pNode);
        }
        assert(Abc_ObjIsNode(Abc_ObjRegular(pChild)));
        pFaninA = Abc_ObjChild0(Abc_ObjRegular(pChild));
        pFaninB = Abc_ObjChild1(Abc_ObjRegular(pChild));
        if (!Abc_ObjIsComplement(pChild))
        {
            pAC = Abc_AigAnd(pMan, pFaninA, pFaninC);
            pBC = Abc_AigAnd(pMan, pFaninB, pFaninC);
            pABAC = Abc_AigAnd(pMan, pChild, pAC);
            pABBC = Abc_AigAnd(pMan, pChild, pBC);
            pTemp = Abc_AigAnd(pMan, pABAC, pABBC);
            Abc_AigReplace(pMan, pNode, pTemp, 1);
        }
        else
        {
            pAC = Abc_AigAnd(pMan, Abc_ObjNot(pFaninA), pFaninC);
            pBC = Abc_AigAnd(pMan, Abc_ObjNot(pFaninB), pFaninC);
            pABBC = Abc_AigAnd(pMan, Abc_ObjNot(pAC), Abc_ObjNot(pBC));
            Abc_AigReplaceSkipNew(pMan, pNode, Abc_AigAnd(pMan, pNode, Abc_ObjNot(pABBC)), 1);
        }
    }
    Abc_NtkReassignIds( pNtk );
    return;
}


////////////////////////////////////////////////////////////////////////
///                       END OF FILE                                ///
////////////////////////////////////////////////////////////////////////


ABC_NAMESPACE_IMPL_END

