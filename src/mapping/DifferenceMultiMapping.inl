/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2019 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include <sofa/core/Multi2Mapping.inl>
#include "DifferenceMultiMapping.h"
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <string>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/core/objectmodel/BaseContext.h>

#ifndef ISSOFA_VERSION
#include <sofa/helper/logging/Message.h>
#include <sofa/helper/types/RGBAColor.h>
#endif


namespace sofa
{
namespace component
{
namespace mapping
{
using sofa::core::objectmodel::BaseContext ;
using sofa::helper::AdvancedTimer;
using sofa::helper::WriteAccessor;

template <class TIn1, class TIn2, class TOut>
DifferenceMultiMapping<TIn1, TIn2, TOut>::DifferenceMultiMapping()
    : d_direction(initData(&d_direction, "direction","The list of directions of fix points .\n"))
    , d_indices(initData(&d_indices, "indices","Indices of fixe points of the cable"))
    , d_raduis(initData(&d_raduis, 2.0, "radius","The size of the cable"))
    , d_color(initData(&d_color,defaulttype::Vec4f(1,0,0,1), "color","The color of the cable"))
    , d_drawArrows(initData(&d_drawArrows,false, "drawArrows","The color of the cable"))
    , m_fromModel1(NULL)
    , m_fromModel2(NULL)
    , m_toModel(NULL)
{

}


template <class TIn1, class TIn2, class TOut>
void DifferenceMultiMapping<TIn1, TIn2, TOut>::initiatTopologies()
{
    m_toModel = this->getToModels()[0];
    if (! m_toModel) {
        std::cout << " No output mechanical state found. Consider setting the "
                  << this->toModels.getName() << " attribute."<< std::endl;
        return;
    }


    if (!d_direction.isSet())
        msg_warning()<<"No direction nor indiece is given.";


    //    unsigned int szIndices =  d_indices.getValue().size();
    //    unsigned int szdirection =  d_direction.getValue().size();
    //    if(!(szIndices == szdirection))
    //        msg_warning()<< "The size of the list of indes is != the the size of directions list, plese fixe this";


}


// _________________________________________________________________________________________

template <class TIn1, class TIn2, class TOut>
void DifferenceMultiMapping<TIn1, TIn2, TOut>::init()
{
    if(this->getFromModels1().empty())
    {
        msg_error() << "Error while initializing ; input getFromModels1 not found" ;
        return;
    }

    if(this->getFromModels2().empty())
    {
        msg_error() << "Error while initializing ; output getFromModels2 not found" ;
        return;
    }

    if(this->getToModels().empty())
    {
        msg_error() << "Error while initializing ; output Model not found" ;
        //return;
    }
    m_fromModel1 = this->getFromModels1()[0];
    m_fromModel2 = this->getFromModels2()[0];
    m_toModel = this->getToModels()[0];

    m_toModel = m_fromModel1;

    initiatTopologies();
}


template <class TIn1, class TIn2, class TOut>
void DifferenceMultiMapping<TIn1, TIn2, TOut>::bwdInit()
{

}

template <class TIn1, class TIn2, class TOut>
void DifferenceMultiMapping<TIn1, TIn2, TOut>::reinit()
{

}

template <class TIn1, class TIn2, class TOut>
void DifferenceMultiMapping<TIn1, TIn2, TOut>::reset()
{
    reinit();
}


template <class TIn1, class TIn2, class TOut>
void DifferenceMultiMapping<TIn1, TIn2, TOut>::computeProximity(const In1VecCoord &x1, const In2VecCoord &x2){

    In1VecCoord from = x1;
    In2VecCoord dst  = x2;
    m_constraints.clear();

    size_t szFrom = from.size();
    size_t szDst = dst.size();
    helper::vector<Rigid> direction = d_direction.getValue();

    ///get the last rigige direction, the main goal is to use it for the
    /// 3D bilaterale constraint i.e the fix point of the cable in the robot structure
    //Rigid direction = d_direction.getValue()[szDst-1];

    //For each point in the FEM find the closest edge of the cable
    for (size_t i = 0 ; i < szFrom; i++) {
        Coord2 P = from[i];
        Constraint constraint;

        // find the min distance between a from mstate point and it's projection on each edge of the cable (destination mstate)
        Real min_dist = std::numeric_limits<Real>::max();
        Real min_dist2 = std::numeric_limits<Real>::max();
        for (size_t j = 0; j < szDst-1; j++) {
            Coord1 Q1 = dst[j];
            Coord1 Q2 = dst[j+1];

            //the axis
            Coord1 dirAxe = Q2 -Q1;
            Real length = dirAxe.norm();
            Real fact_v = dot(P-Q1,dirAxe) / dot(dirAxe,dirAxe) ;

            if(std::abs(fact_v) < min_dist){
                //if(fact_v < min_dist){
                min_dist = std::abs(fact_v) ;

                //define the constraint variables
                Deriv1 proj, distVec;
                Real alpha, dist;

                ///To solve the case that the closest node is
                /// not the node 0 but the node 1 of the beam
                if(fact_v<0.0 && j!=0 && std::abs(fact_v) > 1e-8){
                    //if fact_v < 0.0 that means the last beam is the good beam
                    //printf("if fact_v < 0.0 that means the last beam is the good beam \n");
                    Q1 = dst[j-1] ;
                    dirAxe = dst[j] - Q1;
                    length = dirAxe.norm();
                    fact_v = dot(P-Q1,dirAxe) / dot(dirAxe,dirAxe) ;
                    dirAxe.normalize();
                    alpha = (P-Q1) * dirAxe;

                    proj = Q1 + dirAxe * alpha;
                    distVec = P - proj; // violation vector
                    dist = distVec.norm(); // constraint violation
                    constraint.eid = j-1;
                    //The direction of the axe or the beam
                    constraint.dirAxe = dirAxe;
                    //the node contribution to the constraint which is 1-coeff
                    alpha = alpha / length; //normalize, ensure that <1.0
                    if (alpha < 1e-8)constraint.alpha = 1.0 ;
                    else constraint.alpha = 1.0 - alpha;

                    //The projection on the axe
                    constraint.proj = proj;
                    constraint.Q = from[i];

                    /////
                    length = (dst[j] - Q1).norm();
                    constraint.Q1Q2 = length;
                    constraint.r2 = fact_v;

                    // We move the constraint point onto the projection
                    Deriv1 t1 = P - proj; // violation vector
                    constraint.dist = t1.norm(); // constraint violation
                    t1.normalize(); // direction of the constraint

                    //// First method compute normals using projections
                    //                    if(t1.norm()<1.0e-1 && dirAxe[2] < 0.99){
                    //                        Vector3 temp = Vector3(dirAxe[0],dirAxe[1],dirAxe[2]+50.0);
                    //                        t1 = cross(dirAxe,temp);
                    //                        t1.normalize();
                    //                        constraint.t1 = t1;
                    //                    }
                    //                    if(t1.norm()<1.0e-1){
                    //                        Vector3 temp = Vector3(dirAxe[0],dirAxe[1]+50.0,dirAxe[2]);
                    //                        t1 = cross(dirAxe,temp);
                    //                        t1.normalize();
                    //                        constraint.t1 = t1;
                    //                    }

                    //                    if(t1.norm()<1.0e-1)
                    //                    {

                    //// Second method compute normals using frames directions
                    Rigid dir = direction[constraint.eid];
                    Vector3 vY = Vector3(0.,1.,0.);
                    defaulttype::Quat ori = dir.getOrientation() ;
                    vY = ori.rotate(vY); vY.normalize();
                    t1 = vY ;
                    //                    }

                    constraint.t1 = t1;
                    //tangential 2
                    Deriv1 t2 = cross(t1, dirAxe);  t2.normalize();
                    constraint.t2 = t2;


                    if(i == szFrom-1){
                        ///This handle the fix point constraint the last point of
                        /// of cstr points indeed here we have
                        /// 3D bilaterale constraint and alpha=1.0
                        // We use the given direction of fill H

                        if (!direction.empty()){
                            Rigid dir = direction[szDst-1];
                            Vector3 vY = Vector3(0.,1.,0.);
                            Vector3 vZ = Vector3(0.,0.,1.);
                            defaulttype::Quat ori = dir.getOrientation() ;
                            vY = ori.rotate(vY); vY.normalize();
                            vZ = ori.rotate(vZ); vZ.normalize();
                            //msg_info("1 debug :")<< " ====> t1 :"<< vY;
                            //msg_info("1 debug :")<< " ====> t2 :"<< vZ;
                            constraint.t1 = vY ;
                            constraint.t2 = vZ ;
                        }
                        constraint.proj = dst[szDst-1];
                        constraint.eid = szDst-2;
                        constraint.alpha = 1.0;
                        constraint.dist = (dst[szDst-1] - from[szFrom-1]).norm();
                    }
                }else{
                    //compute needs for constraint
                    dirAxe.normalize();
                    alpha = (P-Q1) * dirAxe;

                    proj = Q1 + dirAxe * alpha;
                    distVec = P - proj; // violation vector
                    dist = distVec.norm(); // constraint violation
                    constraint.eid = j;
                    //The direction of the axe or the beam
                    constraint.dirAxe = dirAxe;
                    //the node contribution to the constraint which is 1-coeff
                    alpha = alpha / length; //normalize, ensure that <1.0
                    if (alpha < 1e-8)constraint.alpha = 1.0 ;
                    else constraint.alpha = 1.0 - alpha;

                    //The projection on the axe
                    constraint.proj = proj;
                    constraint.Q = from[i];

                    /////
                    constraint.Q1Q2 = length;
                    constraint.r2 = fact_v;

                    // We move the constraint point onto the projection
                    Deriv1 t1 = P - proj; // violation vector
                    constraint.dist = t1.norm(); // constraint violation
                    t1.normalize(); // direction of the constraint

                    /// If the violation is very small t1 is close to zero
                    ///
                    //// First method compute normals using projections
                    //                    if(t1.norm()<1.0e-1 && dirAxe[2] < 0.99){
                    //                        Vector3 temp = Vector3(dirAxe[0],dirAxe[1],dirAxe[2]+50.0);
                    //                        t1 = cross(dirAxe,temp);
                    //                        t1.normalize();
                    //                        constraint.t1 = t1;
                    //                    }
                    //                    if(t1.norm()<1.0e-1){
                    //                        Vector3 temp = Vector3(dirAxe[0],dirAxe[1]+50.0,dirAxe[2]);
                    //                        t1 = cross(dirAxe,temp);
                    //                        t1.normalize();
                    //                        constraint.t1 = t1;
                    //                    }

                    //// Second method compute normals using frames directions
                    Rigid dir = direction[constraint.eid];
                    Vector3 vY = Vector3(0.,1.,0.);
                    defaulttype::Quat ori = dir.getOrientation() ;
                    vY = ori.rotate(vY); vY.normalize();
                    t1 = vY ;
                    //                    }
                    constraint.t1 = t1;
                    //tangential 2
                    Deriv1 t2 = cross(t1, dirAxe);  t2.normalize();
                    constraint.t2 = t2;

                    ///This is need because we are applying the a
                    /// billateral constraint on the last node of the mstate
                    if(i == szFrom-1){
                        ///This handle the fix point constraint the last point of
                        /// of cstr points indeed here we have
                        /// 3D bilaterale constraint and alpha=1.0
                        // We use the given direction of fill H
                        if (!d_direction.getValue().empty()){
                            Rigid dir = direction[szDst-1];
                            Vector3 vY = Vector3(0.,1.,0.);
                            Vector3 vZ = Vector3(0.,0.,1.);
                            defaulttype::Quat ori = dir.getOrientation() ;
                            vY = ori.rotate(vY); vY.normalize();
                            vZ = ori.rotate(vZ); vZ.normalize();
                            //msg_info("1 debug :")<< " ====> t1 :"<< vY;
                            //msg_info("1 debug :")<< " ====> t2 :"<< vZ;
                            constraint.t1 = vY ;
                            constraint.t2 = vZ ;
                        }
                        constraint.proj = dst[szDst-1];
                        constraint.eid = szDst-2;
                        constraint.alpha = 1.0;
                        constraint.dist = (dst[szDst-1] - from[szFrom-1]).norm();
                    }
                }
            }
        }
        //        printf("______________________________________________________________\n");
        //        std::cout << "i :" << i << " ; eid:" << constraint.eid << " alpha : " << constraint.alpha << " ;  dist :"<< constraint.dist << std::endl;
        //        std::cout<<" fact_v :"<< constraint.r2 << i << " ; n :"<< constraint.dirAxe << "; t1:" << constraint.t1 << "; t2 :"<<  constraint.t2 << std::endl;
        //        printf("______________________________________________________________\n");
        m_constraints.push_back(constraint);
    }
}


template <class TIn1, class TIn2, class TOut>
void DifferenceMultiMapping<TIn1, TIn2, TOut>::apply(
        const core::MechanicalParams* /* mparams */, const helper::vector<OutDataVecCoord*>& dataVecOutPos,
        const helper::vector<const In1DataVecCoord*>& dataVecIn1Pos ,
        const helper::vector<const In2DataVecCoord*>& dataVecIn2Pos)
{

    if(dataVecOutPos.empty() || dataVecIn1Pos.empty() || dataVecIn2Pos.empty())
        return;

    //printf("///Do Apply//We need only one input In model and input Root model (if present) \n");
    const In1VecCoord& in1 = dataVecIn1Pos[0]->getValue();
    const In2VecCoord& in2 = dataVecIn2Pos[0]->getValue();

    computeProximity(in1,in2);

    OutVecCoord& out = *dataVecOutPos[0]->beginEdit();
    //auto out = sofa::helper::writeOnly(*dataVecOutPos[0]);
    size_t sz = m_constraints.size();
    out.resize(sz);

    for(unsigned int i=0; i<sz; i++){
        Constraint& c = m_constraints[i];
        if(i< sz-1){
            out[i][0] = 0.0;
            out[i][1] = c.t1 * (in1[i] - c.proj)  ; //c.dist;
            out[i][2] = c.t2 * (in1[i] - c.proj); //0.0
        }else{
            Real dist= (in2[in2.size()-1] - in1[in1.size()-1]).norm();
            out[sz-1][0] = c.dirAxe * (in1[in1.size()-1] - in2[in2.size()-1]);
            out[sz-1][1] = c.t1     * (in1[in1.size()-1] - in2[in2.size()-1]); //std::abs(in2[in2.size()-1][1] - in1[in1.size()-1][1]);
            out[sz-1][2] = c.t2     * (in1[in1.size()-1] - in2[in2.size()-1]); //std::abs(in2[in2.size()-1][2] - in1[in1.size()-1][2]);
        }
    }
    dataVecOutPos[0]->endEdit();
}



template <class TIn1, class TIn2, class TOut>
void DifferenceMultiMapping<TIn1, TIn2, TOut>:: applyJ(
        const core::MechanicalParams* /* mparams */, const helper::vector< OutDataVecDeriv*>& dataVecOutVel,
        const helper::vector<const In1DataVecDeriv*>& dataVecIn1Vel,
        const helper::vector<const In2DataVecDeriv*>& dataVecIn2Vel) {
    if(dataVecOutVel.empty() || dataVecIn1Vel.empty() ||dataVecIn2Vel.empty() )
        return;
    const In1VecDeriv& in1 = dataVecIn1Vel[0]->getValue();
    const In2VecDeriv& in2 = dataVecIn2Vel[0]->getValue();
    OutVecDeriv& outVel = *dataVecOutVel[0]->beginEdit();

    //const OutVecCoord& out = m_toModel->read(core::ConstVecCoordId::position())->getValue();
    size_t sz = m_constraints.size();
    outVel.resize(sz);
    for (size_t i = 0 ; i < sz; i++) {
        Constraint& c = m_constraints[i];
        int ei1 = c.eid;
        int ei2 = c.eid+1;
        if(i < sz-1){
            // std::cout << " ei1 : " << ei1 << " ei2 : "<< ei2 << std::endl;
            Real v0 = c.dirAxe * (in1[i] - c.alpha * in2[ei1] - (1-c.alpha) * in2[ei2] );
            Real v1 = c.t1     * (in1[i] - c.alpha * in2[ei1] - (1-c.alpha) * in2[ei2] );
            Real v2 = c.t2     * (in1[i] - c.alpha * in2[ei1] - (1-c.alpha) * in2[ei2] );
            outVel[i] = OutDeriv(v0,v1,v2);
        }else {
            //std::cout << " i : " << i << " ei2 : "<< ei2 << std::endl;
            Real v0 = c.dirAxe * (in1[i] - in2[ei2]);
            Real v1 = c.t1     * (in1[i] - in2[ei2]);
            Real v2 = c.t2     * (in1[i] - in2[ei2]);
            outVel[i] = OutDeriv(v0,v1,v2);
        }
    }
    dataVecOutVel[0]->endEdit();
}


template <class TIn1, class TIn2, class TOut>
void DifferenceMultiMapping<TIn1, TIn2, TOut>::applyJT(
        const core::MechanicalParams* /*mparams*/, const helper::vector< In1DataVecDeriv*>& dataVecOut1Force,
        const helper::vector< In2DataVecDeriv*>& dataVecOut2Force,
        const helper::vector<const OutDataVecDeriv*>& dataVecInForce)  {

    if(dataVecOut1Force.empty() || dataVecInForce.empty() || dataVecOut2Force.empty())
        return;

    const OutVecDeriv& in = dataVecInForce[0]->getValue();

    In1VecDeriv& out1 = *dataVecOut1Force[0]->beginEdit();
    In2VecDeriv& out2 = *dataVecOut2Force[0]->beginEdit();

    //Compute output forces
    size_t sz = m_constraints.size();

    for (size_t i = 0 ; i < sz; i++) {
        Constraint& c = m_constraints[i];
        int ei1 = c.eid;
        int ei2 = c.eid+1;
        OutDeriv f = in[i];
        //std::cout << " ================+++++++++>>>>> The force : " << f << std::endl;
        if(i < sz-1){
            Deriv2 f1   = (f[0] * c.dirAxe) + (f[1] * c.t1) + (f[2] * c.t2) ;
            Deriv1 f2_1   = (c.alpha * f[0]*c.dirAxe) + (c.alpha* f[1] * c.t1) + (c.alpha * f[2] * c.t2);
            Deriv1 f2_2 = ((1-c.alpha) * f[0]*c.dirAxe )+ ((1-c.alpha) * f[1]*c.t1) + ((1-c.alpha) * f[2]*c.t2);

            //std::cout << " f1 : " << f1 << "   f&_1: " << f2_1 << " ; f2 : "<< f2 << std::endl;

            out1[i]   += f1;
            out2[ei1] -= f2_1;
            out2[ei2] -= f2_2;
        }else{
            Deriv2 f = (f[0] * c.dirAxe) + (f[1] * c.t1) + (f[2] * c.t2) ;
            out1[i] += f;
            out2[ei2] -= f;
        }
    }
    dataVecOut1Force[0]->endEdit();
    dataVecOut2Force[0]->endEdit();
}

//___________________________________________________________________________
template <class TIn1, class TIn2, class TOut>
void DifferenceMultiMapping<TIn1, TIn2, TOut>::applyJT(
        const core::ConstraintParams* /*cparams*/ , const helper::vector< In1DataMatrixDeriv*>&  dataMatOut1Const,
        const helper::vector< In2DataMatrixDeriv*>&  dataMatOut2Const ,
        const helper::vector<const OutDataMatrixDeriv*>& dataMatInConst)
{
    if(dataMatOut1Const.empty() || dataMatOut2Const.empty() || dataMatInConst.empty() )
        return;


    //We need only one input In model and input Root model (if present)
    In1MatrixDeriv& out1 = *dataMatOut1Const[0]->beginEdit(); // constraints on the FEM cable points
    In2MatrixDeriv& out2 = *dataMatOut2Const[0]->beginEdit(); // constraints on the frames cable points
    const OutMatrixDeriv& in = dataMatInConst[0]->getValue(); // input constraints defined on the mapped point
    const OutVecCoord& mappedPoints = m_toModel->read(core::ConstVecCoordId::position())->getValue();
    const In1DataVecCoord* x1fromData = m_fromModel1->read(core::ConstVecCoordId::position());
    const In1VecCoord x1from = x1fromData->getValue();

    typename OutMatrixDeriv::RowConstIterator rowIt    = in.begin()  ;
    typename OutMatrixDeriv::RowConstIterator rowItEnd = in.end();

    for (rowIt = in.begin(); rowIt != rowItEnd; ++rowIt)
    {
        typename OutMatrixDeriv::ColConstIterator colIt = rowIt.begin();
        typename OutMatrixDeriv::ColConstIterator colItEnd = rowIt.end();

        // Creates a constraints if the input constraint is not empty.
        if (colIt == colItEnd)
        {
            continue;
        }
        typename In1MatrixDeriv::RowIterator o1 = out1.writeLine(rowIt.index()); // we store the constraint number
        typename In2MatrixDeriv::RowIterator o2 = out2.writeLine(rowIt.index());

        if((rowIt.index()/2) < (x1from.size() -1)){
            while (colIt != colItEnd)
            {
                int childIndex = colIt.index();
                Constraint c = m_constraints[childIndex];
                const OutDeriv h = colIt.val();
                int indexBeam =  c.eid;

                Deriv2 h1 = (h[0] * c.dirAxe) + (h[1] * c.t1) + (h[2] * c.t2) ;
                Deriv1 h2_1 = (c.alpha * h[0]*c.dirAxe) + (c.alpha* h[1] * c.t1) + (c.alpha * h[2] * c.t2);
                Deriv1 h2_2 = ((1.0-c.alpha) * h[0]*c.dirAxe )+ ((1.0-c.alpha) * h[1]*c.t1) + ((1.0-c.alpha) * h[2]*c.t2);

                //std::cout << " ==> t1 : "<< c.t1 << " ==> t2 : "<< c.t2 << std::endl;
                //std::cout << " ===> h1 : " << h1<< " ===> h2 : " << h2 << " ===> h2_1 : " << h2_1 << std::endl;

                o1.addCol(childIndex, h1);
                o2.addCol(indexBeam, -h2_1);
                o2.addCol(indexBeam+1, -h2_2);

                colIt++;
            }
        }else{
            while (colIt != colItEnd)
            {
                int childIndex = colIt.index();
                Constraint c = m_constraints[childIndex];
                const OutDeriv h = colIt.val();
                int indexBeam =  c.eid;

                Deriv2 h1 = (h[0] * c.dirAxe) + (h[1] * c.t1) + (h[2] * c.t2) ;
                Deriv1 h2 = (h[0] * c.dirAxe) + (h[1] * c.t1) + (h[2] * c.t2);

                o1.addCol(childIndex, h1);
                o2.addCol(indexBeam+1, -h2);
                colIt++;
            }
        }
    }

    dataMatOut1Const[0]->endEdit();
    dataMatOut2Const[0]->endEdit();
}


template <class TIn1, class TIn2, class TOut>
void DifferenceMultiMapping<TIn1, TIn2, TOut>::draw(const core::visual::VisualParams* vparams)
{

    ///draw cable
    ///
    //    const In2DataVecDeriv* xfromData = m_toModel->read(core::ConstVecCoordId::position());
    //    const In2VecCoord& postions = xfromData[0].getValue();
    //    unsigned int sz = postions.size();
    //    //    msg_info("DRAW")<< "The size of object is : "<< sz;

    //    double radius = d_raduis.getValue();
    //    vparams->drawTool()->drawLineStrip(postions,8.8,defaulttype::Vec4f(1.,0.,0.,1));

    //printf("CosseratSlidingConstraint<DataTypes>::draw(const core::visual::VisualParams* vparams) before \n");
    if (!vparams->displayFlags().getShowInteractionForceFields())
        return;

#ifndef ISSOFA_VERSION
    vparams->drawTool()->saveLastState();

    vparams->drawTool()->disableLighting();
#endif

    std::vector<sofa::defaulttype::Vector3> vertices;

    //    vparams->drawTool()->drawLines(vertices, 1, color);
    sofa::defaulttype::Vec4f color = sofa::defaulttype::Vec4f(0.f, 1.f, 0.f, 1.f);

    sofa::defaulttype::Vec4f colorL = d_color.getValue();
    if(d_drawArrows.getValue()){
        for (size_t i =0 ; i < m_constraints.size(); i++) {
            //        std::cout << " Projection : "<< m_constraints[i].proj << " ;  Q :" << m_constraints[i].Q << std::endl;
            vertices.push_back(m_constraints[i].proj);
            vertices.push_back(m_constraints[i].Q);
            vparams->drawTool()->drawLines(vertices, 4.0, color);
            if(i==(m_constraints.size()-1)){
                Coord2 P1 = m_constraints[i].Q;
                Real radius_arrow = 0.30;
                Coord2 x = m_constraints[i].dirAxe * 5.0;
                Coord2 y = m_constraints[i].t1 * 5.0;
                Coord2 z = m_constraints[i].t2 * 5.0;
                vparams->drawTool()->drawArrow(P1,P1 + x, radius_arrow, defaulttype::Vec<4,Real>(0,0,1,1));
                vparams->drawTool()->drawArrow(P1,P1 + y, radius_arrow, defaulttype::Vec<4,Real>(0,0,1,1));
                vparams->drawTool()->drawArrow(P1,P1 + z, radius_arrow, defaulttype::Vec<4,Real>(0,0,1,1));

            }else{
                Coord2 P1 = m_constraints[i].Q;
                Real radius_arrow = 0.30;
                Coord2 x = m_constraints[i].dirAxe * 5.0;
                Coord2 y = m_constraints[i].t1 * 5.0;
                Coord2 z = m_constraints[i].t2 * 5.0;
                vparams->drawTool()->drawArrow(P1,P1 + y, radius_arrow, defaulttype::Vec<4,Real>(0,0,1,1));
                vparams->drawTool()->drawArrow(P1,P1 + z, radius_arrow, defaulttype::Vec<4,Real>(0,0,1,1));
            }
        }
        const In1DataVecDeriv* xDestData = m_fromModel1->read(core::ConstVecCoordId::position());
        const In1VecCoord& fromPos = xDestData[0].getValue();
        //        msg_info("DRAW")<< "The size of object is : "<< postions.size();
#ifndef ISSOFA_VERSION
        vparams->drawTool()->draw3DText_Indices(fromPos,6,defaulttype::Vec<4,Real>(0,2,0,1));
#endif
    }


    //    for(unsigned int j = 0; j<sz-1; j++){
    //        vparams->drawTool()->drawLine(postions[j],postions[j+1],sofa::defaulttype::Vec4f(colorL[0],colorL[1],colorL[2],radius));
    //    }
#ifndef ISSOFA_VERSION
    vparams->drawTool()->restoreLastState();
#endif
}

}
}
} // namespace sofa
