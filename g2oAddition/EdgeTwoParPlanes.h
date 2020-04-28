//
// Created by fishmarch on 19-6-10.
//

#ifndef ORB_SLAM2_EDGETWOPARPLANES_H
#define ORB_SLAM2_EDGETWOPARPLANES_H

#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/hyper_graph_action.h"
#include "Thirdparty/g2o/g2o/core/eigen_types.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/stuff/misc.h"
#include "g2oAddition/Plane3D.h"
#include "g2oAddition/VertexPlane.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace g2o {
    class EdgeTwoParPlanes : public BaseBinaryEdge<2, double, VertexPlane, VertexPlane> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeTwoParPlanes(){}
        void computeError()
        {
            const VertexPlane* planeVertex2 = static_cast<const VertexPlane*>(_vertices[1]);
            const VertexPlane* planeVertex1 = static_cast<const VertexPlane*>(_vertices[0]);

            Plane3D plane1 = planeVertex1->estimate();
            const Plane3D& plane2 = planeVertex2->estimate();
            // measurement function: remap the plane in global coordinates
            _error = plane1.ominus_par(plane2);
        }


        virtual bool read(std::istream& is) {}
        virtual bool write(std::ostream& os) const{}
    };
}

#endif //ORB_SLAM2_EDGETWOPARPLANES_H
