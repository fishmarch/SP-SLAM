//
// Created by fishmarch on 19-6-8.
//

#ifndef ORB_SLAM2_EDGEPARALLELPLANE_H
#define ORB_SLAM2_EDGEPARALLELPLANE_H

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
    class EdgeParallelPlane : public BaseBinaryEdge<2, Plane3D, VertexPlane, VertexSE3Expmap> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeParallelPlane();
        void computeError()
        {
            const VertexSE3Expmap* poseVertex = static_cast<const VertexSE3Expmap*>(_vertices[1]);
            const VertexPlane* planeVertex = static_cast<const VertexPlane*>(_vertices[0]);

            const Plane3D& plane = planeVertex->estimate();
            // measurement function: remap the plane in global coordinates
            Isometry3D w2n = poseVertex->estimate();
            Plane3D localPlane = w2n*plane;

            _error = localPlane.ominus_par(_measurement);
        }

        void setMeasurement(const Plane3D& m){
            _measurement = m;
        }

        bool isDepthPositive(){
            const VertexSE3Expmap* poseVertex = static_cast<const VertexSE3Expmap*>(_vertices[1]);
            const VertexPlane* planeVertex = static_cast<const VertexPlane*>(_vertices[0]);

            const Plane3D& plane = planeVertex->estimate();
            // measurement function: remap the plane in global coordinates
            Isometry3D w2n = poseVertex->estimate();
            Plane3D localPlane = w2n*plane;

            return localPlane.distance() > 0;
        }

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;
    };
}


#endif //ORB_SLAM2_EDGEPARALLELPLANE_H
