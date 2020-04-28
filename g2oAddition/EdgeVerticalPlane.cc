//
// Created by fishmarch on 19-6-8.
//

#include "EdgeVerticalPlane.h"
namespace g2o {
    EdgeVerticalPlane::EdgeVerticalPlane(){

    }

    bool EdgeVerticalPlane::read(std::istream &is) {
        Vector4D v;
        is >> v(0) >> v(1) >> v(2) >> v(3);
        setMeasurement(Plane3D(v));
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j) {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;
    }

    bool EdgeVerticalPlane::write(std::ostream &os) const {
        Vector4D v = _measurement.toVector();
        os << v(0) << " " << v(1) << " " << v(2) << " " << v(3) << " ";
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j)
                os << " " << information()(i, j);
        return os.good();
    }
}