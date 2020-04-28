//
// Created by fishmarch on 19-5-28.
//

#include "VertexPlane.h"

namespace g2o{
    VertexPlane::VertexPlane(){
    }

    bool VertexPlane::read(std::istream& is) {
        Vector4D lv;
        for (int i=0; i<4; i++)
            is >> lv[i];
        setEstimate(Plane3D(lv));
        return true;
    }

    bool VertexPlane::write(std::ostream& os) const {
        Vector4D lv=_estimate.toVector();
        for (int i=0; i<4; i++){
            os << lv[i] << " ";
        }
        return os.good();
    }
}