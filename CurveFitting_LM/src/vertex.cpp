//
// Created by alex on 2020/12/11.
//

#include "mysolver/vertex.h"

namespace mysolver{

    unsigned long global_vertex_id = 0;


    Vertex::Vertex(int num_dimension, int local_dimension) {
        parameters_.resize(num_dimension, 1);
        local_dimension_ = local_dimension > 0 ? local_dimension : num_dimension;
        id_ = global_vertex_id++;
    }

    Vertex::~Vertex() {}




    void Vertex::Plus(const VecX &delta) {
        parameters_ += delta;
    }


}




