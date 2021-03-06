/* This file is part of PyMesh. Copyright (c) 2015 by Qingnan Zhou */
#pragma once

#include <memory>
#include <string>
#include <Core/Exception.h>
#include <Lattice2D.h>

namespace PyMesh {
/**
 * SkeletonEngine defines the minimalistic interface that compute the
 * skeleton of given closed enclosure or contour.
 * only 2D is supported at the moment.
 * TODO: Add support for 3D contour(perhaps mesh based)
 */
class SkeletonEngine{
    public:
        typedef std::shared_ptr<SkeletonEngine> Ptr;
        static Ptr create(size_t dim, const std::string& library_name);
        static bool supports(const std::string& library_name);
        static std::vector<std::string> get_available_engines();

    public:
        virtual ~SkeletonEngine(){}

        virtual void run(const MatrixFr& points,const std::vector<MatrixFr>& holes) {
            throw NotImplementedError("This function is not implemented");
        }

        Lattice2D::Ptr get_lattice() const { return m_lattice; }
    protected:
        Lattice2D::Ptr m_lattice;
};

} //namespace PyMesh