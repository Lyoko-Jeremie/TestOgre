// jeremie

#ifndef TESTOGRE_VERTEXINDEXTOSHAPE_H
#define TESTOGRE_VERTEXINDEXTOSHAPE_H

#include "./BulletMemoryContainer.h"
#include "btBulletDynamicsCommon.h"
#include "Ogre.h"

namespace Ogre::Bullet {

    class VertexIndexToShape {
    public:
        explicit VertexIndexToShape(const Affine3 &transform = Affine3::IDENTITY);

        explicit VertexIndexToShape(Renderable *rend, const Affine3 &transform = Affine3::IDENTITY);

        explicit VertexIndexToShape(const Entity *entity, const Affine3 &transform = Affine3::IDENTITY);

        ~VertexIndexToShape();

        Real getRadius();

        Vector3 getSize();

        boost::shared_ptr<btBvhTriangleMeshShape> createTrimesh(
                const boost::shared_ptr<BulletMemoryContainer::BulletMemoryContainerManager>& memoryContainerManager
        );

        boost::shared_ptr<btConvexHullShape> createConvex(
                const boost::shared_ptr<BulletMemoryContainer::BulletMemoryContainerManager>& memoryContainerManager
        );

        void addEntity(const Entity *entity, const Affine3 &transform = Affine3::IDENTITY);

        void addMesh(const MeshPtr &mesh, const Affine3 &transform = Affine3::IDENTITY);

        const Vector3 *getVertices() { return mVertexBuffer; }

        unsigned int getVertexCount() { return mVertexCount; };

    private:
        void addStaticVertexData(const VertexData *vertex_data);

        void addAnimatedVertexData(const VertexData *vertex_data, const VertexData *blended_data,
                                   const Mesh::IndexMap *indexMap);

        void addIndexData(IndexData *data, const unsigned int offset = 0);

        Vector3 *mVertexBuffer;
        unsigned int *mIndexBuffer;
        unsigned int mVertexCount;
        unsigned int mIndexCount;

        Vector3 mBounds;
        Real mBoundRadius;

        typedef std::map<unsigned char, std::vector<Vector3> *>
                BoneIndex;
        BoneIndex *mBoneIndex;

        Affine3 mTransform;

        Vector3 mScale;
    };

} // Bullet

#endif //TESTOGRE_VERTEXINDEXTOSHAPE_H
