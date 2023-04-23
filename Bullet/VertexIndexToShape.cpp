// jeremie

#include "VertexIndexToShape.h"

namespace Ogre::Bullet {

    typedef std::vector<Vector3> Vector3Array;
    typedef std::pair<unsigned short, Vector3Array *> BoneKeyIndex;

    inline btQuaternion convert(const Quaternion &q) { return btQuaternion(q.x, q.y, q.z, q.w); }

    inline btVector3 convert(const Vector3 &v) { return btVector3(v.x, v.y, v.z); }

    inline Quaternion convert(const btQuaternion &q) { return Quaternion(q.w(), q.x(), q.y(), q.z()); }

    inline Vector3 convert(const btVector3 &v) { return Vector3(v.x(), v.y(), v.z()); }

/*
 * =============================================================================================
 * BtVertexIndexToShape
 * =============================================================================================
 */

    void VertexIndexToShape::addStaticVertexData(const VertexData *vertex_data) {
        if (!vertex_data)
            return;

        const VertexData *data = vertex_data;

        const unsigned int prev_size = mVertexCount;
        mVertexCount += (unsigned int) data->vertexCount;

        Vector3 *tmp_vert = new Vector3[mVertexCount];
        if (mVertexBuffer) {
            memcpy(tmp_vert, mVertexBuffer, sizeof(Vector3) * prev_size);
            delete[] mVertexBuffer;
        }
        mVertexBuffer = tmp_vert;

        // Get the positional buffer element
        {
            const VertexElement *posElem = data->vertexDeclaration->findElementBySemantic(VES_POSITION);
            HardwareVertexBufferSharedPtr vbuf = data->vertexBufferBinding->getBuffer(posElem->getSource());
            const unsigned int vSize = (unsigned int) vbuf->getVertexSize();

            unsigned char *vertex = static_cast<unsigned char *>(vbuf->lock(HardwareBuffer::HBL_READ_ONLY));
            float *pReal;
            Vector3 *curVertices = &mVertexBuffer[prev_size];
            const unsigned int vertexCount = (unsigned int) data->vertexCount;
            for (unsigned int j = 0; j < vertexCount; ++j) {
                posElem->baseVertexPointerToElement(vertex, &pReal);
                vertex += vSize;

                curVertices->x = (*pReal++);
                curVertices->y = (*pReal++);
                curVertices->z = (*pReal++);

                *curVertices = mTransform * (*curVertices);

                curVertices++;
            }
            vbuf->unlock();
        }
    }

//------------------------------------------------------------------------------------------------
    void VertexIndexToShape::addAnimatedVertexData(const VertexData *vertex_data, const VertexData *blend_data,
                                                   const Mesh::IndexMap *indexMap) {
        // Get the bone index element
        assert(vertex_data);

        const VertexData *data = blend_data;
        const unsigned int prev_size = mVertexCount;
        mVertexCount += (unsigned int) data->vertexCount;
        Vector3 *tmp_vert = new Vector3[mVertexCount];
        if (mVertexBuffer) {
            memcpy(tmp_vert, mVertexBuffer, sizeof(Vector3) * prev_size);
            delete[] mVertexBuffer;
        }
        mVertexBuffer = tmp_vert;

        // Get the positional buffer element
        {
            const VertexElement *posElem = data->vertexDeclaration->findElementBySemantic(VES_POSITION);
            assert(posElem);
            HardwareVertexBufferSharedPtr vbuf = data->vertexBufferBinding->getBuffer(posElem->getSource());
            const unsigned int vSize = (unsigned int) vbuf->getVertexSize();

            unsigned char *vertex = static_cast<unsigned char *>(vbuf->lock(HardwareBuffer::HBL_READ_ONLY));
            float *pReal;
            Vector3 *curVertices = &mVertexBuffer[prev_size];
            const unsigned int vertexCount = (unsigned int) data->vertexCount;
            for (unsigned int j = 0; j < vertexCount; ++j) {
                posElem->baseVertexPointerToElement(vertex, &pReal);
                vertex += vSize;

                curVertices->x = (*pReal++);
                curVertices->y = (*pReal++);
                curVertices->z = (*pReal++);

                *curVertices = mTransform * (*curVertices);

                curVertices++;
            }
            vbuf->unlock();
        }
        {
            const VertexElement *bneElem = vertex_data->vertexDeclaration->findElementBySemantic(VES_BLEND_INDICES);
            assert(bneElem);

            HardwareVertexBufferSharedPtr vbuf = vertex_data->vertexBufferBinding->getBuffer(bneElem->getSource());
            const unsigned int vSize = (unsigned int) vbuf->getVertexSize();
            unsigned char *vertex = static_cast<unsigned char *>(vbuf->lock(HardwareBuffer::HBL_READ_ONLY));

            unsigned char *pBone;

            if (!mBoneIndex)
                mBoneIndex = new BoneIndex();
            BoneIndex::iterator i;

            Vector3 *curVertices = &mVertexBuffer[prev_size];

            const unsigned int vertexCount = (unsigned int) vertex_data->vertexCount;
            for (unsigned int j = 0; j < vertexCount; ++j) {
                bneElem->baseVertexPointerToElement(vertex, &pBone);
                vertex += vSize;

                const unsigned char currBone = (indexMap) ? (*indexMap)[*pBone] : *pBone;
                i = mBoneIndex->find(currBone);
                Vector3Array *l = 0;
                if (i == mBoneIndex->end()) {
                    l = new Vector3Array;
                    mBoneIndex->emplace(currBone, l);
                } else {
                    l = i->second;
                }

                l->push_back(*curVertices);

                curVertices++;
            }
            vbuf->unlock();
        }
    }

//------------------------------------------------------------------------------------------------
    void VertexIndexToShape::addIndexData(IndexData *data, const unsigned int offset) {
        const unsigned int prev_size = mIndexCount;
        mIndexCount += (unsigned int) data->indexCount;

        unsigned int *tmp_ind = new unsigned int[mIndexCount];
        if (mIndexBuffer) {
            memcpy(tmp_ind, mIndexBuffer, sizeof(unsigned int) * prev_size);
            delete[] mIndexBuffer;
        }
        mIndexBuffer = tmp_ind;

        const unsigned int numTris = (unsigned int) data->indexCount / 3;
        HardwareIndexBufferSharedPtr ibuf = data->indexBuffer;
        const bool use32bitindexes = (ibuf->getType() == HardwareIndexBuffer::IT_32BIT);
        unsigned int index_offset = prev_size;

        if (use32bitindexes) {
            const unsigned int *pInt = static_cast<unsigned int *>(ibuf->lock(HardwareBuffer::HBL_READ_ONLY));
            for (unsigned int k = 0; k < numTris; ++k) {
                mIndexBuffer[index_offset++] = offset + *pInt++;
                mIndexBuffer[index_offset++] = offset + *pInt++;
                mIndexBuffer[index_offset++] = offset + *pInt++;
            }
            ibuf->unlock();
        } else {
            const unsigned short *pShort = static_cast<unsigned short *>(ibuf->lock(HardwareBuffer::HBL_READ_ONLY));
            for (unsigned int k = 0; k < numTris; ++k) {
                mIndexBuffer[index_offset++] = offset + static_cast<unsigned int>(*pShort++);
                mIndexBuffer[index_offset++] = offset + static_cast<unsigned int>(*pShort++);
                mIndexBuffer[index_offset++] = offset + static_cast<unsigned int>(*pShort++);
            }
            ibuf->unlock();
        }
    }

//------------------------------------------------------------------------------------------------
    Real VertexIndexToShape::getRadius() {
        if (mBoundRadius == (-1)) {
            getSize();
            mBoundRadius = (std::max(mBounds.x, std::max(mBounds.y, mBounds.z)) * 0.5);
        }
        return mBoundRadius;
    }

//------------------------------------------------------------------------------------------------
    Vector3 VertexIndexToShape::getSize() {
        const unsigned int vCount = getVertexCount();
        if (mBounds == Vector3(-1, -1, -1) && vCount > 0) {

            const Vector3 *const v = getVertices();

            Vector3 vmin(v[0]);
            Vector3 vmax(v[0]);

            for (unsigned int j = 1; j < vCount; j++) {
                vmin.x = std::min(vmin.x, v[j].x);
                vmin.y = std::min(vmin.y, v[j].y);
                vmin.z = std::min(vmin.z, v[j].z);

                vmax.x = std::max(vmax.x, v[j].x);
                vmax.y = std::max(vmax.y, v[j].y);
                vmax.z = std::max(vmax.z, v[j].z);
            }

            mBounds.x = vmax.x - vmin.x;
            mBounds.y = vmax.y - vmin.y;
            mBounds.z = vmax.z - vmin.z;
        }

        return mBounds;
    }

//------------------------------------------------------------------------------------------------
    boost::shared_ptr<btConvexHullShape> VertexIndexToShape::createConvex(
            const boost::shared_ptr<BulletMemoryContainer::BulletMemoryContainerManager> &memoryContainerManager
    ) {
        assert(mVertexCount && (mIndexCount >= 6) &&
               ("Mesh must have some vertices and at least 6 indices (2 triangles)"));

//            btConvexHullShape *shape = new btConvexHullShape(
//                    (btScalar *) &mVertexBuffer[0].x,
//                    mVertexCount,
//                    sizeof(Vector3)
//            );
        auto shape = memoryContainerManager->makeSharedPtr<btConvexHullShape>(
                (btScalar *) &mVertexBuffer[0].x,
                mVertexCount,
                (int) sizeof(Vector3)
        );

        shape->setLocalScaling(convert(mScale));

        return shape;
    }

//------------------------------------------------------------------------------------------------
    boost::shared_ptr<btBvhTriangleMeshShape> VertexIndexToShape::createTrimesh(
            const boost::shared_ptr<BulletMemoryContainer::BulletMemoryContainerManager> &memoryContainerManager
    ) {
        assert(mVertexCount && (mIndexCount >= 6) &&
               ("Mesh must have some vertices and at least 6 indices (2 triangles)"));

        unsigned int numFaces = mIndexCount / 3;

//            btTriangleMesh *trimesh = new btTriangleMesh();
        auto trimesh = memoryContainerManager->newRawPtr<btTriangleMesh>();
        unsigned int *indices = mIndexBuffer;
        Vector3 *vertices = mVertexBuffer;

        btVector3 vertexPos[3];
        for (unsigned int n = 0; n < numFaces; ++n) {
            {
                const Vector3 &vec = vertices[*indices];
                vertexPos[0][0] = vec.x;
                vertexPos[0][1] = vec.y;
                vertexPos[0][2] = vec.z;
            }
            {
                const Vector3 &vec = vertices[*(indices + 1)];
                vertexPos[1][0] = vec.x;
                vertexPos[1][1] = vec.y;
                vertexPos[1][2] = vec.z;
            }
            {
                const Vector3 &vec = vertices[*(indices + 2)];
                vertexPos[2][0] = vec.x;
                vertexPos[2][1] = vec.y;
                vertexPos[2][2] = vec.z;
            }

            indices += 3;

            trimesh->addTriangle(vertexPos[0], vertexPos[1], vertexPos[2]);
        }

        const bool useQuantizedAABB = true;
//            btBvhTriangleMeshShape *shape = new btBvhTriangleMeshShape(trimesh, useQuantizedAABB);
        auto shape = memoryContainerManager->makeSharedPtr<btBvhTriangleMeshShape>(trimesh, useQuantizedAABB);

        shape->setLocalScaling(convert(mScale));

        return shape;
    }

//------------------------------------------------------------------------------------------------
    VertexIndexToShape::~VertexIndexToShape() {
        delete[] mVertexBuffer;
        delete[] mIndexBuffer;

        if (mBoneIndex) {
            for (auto &i: *mBoneIndex) {
                delete i.second;
            }
            delete mBoneIndex;
        }
    }

//------------------------------------------------------------------------------------------------
    VertexIndexToShape::VertexIndexToShape(const Affine3 &transform)
            : mVertexBuffer(nullptr), mIndexBuffer(nullptr), mVertexCount(0), mIndexCount(0),
              mBounds(Vector3(-1, -1, -1)),
              mBoundRadius(-1), mBoneIndex(nullptr), mTransform(transform), mScale(1) {
    }

//------------------------------------------------------------------------------------------------
    VertexIndexToShape::VertexIndexToShape(const Entity *entity, const Affine3 &transform)
            : VertexIndexToShape(transform) {
        addEntity(entity, transform);
    }

//------------------------------------------------------------------------------------------------
    VertexIndexToShape::VertexIndexToShape(Renderable *rend, const Affine3 &transform)
            : VertexIndexToShape(transform) {
        RenderOperation op;
        rend->getRenderOperation(op);
        addStaticVertexData(op.vertexData);
        if (op.useIndexes)
            addIndexData(op.indexData);
    }

//------------------------------------------------------------------------------------------------
    void VertexIndexToShape::addEntity(const Entity *entity, const Affine3 &transform) {
        // Each entity added need to reset size and radius
        // next time getRadius and getSize are asked, they're computed.
        mBounds = Vector3(-1, -1, -1);
        mBoundRadius = -1;

        auto node = entity->getParentSceneNode();
        mTransform = transform;
        mScale = node ? node->getScale() : Vector3(1, 1, 1);

        bool hasSkeleton = entity->hasSkeleton();

        if (entity->getMesh()->sharedVertexData) {
            if (hasSkeleton)
                addAnimatedVertexData(entity->getMesh()->sharedVertexData, entity->_getSkelAnimVertexData(),
                                      &entity->getMesh()->sharedBlendIndexToBoneIndexMap);
            else
                addStaticVertexData(entity->getMesh()->sharedVertexData);
        }

        for (unsigned int i = 0; i < entity->getNumSubEntities(); ++i) {
            SubMesh *sub_mesh = entity->getSubEntity(i)->getSubMesh();

            if (!sub_mesh->useSharedVertices) {
                addIndexData(sub_mesh->indexData, mVertexCount);

                if (hasSkeleton)
                    addAnimatedVertexData(sub_mesh->vertexData, entity->getSubEntity(i)->_getSkelAnimVertexData(),
                                          &sub_mesh->blendIndexToBoneIndexMap);
                else
                    addStaticVertexData(sub_mesh->vertexData);
            } else {
                addIndexData(sub_mesh->indexData);
            }
        }
    }

//------------------------------------------------------------------------------------------------
    void VertexIndexToShape::addMesh(const MeshPtr &mesh, const Affine3 &transform) {
        // Each entity added need to reset size and radius
        // next time getRadius and getSize are asked, they're computed.
        mBounds = Vector3(-1, -1, -1);
        mBoundRadius = -1;

        mTransform = transform;

        if (mesh->hasSkeleton())
            LogManager::getSingleton().logWarning(
                    "Mesh " + mesh->getName() + " has a skeleton but added non animated");

        if (mesh->sharedVertexData) {
            VertexIndexToShape::addStaticVertexData(mesh->sharedVertexData);
        }

        for (unsigned int i = 0; i < mesh->getNumSubMeshes(); ++i) {
            SubMesh *sub_mesh = mesh->getSubMesh(i);

            if (!sub_mesh->useSharedVertices) {
                VertexIndexToShape::addIndexData(sub_mesh->indexData, mVertexCount);
                VertexIndexToShape::addStaticVertexData(sub_mesh->vertexData);
            } else {
                VertexIndexToShape::addIndexData(sub_mesh->indexData);
            }
        }
    }


} // Bullet